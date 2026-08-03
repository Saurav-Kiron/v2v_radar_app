import cv2
import numpy as np
import math
import struct
from pupil_apriltags import Detector
import time
import serial

class AprilTagTracker:
    def __init__(self, ref_id=0):
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.ref_id = ref_id
        
        # Persistent State
        self.ref_ever_detected = False
        self.last_ref_pose = [float(ref_id), 0.0, 0.0, 0.0]  # [id, px, py, yaw]
        self.last_ppm = 1.0  # Pixels Per Meter

    def _get_robust_yaw(self, corners):
        """Uses Pupil-AprilTag corner layout: [p1, p2, p3, p4] where p1->p2 is top."""
        v_top = corners[1] - corners[0]
        v_bottom = corners[2] - corners[3]
        v_avg = (v_top + v_bottom) * 0.5
        return math.atan2(v_avg[1], v_avg[0])

    def _normalize_angle(self, angle):
        """Wraps angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def detect_apriltag_relative_pose(self, frame, reference_tag_size_m, moving_tag_size_m) -> tuple:
        """
        Computes 2D relative pose. moving_tag_size_m is unused to maintain 
        world-scale consistency tied strictly to the reference tag.
        """
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        detections = self.detector.detect(gray)
        
        # Confidence gate for robust tracking
        raw_tags = {d.tag_id: d for d in detections if d.decision_margin > 12}
        new_ref_found = self.ref_id in raw_tags
        
        # 1. Update Reference & Scale Authority
        if new_ref_found:
            self.ref_ever_detected = True
            det = raw_tags[self.ref_id]
            
            center = det.center 
            yaw = self._get_robust_yaw(det.corners)
            self.last_ref_pose = [float(self.ref_id), center[0], center[1], yaw]
            
            # Calculate and sanity-check Pixels Per Meter (PPM)
            side_px = (np.linalg.norm(det.corners[1] - det.corners[0]) + 
                       np.linalg.norm(det.corners[2] - det.corners[3])) / 2.0
            new_ppm = side_px / reference_tag_size_m
            
            # Clamp insane PPM changes to prevent bad frames from corrupting world scale
            if 0.2 < new_ppm < 5000:
                self.last_ppm = new_ppm
        
        # Defensive Return: Consistent shape even if ref never seen
        if not self.ref_ever_detected:
            return [-1, 0, []], []

        # 2. Compute Relative SE(2) Poses
        ref_px, ref_py, ref_yaw = self.last_ref_pose[1:]
        moving_results = []
        visual_data = [] 

        for tag_id, det in raw_tags.items():
            m_center = det.center
            m_yaw = self._get_robust_yaw(det.corners)
            visual_data.append({'id': tag_id, 'center': m_center, 'yaw': m_yaw})

            if tag_id == self.ref_id:
                continue

            # Transform to Reference Frame
            dx_pix = m_center[0] - ref_px
            dy_pix = m_center[1] - ref_py
            cos_t, sin_t = math.cos(-ref_yaw), math.sin(-ref_yaw)
            
            x_m = (dx_pix * cos_t - dy_pix * sin_t) / self.last_ppm
            y_m = (dx_pix * sin_t + dy_pix * cos_t) / self.last_ppm
            rel_yaw = self._normalize_angle(m_yaw - ref_yaw)
            
            moving_results.append([float(tag_id), round(x_m, 4), round(y_m, 4), round(rel_yaw, 4)])

        # Final structure: [ref_found_bool, num_moving, all_detections_list]
        output = [
            1 if new_ref_found else 0, 
            len(moving_results), 
            [self.last_ref_pose] + moving_results
        ]
        return output, visual_data

def visualize_frame(frame, visual_data, output):
    """Spatially accurate visualization with ID and relative metric coordinates."""
    if output[0] == -1:
        return

    all_poses = output[2]
    ref_id = all_poses[0][0]

    for tag in visual_data:
        tid = tag['id']
        ctr = (int(tag['center'][0]), int(tag['center'][1]))
        yaw = tag['yaw']
        
        color = (0, 0, 255) if tid == ref_id else (0, 255, 0)
        cv2.circle(frame, ctr, 5, color, -1)
        
        # Orientation Arrow
        end_pt = (int(ctr[0] + 40 * math.cos(yaw)), int(ctr[1] + 40 * math.sin(yaw)))
        cv2.arrowedLine(frame, ctr, end_pt, color, 2)
        
        # Annotation: ID + (X, Y) relative to reference
        label = f"ID:{int(tid)}"
        if tid != ref_id:
            for p in all_poses[1:]:
                if p[0] == tid:
                    yaw_deg = math.degrees(p[3])
                    label += f" ({p[1]:.2f}m, {p[2]:.2f}m, {yaw_deg:.1f}°)"

        cv2.putText(frame, label, (ctr[0]+10, ctr[1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    cv2.imshow("Pupil-AprilTag Tracker", frame)

# --- PACKET GENERATION ---

def create_v2v_packets(detection_output, t_capture_us, sender_id):
    """
    Converts detection output into individual vehicle packets.
    Format: [sender_id, target_id, t_us, x, y, yaw]
    """
    if detection_output[0] == -1 or detection_output[1] == 0:
        return []

    moving_tags = detection_output[2][1:] # Skip index 0 (Reference Tag)
    packets = []

    for tag in moving_tags:
        packet = {
            'sender_id': int(sender_id),
            'target_id': int(tag[0]),
            't_us': t_capture_us, # <--- This is now the true hardware Global Mesh Time!
            'x': float(tag[1]),
            'y': float(tag[2]),
            'yaw': float(tag[3])
        }
        packets.append(packet)
    
    return packets

# --- SERIALIZATION ---

def serialize_v2v_packet(p):
    """
    Serializes a single packet using struct.
    Format: < (little-endian), B (uint8), B (uint8), Q (uint64), fff (3x float32)
    Header: 0xAA, 0x55 (2 bytes)
    Total payload: 22 bytes
    """
    header = struct.pack('<BB', 0xAA, 0x55)
    payload = struct.pack('<BBQfff', 
                          p['sender_id'], 
                          p['target_id'], 
                          p['t_us'], 
                          p['x'], 
                          p['y'], 
                          p['yaw'])
    return header + payload

# --- TRANSMISSION & RATE LIMITING ---

def send_v2v_telemetry(ser, packets, last_send_time, interval_s=0.100):
    """
    Rate-limits and sends packets over USB Serial.
    Returns the updated last_send_time.
    """
    current_time = time.monotonic()
    
    if (current_time - last_send_time) < interval_s:
        return last_send_time

    if not ser or not ser.is_open:
        return current_time

    for p in packets:
        binary_data = serialize_v2v_packet(p)
        ser.write(binary_data)
    
    ser.flush() # Ensure data is sent to ESP32 immediately
    return current_time


def main():
    # ---------------- CONFIG ----------------
    REF_ID = 0
    REF_SIZE = 0.152
    MOV_SIZE = 0.10

    MY_VEHICLE_ID = 1
    SEND_INTERVAL = 0.010   # 10 ms
    PRINT_INTERVAL = 30

    CAMERA_INDEX = 0
    SERIAL_PORT = 'COM8'
    BAUDRATE = 921600
    # ----------------------------------------

    tracker = AprilTagTracker(ref_id=REF_ID)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {CAMERA_INDEX}")
        return
    else:
        print(f"[OK] Camera {CAMERA_INDEX} opened")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        print(f"[OK] Serial opened on {SERIAL_PORT}")
    except Exception as e:
        print("[ERROR] Failed to open serial port:", e)
        return

    last_tx_time = 0.0
    frame_count = 0

    print("Vision-GPS + V2V TX running. Press 'q' to exit.")

    try:
        while True:
            # 1️⃣ Capture frame
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Camera read failed")
                break

            # 2️⃣ ---> NEW: HARDWARE TIME POLLING <---
            # Instantly ask the Bridge ESP for the current mesh time
            ser.write(b'T')
            ser.flush()
            
            t_cap_mesh_time = 0
            
            # Read the response (allows up to 50ms for the serial buffer to respond)
            timeout = time.time() + 0.05 
            while time.time() < timeout:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith("TIME:"):
                        try:
                            t_cap_mesh_time = int(line.split(":")[1])
                        except ValueError:
                            pass
                        break  # Got the time, break out of reading loop
                    elif line:
                        # Print any other debug info coming from the ESP32
                        print(f"[ESP] {line}")
                else:
                    time.sleep(0.001) # Yield to CPU
            
            # If the ESP returned 0 (or timed out), the mesh isn't synced yet.
            if t_cap_mesh_time == 0:
                cv2.putText(frame, "WAITING FOR MESH SYNC...", (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Pupil-AprilTag Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue # Skip vision processing until we have a valid time

            # 3️⃣ Vision processing (Takes time, but our timestamp is already secured!)
            output, visual_data = tracker.detect_apriltag_relative_pose(
                frame,
                reference_tag_size_m=REF_SIZE,
                moving_tag_size_m=MOV_SIZE
            )

            # 4️⃣ Optional console debug
            if frame_count % PRINT_INTERVAL == 0:
                print(output)
            frame_count += 1

            # 5️⃣ Visualization
            visualize_frame(frame, visual_data, output)

            # 6️⃣ Packet creation using the polled MESH TIME
            packets = create_v2v_packets(output, t_cap_mesh_time, MY_VEHICLE_ID)

            # 7️⃣ Rate-limited serial transmission
            last_tx_time = send_v2v_telemetry(
                ser,
                packets,
                last_tx_time,
                SEND_INTERVAL
            )
            
            # 8️⃣ Exit handling
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print("[FATAL ERROR]", e)
    finally:
        cap.release()
        ser.close()
        cv2.destroyAllWindows()
        print("Clean shutdown")

if __name__ == "__main__":
    main()