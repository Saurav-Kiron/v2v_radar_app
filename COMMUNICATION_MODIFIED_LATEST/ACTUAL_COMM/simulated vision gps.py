import cv2
import numpy as np
import math
import struct
import time
import serial

class AprilTagTracker:
    def __init__(self, ref_id=0):
        self.ref_id = ref_id
        self.ref_ever_detected = True 
        # Simulated "World Center" in a 640x480 virtual space
        self.last_ref_pose = [float(ref_id), 320.0, 240.0, 0.0] 
        self.last_ppm = 150.0  # 150 pixels = 1 meter

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_simulated_poses(self):
        """
        Generates fake data for two vehicles: ID 1 and ID 3.
        """
        t = time.time()
        
        # --- VEHICLE 1: Orbital Motion ---
        v1_id = 1.0
        v1_radius = 1.2
        v1_speed = 0.7
        v1_x = v1_radius * math.cos(t * v1_speed)
        v1_y = v1_radius * math.sin(t * v1_speed)
        v1_yaw = self._normalize_angle(t * v1_speed + math.pi/2)

        # --- VEHICLE 3: Linear/Oscillating Motion ---
        v3_id = 3.0
        v3_x = 1.5 * math.sin(t * 0.5) # Moves left/right between -1.5m and 1.5m
        v3_y = 1.0                     # Fixed at 1 meter "North"
        v3_yaw = 0.0 if math.cos(t * 0.5) > 0 else math.pi # Flip facing based on direction

        # Combine into the results list
        moving_results = [
            [v1_id, round(v1_x, 4), round(v1_y, 4), round(v1_yaw, 4)],
            [v3_id, round(v3_x, 4), round(v3_y, 4), round(v3_yaw, 4)]
        ]

        # Create Visual Data for the UI
        ref_px, ref_py = self.last_ref_pose[1], self.last_ref_pose[2]
        visual_data = [
            {'id': self.ref_id, 'center': np.array([ref_px, ref_py]), 'yaw': 0.0},
            {'id': 1, 'center': np.array([ref_px + (v1_x * self.last_ppm), ref_py + (v1_y * self.last_ppm)]), 'yaw': v1_yaw},
            {'id': 3, 'center': np.array([ref_px + (v3_x * self.last_ppm), ref_py + (v3_y * self.last_ppm)]), 'yaw': v3_yaw}
        ]

        # Standard Output structure
        output = [1, len(moving_results), [self.last_ref_pose] + moving_results]
        return output, visual_data

def visualize_sim(frame, visual_data, output):
    all_poses = output[2]
    ref_id = all_poses[0][0]

    for tag in visual_data:
        tid = int(tag['id'])
        ctr = (int(tag['center'][0]), int(tag['center'][1]))
        color = (0, 0, 255) if tid == ref_id else (0, 255, 0)
        
        cv2.circle(frame, ctr, 10, color, -1)
        # Heading line
        end_pt = (int(ctr[0] + 30 * math.cos(tag['yaw'])), int(ctr[1] + 30 * math.sin(tag['yaw'])))
        cv2.line(frame, ctr, end_pt, (255, 255, 255), 2)
        
        label = f"ID:{tid}"
        if tid != ref_id:
            for p in all_poses[1:]:
                if int(p[0]) == tid:
                    label += f" [{p[1]}m, {p[2]}m]"
        
        cv2.putText(frame, label, (ctr[0]+15, ctr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def serialize_v2v_packet(p):
    header = struct.pack('<BB', 0xAA, 0x55)
    payload = struct.pack('<BBQfff', 
                          p['sender_id'], p['target_id'], p['t_us'], 
                          p['x'], p['y'], p['yaw'])
    return header + payload

def main():
    # ---------------- CONFIG ----------------
    SERIAL_PORT = 'COM8'   # Replace with your ESP port
    BAUDRATE = 921600
    MY_VEHICLE_ID = 1       # The ID of this "Simulator" node
    SEND_INTERVAL = 0.010   # 100Hz
    # ----------------------------------------

    tracker = AprilTagTracker(ref_id=0)
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        print(f"[OK] Connected to Bridge ESP on {SERIAL_PORT}")
    except Exception as e:
        print(f"[ERROR] Serial Connection Failed: {e}")
        return

    last_tx_time = 0.0
    print("V2V Simulation (Headless) running. Press 'q' on window to stop.")

    try:
        while True:
            # 1. Create a blank black canvas (No Camera Feed)
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "SIMULATION RUNNING (HEADLESS)", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

            # 2. Hardware Time Handshake
            ser.write(b'T')
            ser.flush()
            t_cap_mesh_time = 0
            
            # Read Serial Response
            timeout = time.time() + 0.04
            while time.time() < timeout:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("TIME:"):
                        try:
                            t_cap_mesh_time = int(line.split(":")[1])
                        except: pass
                        break
                time.sleep(0.001)

            # 3. Only proceed if we have a valid mesh time
            if t_cap_mesh_time == 0:
                cv2.putText(frame, "WAITING FOR ESP MESH SYNC...", (180, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # 4. Generate & Process Simulated Data
                output, visual_data = tracker.get_simulated_poses()
                
                # 5. Visualize on the blank frame
                visualize_sim(frame, visual_data, output)

                # 6. Send Telemetry
                if (time.monotonic() - last_tx_time) >= SEND_INTERVAL:
                    moving_tags = output[2][1:] # Skip Ref Tag
                    for tag in moving_tags:
                        packet = {
                            'sender_id': MY_VEHICLE_ID,
                            'target_id': int(tag[0]),
                            't_us': t_cap_mesh_time,
                            'x': float(tag[1]),
                            'y': float(tag[2]),
                            'yaw': float(tag[3])
                        }
                        ser.write(serialize_v2v_packet(packet))
                    ser.flush()
                    last_tx_time = time.monotonic()

            cv2.imshow("V2V Virtual Field", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        ser.close()
        cv2.destroyAllWindows()
        print("Clean shutdown.")

if __name__ == "__main__":
    main()