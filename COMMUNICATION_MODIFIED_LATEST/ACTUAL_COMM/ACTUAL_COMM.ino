#include "communication.h"

/* ===================================================== 
   CONFIGURATION: Set this ID for each car
   ===================================================== */
#define MY_VEHICLE_ID 1   // Must match the AprilTag ID on this specific car
#define BUZZER_PIN    13  // GPIO for the collision warning buzzer

// ---> NEW: IEKF State Struct <---
// This cleanly packages the perfect Global Time alongside the coordinates
struct IEKF_Measurement {
    uint64_t timestamp_us;
    float x;
    float y;
    float yaw;
    bool  is_new;
};

IEKF_Measurement vision_meas = {0, 0.0f, 0.0f, 0.0f, false};

// Watchdog variables
uint64_t last_valid_vision_time_us = 0;
uint32_t latest_vision_latency_us = 0; // Tracks "Glass-to-Glass" data age
bool is_location_valid = false;

void setup() {
    Serial.begin(115200);
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Start the V2V Engine
    Comm.begin(); 
    
    Serial.printf("IEKF Car Node Started. Monitoring Tag ID: %d\n", MY_VEHICLE_ID);
}

void loop() {
    // 1. CHECK THE MAILBOX
    if (Comm.new_vision_data) {
        Comm.new_vision_data = false; // Reset the flag
        
        // Filter: Is this packet specifically for ME?
        if (Comm.last_vision.target_id == MY_VEHICLE_ID) {
            
            // ---> SAVE DIRECTLY FOR THE KALMAN FILTER <---
            // The timestamp is the exact Global Time the camera shutter clicked
            vision_meas.timestamp_us = Comm.last_vision.t_us;
            vision_meas.x = Comm.last_vision.x;
            vision_meas.y = Comm.last_vision.y;
            vision_meas.yaw = Comm.last_vision.yaw;
            vision_meas.is_new = true;
            
            // Calculate exact End-to-End Latency
            uint64_t now_us = Comm.get_global_time();
            if (now_us > vision_meas.timestamp_us) {
                latest_vision_latency_us = (uint32_t)(now_us - vision_meas.timestamp_us);
            }
            
            // Mark local hardware timestamp so Watchdog knows data is fresh
            last_valid_vision_time_us = esp_timer_get_time();
            is_location_valid = true;
            
            // Feed the Black Box so it shares this with the V2V Mesh
            Comm.update_state(vision_meas.x, vision_meas.y, vision_meas.yaw);
        }
    }

    // 2. RUN THE IEKF (If new data arrived)
    if (vision_meas.is_new) {
        vision_meas.is_new = false; // Mark as consumed so we don't process it twice
        
        // [PLACEHOLDER]: Execute your filter logic here using vision_meas.timestamp_us
        // run_iekf_update(vision_meas.x, vision_meas.y, vision_meas.yaw, vision_meas.timestamp_us);
    }

    // 3. WATCHDOG SAFETY CHECK
    // If we haven't received an update for our ID in > 500ms, assume the camera lost us
    if (is_location_valid && (esp_timer_get_time() - last_valid_vision_time_us > 500000)) {
        is_location_valid = false;
        digitalWrite(BUZZER_PIN, LOW); // Fail-safe: turn off buzzer if sensor is lost
    }

    // 4. COLLISION BRAIN (Application Layer)
    check_v2v_safety();

    // 5. RUN THE V2V ENGINE
    Comm.poll();
    Comm.prune_neighbors();

    // 6. DIAGNOSTIC STATUS (Every 1 second)
    static uint32_t last_print = 0;
    if (millis() - last_print > 1000) {
        last_print = millis();
        print_mesh_status();
    }
}

/* ===================================================== 
   PHASE 9: COLLISION LOGIC
   ===================================================== */
void check_v2v_safety() {
    // Only calculate collisions if we actually know where we are
    if (!is_location_valid) {
        digitalWrite(BUZZER_PIN, LOW);
        return; 
    }

    float safe_distance = 0.5f; // 0.5 meters (50cm) threshold
    bool danger = false;

    // Iterate through all cars discovered by the Black Box mesh
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (Comm.neighbors[i].active) {
            // Euclidean distance: sqrt(dx^2 + dy^2)
            float dx = Comm.neighbors[i].x - vision_meas.x;
            float dy = Comm.neighbors[i].y - vision_meas.y;
            float distance = sqrt(dx*dx + dy*dy);

            if (distance < safe_distance) {
                danger = true;
                break;
            }
        }
    }

    digitalWrite(BUZZER_PIN, danger ? HIGH : LOW);
}

/* ===================================================== 
   MESH DIAGNOSTICS
   ===================================================== */
void print_mesh_status() {
    Serial.printf("\n--- CAR [%d] | ROLE: %s ---\n", MY_VEHICLE_ID, Comm.get_current_role());
    
    if (is_location_valid) {
        Serial.printf("My Pos: (%.2fm, %.2fm) | Latency: %u ms | Synced: %s\n", 
                      vision_meas.x, vision_meas.y, 
                      (latest_vision_latency_us / 1000), 
                      Comm.is_synced() ? "YES" : "NO");
    } else {
        Serial.printf("My Pos: LOST SENSOR VIEW | Synced: %s\n", Comm.is_synced() ? "YES" : "NO");
    }
    
    Serial.print("Neighbors in Mesh: ");
    int active_cars = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (Comm.neighbors[i].active) {
            active_cars++;
            Serial.printf("\n  [Car %02X:%02X] at (%.2fm, %.2fm)", 
                           Comm.neighbors[i].mac[4], Comm.neighbors[i].mac[5], 
                           Comm.neighbors[i].x, Comm.neighbors[i].y);
        }
    }
    
    if (active_cars == 0) Serial.print("NONE");
    Serial.println("\n-------------------------");
}