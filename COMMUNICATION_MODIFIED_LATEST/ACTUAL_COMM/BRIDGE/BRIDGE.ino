#include "communication.h"

void setup() {
    // 1. MUST match Python script's BAUDRATE (921600)
    Serial.begin(921600); 
    
    // 2. Initialize the Black Box to participate in time sync
    Comm.begin(); 
    
    Serial.println("V2V Bridge Active: Awaiting Time Requests or Vision Data...");
}

void loop() {
    // 3. Keep the V2V Engine spinning (Handles time sync and leader election)
    Comm.poll();

    // 4. Handle Incoming Serial Data from Python
    if (Serial.available() > 0) { 
        uint8_t peek_byte = Serial.peek();
        
        // ---> NEW: 4A. THE TIME REQUEST HANDLER <---
        // Python sends 'T' the exact microsecond the camera captures a frame
        if (peek_byte == 'T') { 
            Serial.read(); // Consume the 'T' from the buffer
            
            // Only send a valid time if we are safely synced to the mesh
            if (Comm.is_synced()) {
                Serial.printf("TIME:%llu\n", Comm.get_global_time());
            } else {
                Serial.println("TIME:0"); // Tell Python the mesh isn't ready
            }
        } 
        
        // ---> 4B. THE VISION DATA INJECTOR <---
        // Look for the Python Binary Header [0xAA, 0x55] (24 bytes total)
        else if (peek_byte == 0xAA && Serial.available() >= 24) {
            uint8_t header[2];
            Serial.readBytes(header, 2);
            
            if (header[0] == 0xAA && header[1] == 0x55) {
                VisionPacket vp;
                Serial.readBytes((char*)&vp, sizeof(vp));

                // ---> SAFE INJECTION <---
                // Only broadcast to the cars if our clock is safely synced to the mesh
                if (Comm.is_synced()) {
                    Comm.handle_vision_data(vp);
                }
            }
        } 
        
        // ---> 4C. JUNK CLEARING <---
        // If it's not a 'T' and not the start of a vision packet, throw it away
        else if (peek_byte != 0xAA && peek_byte != 'T') {
            Serial.read(); 
        }
    }
}