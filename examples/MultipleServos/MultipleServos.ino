/*
 * MultipleServos - Scan and control multiple servos
 * 
 * Automatically finds all connected Hiwonder servos and makes them
 * perform a coordinated sweep motion together.
 * Connect servo data lines to Serial2 TX pin.
 */

#include <HiBusServo.h>

const int MAX_SERVOS = 20;        // Maximum number of servos to scan for
const int SCAN_START_ID = 1;      // First servo ID to check
const int SCAN_END_ID = 20;       // Last servo ID to check
const int MOVE_TIME = 2000;       // Movement duration (ms)
const int PAUSE_TIME = MOVE_TIME + 10;      // Pause between moves (ms)

HiBusServo servo;
int found_servos[MAX_SERVOS];     // Array to store found servo IDs
int servo_count = 0;              // Number of servos found

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Multiple Servos Demo");
    Serial.println("Scanning for connected servos...");
    
    servo.begin(Serial2);
    delay(1000);
    
    // Scan for connected servos
    scan_for_servos();
    show_all_status();
    
    if (servo_count == 0) {
        Serial.println("No servos found! Check connections.");
        while(1); // Stop here if no servos found
    }
    
    Serial.print("Found ");
    Serial.print(servo_count);
    Serial.println(" servo(s). Starting coordinated sweep...");
    Serial.println();
}

void loop() {
    // Sweep all servos left
    Serial.println("→ All servos moving LEFT");
    move_all_servos(-90.0);
    delay(PAUSE_TIME);
    show_all_status();

    
    // Sweep all servos right  
    Serial.println("→ All servos moving RIGHT");
    move_all_servos(90.0);
    delay(PAUSE_TIME);
    show_all_status();

    
    // Return all servos to center
    Serial.println("→ All servos moving CENTER");
    move_all_servos(0.0);
    delay(PAUSE_TIME);
    show_all_status();
    
    // Show status of all servos
    Serial.println("--- Cycle complete ---");
    Serial.println();
    delay(2000);
}

// Scan for connected servos by trying to read position and voltage from each ID
void scan_for_servos() {
    servo_count = 0;
    
    for (int id = SCAN_START_ID; id <= SCAN_END_ID; id++) {
        Serial.print("Checking ID ");
        Serial.print(id);
        Serial.print("... ");
        
        // Try to read position to see if servo responds
        int pos = servo.readPosition(id);
        
        if (pos >= 0) {
            // Servo found via position reading
            found_servos[servo_count] = id;
            servo_count++;
            Serial.println("✓ Found! (position)");
        } else {
            // If position read fails, try reading voltage as a fallback
            int voltage = servo.readVin(id);
            
            if (voltage != -2048) {
                // Servo found via voltage reading
                found_servos[servo_count] = id;
                servo_count++;
                Serial.println("✓ Found! (voltage)");
            } else {
                // Try temperature as a last resort
                int temp = servo.readTemperature(id);
                
                if (temp != -1) {
                    // Servo found via temperature reading
                    found_servos[servo_count] = id;
                    servo_count++;
                    Serial.println("✓ Found! (temperature)");
                } else {
                    Serial.println("✗ Not found");
                }
            }
        }
        
        delay(50); // Small delay between scans
    }
}

// Move all found servos to the same position
void move_all_servos(float target_degrees) {
    for (int i = 0; i < servo_count; i++) {
        servo.moveTo(found_servos[i], target_degrees, MOVE_TIME);
    }
}

// Show status of all found servos
void show_all_status() {
    Serial.println("Servo Status:");
    
    for (int i = 0; i < servo_count; i++) {
        int id = found_servos[i];
        Serial.print("ID ");
        Serial.print(id);
        Serial.print(": ");
        
        int pos = servo.readPosition(id);
        int voltage = servo.readVin(id);
        int temp = servo.readTemperature(id);
        
        if (pos >= 0) {
            float degrees = (pos - 500) * (120.0 / 500.0);
            Serial.print(degrees, 1);
            Serial.print("° ");
        }
        
        if (voltage != -2048) {
            Serial.print(voltage / 1000.0, 1);
            Serial.print("V ");
        }
        
        if (temp != -1) {
            Serial.print(temp);
            Serial.print("°C");
        }
        
        Serial.println();
    }
}
