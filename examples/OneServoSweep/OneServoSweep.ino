/*
 * OneServoSweep - Simple servo sweep example
 * 
 * Makes a Hiwonder servo sweep left, right, center, and repeat.
 * Connect servo data line to Serial2 TX pin.
 */

#include <HiBusServo.h>

const int SERVO_ID = 1;        // Change to match your servo ID
const int MOVE_TIME = 1000;    // Movement duration (ms)
const int PAUSE_TIME = 1500;   // Pause between moves (ms)

HiBusServo servo;

void setup() {
    Serial.begin(115200);
    Serial.println("Servo Sweep Demo");
    
    servo.begin(Serial2);
    delay(100);
}

void loop() {
    // Sweep left
    Serial.println("→ Left");
    servo.moveTo(SERVO_ID, -90.0, MOVE_TIME);
    delay(PAUSE_TIME);
    
    // Sweep right
    Serial.println("→ Right");
    servo.moveTo(SERVO_ID, 90.0, MOVE_TIME);
    delay(PAUSE_TIME);
    
    // Return to center
    Serial.println("→ Center");
    servo.moveTo(SERVO_ID, 0.0, MOVE_TIME);
    delay(PAUSE_TIME);
    
    // Show status
    show_status();
    delay(2000);
}

void show_status() {
    int pos = servo.readPosition(SERVO_ID);
    int voltage = servo.readVin(SERVO_ID);
    int temp = servo.readTemperature(SERVO_ID);
    
    if (pos >= 0) {
        float degrees = (pos - 500) * (120.0 / 500.0);
        Serial.print("Position: ");
        Serial.print(degrees, 1);
        Serial.println("°");
    }
    
    if (voltage != -2048) {
        Serial.print("Voltage: ");
        Serial.print(voltage / 1000.0, 1);
        Serial.println("V");
    }
    
    if (temp != -1) {
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println("°C");
    }
    
    Serial.println("---");
}
