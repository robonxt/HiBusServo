#include "test.h"

// Use Serial1 for communication with the servo
// IMPORTANT: Ensure your board has a Serial1 port (e.g., Mega, Leonardo)
LewanSoul_LX16A_Enhanced myServo(Serial2);

const uint8_t SERVO_ID = 5;
// IMPORTANT: Change this if you change the servo's baud rate for testing
const long SERVO_BAUD_RATE = 115200;

void printAllServoData(uint8_t id);

void setup() {
  // Serial port for debugging output to your computer
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial Monitor to connect

  // Serial port for communication with the servo bus
  myServo.begin(SERVO_BAUD_RATE);

  delay(1000);
  Serial.println("\n--- Servo Full Status Check ---");
}

void loop() {
  Serial.println("\n--- Reading All Values from Servo ID: " + String(SERVO_ID) + " ---");
  printAllServoData(SERVO_ID);

  Serial.println("\n---------------------------------");
  delay(5000);  // Wait 5 seconds before repeating the check
}

void printAllServoData(uint8_t id) {
  // --- Position ---
  int16_t pos_raw = myServo.getPosition(id);
  if (pos_raw != -1) {
    Serial.println("Position (raw): " + String(pos_raw));
  } else {
    Serial.println("Position (raw): TIMEOUT/ERROR");
  }

  float pos_deg = myServo.getPositionInDegrees(id);
  if (pos_deg != -999.0) {
    Serial.println("Position (deg): " + String(pos_deg));
  } else {
    Serial.println("Position (deg): TIMEOUT/ERROR");
  }

  // --- Physical State ---
  int16_t vin = myServo.getVoltage(id);
  if (vin != -1) {
    Serial.println("Voltage: " + String(vin) + " mV");
  } else {
    Serial.println("Voltage: TIMEOUT/ERROR");
  }

  int8_t temp = myServo.getTemperature(id);
  if (temp != -1) {
    Serial.println("Temperature: " + String(temp) + " C");
  } else {
    Serial.println("Temperature: TIMEOUT/ERROR");
  }

  // --- Mode ---
  uint8_t mode;
  int16_t speed;
  if (myServo.getMode(id, mode, speed)) {
    Serial.print("Mode: " + String(mode));
    if (mode == 0) Serial.println(" (Servo Mode)");
    if (mode == 1) Serial.println(" (Motor Mode), Speed: " + String(speed));
  } else {
    Serial.println("Mode: TIMEOUT/ERROR");
  }

  // --- Configuration Limits & Offsets ---
  int8_t offset = myServo.getAngleOffset(id);
  if (offset != 127) {
    Serial.println("Angle Offset: " + String(offset));
  } else {
    Serial.println("Angle Offset: TIMEOUT/ERROR");
  }

  int16_t min_angle, max_angle;
  if (myServo.getAngleLimits(id, min_angle, max_angle)) {
    Serial.println("Angle Limits: " + String(min_angle) + " to " + String(max_angle));
  } else {
    Serial.println("Angle Limits: TIMEOUT/ERROR");
  }

  int16_t min_vin, max_vin;
  if (myServo.getVoltageLimits(id, min_vin, max_vin)) {
    Serial.println("Voltage Limits: " + String(min_vin) + " to " + String(max_vin));
  } else {
    Serial.println("Voltage Limits: TIMEOUT/ERROR");
  }

  uint8_t max_temp = myServo.getMaxTemperatureLimit(id);
  if (max_temp != 0) {
    Serial.println("Max Temp Limit: " + String(max_temp) + " C");
  } else {
    Serial.println("Max Temp Limit: TIMEOUT/ERROR");
  }

  // --- LED Status ---
  uint8_t led_errors = myServo.getLedErrors(id);
  if (led_errors != 255) {
    Serial.println("LED Error Code: " + String(led_errors));
  } else {
    Serial.println("LED Error Code: TIMEOUT/ERROR");
  }
}