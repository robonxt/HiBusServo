#include <HiBusServo.h>

HiBusServo myServo(Serial2);
const uint8_t SERVO_ID = 1;

void setup() {
  Serial.begin(115200);
  myServo.begin(115200);
  delay(2000);
  Serial.println("=== HiBusServo Comprehensive Test ===");
  Serial.println("Testing all read functions and error handling");
}

void loop() {
  Serial.println("\n=== Reading All Values from Servo ID: " + String(SERVO_ID) + " ===");
  printAllServoData(SERVO_ID);

  Serial.println("\n=== Testing Error Constants ===");
  testErrorConstants();

  Serial.println("\n---------------------------------");
  delay(5000);  // Wait 5 seconds before repeating the check
}

void printAllServoData(uint8_t id) {
  Serial.println("--- Position & Movement ---");
  int16_t pos_raw = myServo.getPosition(id);
  if (pos_raw != SERVO_ERROR_TIMEOUT) {
    Serial.println("Position (raw): " + String(pos_raw));
  } else {
    Serial.println("Position (raw): TIMEOUT/ERROR");
  }

  float pos_deg = myServo.getPositionInDegrees(id);
  if (pos_deg != SERVO_POSITION_INVALID) {
    Serial.println("Position (deg): " + String(pos_deg));
  } else {
    Serial.println("Position (deg): TIMEOUT/ERROR");
  }

  Serial.println("--- Physical State ---");
  int16_t vin = myServo.getVoltage(id);
  if (vin != SERVO_ERROR_TIMEOUT) {
    Serial.println("Voltage: " + String(vin) + " mV");
  } else {
    Serial.println("Voltage: TIMEOUT/ERROR");
  }

  int8_t temp = myServo.getTemperature(id);
  if (temp != SERVO_ERROR_TIMEOUT) {
    Serial.println("Temperature: " + String(temp) + " C");
  } else {
    Serial.println("Temperature: TIMEOUT/ERROR");
  }

  Serial.println("--- Mode & Motor State ---");
  uint8_t mode;
  int16_t speed;
  if (myServo.getMode(id, mode, speed)) {
    Serial.print("Mode: " + String(mode));
    if (mode == 0) Serial.println(" (Servo Mode)");
    if (mode == 1) Serial.println(" (Motor Mode), Speed: " + String(speed));
  } else {
    Serial.println("Mode: TIMEOUT/ERROR");
  }

  bool motor_on = myServo.isMotorOn(id);
  Serial.println("Motor Torque: " + String(motor_on ? "ON" : "OFF"));

  Serial.println("--- Configuration Limits & Offsets ---");
  int8_t offset = myServo.getAngleOffset(id);
  if (offset != SERVO_OFFSET_INVALID) {
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
    Serial.println("Voltage Limits: " + String(min_vin) + " to " + String(max_vin) + " mV");
  } else {
    Serial.println("Voltage Limits: TIMEOUT/ERROR");
  }

  uint8_t max_temp = myServo.getMaxTemperatureLimit(id);
  if (max_temp != 0) {
    Serial.println("Max Temp Limit: " + String(max_temp) + " C");
  } else {
    Serial.println("Max Temp Limit: TIMEOUT/ERROR");
  }

  Serial.println("--- LED Status ---");
  uint8_t led_errors = myServo.getLedErrors(id);
  if (led_errors != SERVO_ID_INVALID) {
    Serial.println("LED Error Code: " + String(led_errors) + " (0=No alarm, 7=All alarms)");
  } else {
    Serial.println("LED Error Code: TIMEOUT/ERROR");
  }

  Serial.println("--- ID Information ---");
  uint8_t current_id = myServo.getID(id);
  if (current_id != SERVO_ID_INVALID) {
    Serial.println("Current ID: " + String(current_id));
  } else {
    Serial.println("Current ID: TIMEOUT/ERROR");
  }
}

void testErrorConstants() {
  Serial.println("Testing error constant values:");
  Serial.println("SERVO_ERROR_TIMEOUT = " + String(SERVO_ERROR_TIMEOUT));
  Serial.println("SERVO_POSITION_INVALID = " + String(SERVO_POSITION_INVALID));
  Serial.println("SERVO_OFFSET_INVALID = " + String(SERVO_OFFSET_INVALID));
  Serial.println("SERVO_ID_INVALID = " + String(SERVO_ID_INVALID));
  
  // Test invalid servo ID (should fail gracefully)
  Serial.println("\nTesting invalid servo ID (254):");
  int16_t invalid_pos = myServo.getPosition(254);
  Serial.println("Invalid ID position result: " + String(invalid_pos));
}
