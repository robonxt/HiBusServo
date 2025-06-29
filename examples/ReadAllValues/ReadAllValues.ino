/*
 * ReadAllValues.ino - Example for HiBusServo library
 * 
 * This example reads all possible values from a single servo (ID 1) and prints them
 * to the Serial monitor. It's useful for verifying communication and checking that
 * all values are being read correctly with the proper data types.
 * 
 * Created: 2024-06-28
 * By: HiBusServo Library
 */

#include "test.h"

// Enable debug output for the servo library
// Comment this line to disable debug output
#define DEBUG_SERVO

// Create a servo controller instance with default settings
HiBusServo servo;

// Servo ID to read from (change if needed)
const uint8_t SERVO_ID = 1;

// Variables to store read values
uint8_t mode, maxTemp;
int16_t speed, minAngle, maxAngle, position, vin, minVin, maxVin;
int8_t temp, angleOffset;
bool ledOn, errorLedOn;

// Error counter
uint8_t readErrors = 0;
const uint8_t MAX_RETRIES = 3;

void setup() {
  // Start serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect
  }

// Wait for serial monitor to be ready
#ifdef DEBUG_SERVO
  while (!Serial) {
    ;  // wait for serial port to connect
  }
  delay(1000);
#endif

  Serial.println(F("HiBusServo - Read All Values Example"));
  Serial.println(F("==================================="));

  // Initialize communication with the servos on Serial2
  // Note: The begin() method will initialize the serial port
  servo.begin(Serial2);

  // Set a longer timeout for reading
  Serial2.setTimeout(100);

  // Set max retries for more reliable communication
  servo.setMaxRetries(MAX_RETRIES);

  Serial.print(F("Reading values from servo ID: "));
  Serial.println(SERVO_ID);
  Serial.println();
}

void loop() {
  // Clear the screen for better readability
  Serial.println("\n\n\n\n\n\n\n\n\n\n");  // Add some newlines to clear the screen
  Serial.println(F("=== Reading Servo Values ==="));

  // Reset error counter
  readErrors = 0;

  // Read position (0-1000, where 500 is center)
  position = servo.readPosition(SERVO_ID);
  if (position == -1) {
    Serial.println(F("Error reading position!"));
    readErrors++;
  } else {
    Serial.print(F("Position: "));
    Serial.print(position);
    Serial.print(F(" ("));
    Serial.print((position - 500) * 0.24f, 1);  // Convert to degrees (-120 to 120)
    Serial.println(F("°)"));
  }

  // Read input voltage (mV)
  // Note: readVin returns -2048 on error
  vin = servo.readVin(SERVO_ID);
  if (vin == -2048) {
    Serial.println(F("Error reading input voltage!"));
    readErrors++;
  } else {
    Serial.print(F("Input Voltage: "));
    Serial.print(vin);
    Serial.println(F(" mV"));
  }

  // Read temperature (°C)
  temp = servo.readTemperature(SERVO_ID);
  if (temp == -1) {
    Serial.println(F("Error reading temperature!"));
    readErrors++;
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(temp);
    Serial.println(F("°C"));
  }

  // Read servo mode and speed
  if (!servo.getServoMode(SERVO_ID, mode, speed)) {
    Serial.println(F("Error reading servo mode!"));
    readErrors++;
  } else {
    Serial.print(F("Mode: "));
    Serial.print(mode == 0 ? "Position" : "Speed");
    Serial.print(F(" | Speed: "));
    Serial.println(speed);
  }

  // Read angle limits
  if (!servo.getAngleLimits(SERVO_ID, minAngle, maxAngle)) {
    Serial.println(F("Error reading angle limits!"));
    readErrors++;
  } else {
    Serial.print(F("Angle Limits: "));
    Serial.print(minAngle);
    Serial.print(F(" to "));
    Serial.print(maxAngle);
    Serial.print(F(" ("));
    Serial.print((minAngle - 500) * 0.24f, 1);
    Serial.print(F("° to "));
    Serial.print((maxAngle - 500) * 0.24f, 1);
    Serial.println(F("°)"));
  }

  // Read voltage limits
  // Note: getVinLimits returns true on success, false on failure
  // The values are passed by reference and will be updated if successful
  uint16_t minVoltage, maxVoltage;
  if (!servo.getVinLimits(SERVO_ID, minVoltage, maxVoltage)) {
    Serial.println(F("Error reading voltage limits!"));
    readErrors++;
  } else {
    minVin = minVoltage;
    maxVin = maxVoltage;
    Serial.print(F("Voltage Limits: "));
    Serial.print(minVoltage);
    Serial.print(F(" to "));
    Serial.print(maxVoltage);
    Serial.println(F(" mV"));
  }

  // Read max temperature limit
  if (!servo.getTempMaxLimit(SERVO_ID, maxTemp)) {
    Serial.println(F("Error reading max temperature limit!"));
    readErrors++;
  } else {
    Serial.print(F("Max Temperature: "));
    Serial.print(maxTemp);
    Serial.println(F("°C"));
  }

  // Read angle offset
  if (!servo.getAngleOffset(SERVO_ID, angleOffset)) {
    Serial.println(F("Error reading angle offset!"));
    readErrors++;
  } else {
    Serial.print(F("Angle Offset: "));
    Serial.print(angleOffset);
    Serial.print(F(" ("));
    Serial.print(angleOffset * 0.24f, 1);  // Convert to degrees
    Serial.println(F("°)"));
  }

  // Read LED control
  if (!servo.getLEDControl(SERVO_ID, ledOn)) {
    Serial.println(F("Error reading LED control!"));
    readErrors++;
  } else {
    Serial.print(F("LED: "));
    Serial.println(ledOn ? "ON" : "OFF");
  }

  // Read error LED control
  if (!servo.getLEDErrorControl(SERVO_ID, errorLedOn)) {
    Serial.println(F("Error reading error LED control!"));
    readErrors++;
  } else {
    Serial.print(F("Error LED: "));
    Serial.println(errorLedOn ? "ENABLED" : "DISABLED");
  }

  // Print summary
  Serial.println(F("\n=== Summary ==="));
  if (readErrors > 0) {
    Serial.print(readErrors);
    Serial.println(F(" errors occurred while reading values."));
  } else {
    Serial.println(F("All values read successfully!"));
  }

  // Wait before next read
  Serial.println(F("\nNext update in 2 seconds..."));
  delay(2000);
}
