# HiBusServo Library

A modern Arduino library for controlling Hiwonder serial bus servos.

## Features

- **Position Control**: Move servos to specific angles (-120° to 120°) with precise timing.
- **Continuous Rotation**: Use servos in motor mode with speed control (-100 to 100).
- **Status Monitoring**: Read servo position, voltage, and temperature.
- **Parameter Configuration**: Set angle limits, voltage limits, temperature limits, and angle offset.
- **Torque Control**: Enable or disable servo torque (load/unload).
- **LED Control**: Control servo status LED and error LED indicators.
- **ID Management**: Change servo IDs for bus configuration.
- **Configuration Reading**: Read back all servo configuration parameters.
- **Robust Communication**: Built-in retry mechanism for reliable serial communication.

## Installation

**Note**: This library is not yet available through the Arduino Library Manager. Please use one of the manual installation methods below.

### Option 1: Manual ZIP Installation (Recommended)
1. Download this library as a ZIP file.
2. Open the Arduino IDE.
3. Go to **Sketch** → **Include Library** → **Add .ZIP Library**.
4. Select the downloaded ZIP file.

### Option 2: Development Installation
1. Clone or download this repository.
2. Copy the entire folder to your Arduino libraries directory. Default locations include:
   - **Windows**: `Documents\Arduino\libraries\`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`

## Hardware Setup

### Connections
- Connect the BusLinker to your Arduino's hardware serial port (e.g., `Serial2` TX and RX pins).
- Connect servo bus cables to the BusLinker's servo ports.
- Provide an appropriate power supply to the BusLinker/servos (typically 6V to 8.4V).
- Ensure a common ground between the Arduino and BusLinker power supply.

### Servo Configuration
- Set each servo to a unique ID (1-253) using Hiwonder's configuration software.
- Note the servo ID for use in your code.

## Quick Start

```cpp
#include <HiBusServo.h>

// Create a servo controller instance with default retry count (3).
// You can specify a custom retry count: HiBusServo servo(5);
HiBusServo servo;

void setup() {
  // Initialize communication with the servos on Serial2.
  // The library uses a fixed baud rate of 115200.
  servo.begin(Serial2);
}

void loop() {
  // Move servo 1 to the -90 degree position over half a second.
  servo.moveTo(1, -90.0, 500);
  delay(1000); // Wait 1 second.

  // Move servo 1 to the 90 degree position over half a second.
  servo.moveTo(1, 90.0, 500);
  delay(1000); // Wait 1 second.
}
```

## API Reference

### Initialization

```cpp
HiBusServo(int maxRetries = 3);
```
Constructor for the HiBusServo object. Optionally specify the maximum number of retries for read operations.

```cpp
void begin(HardwareSerial &serial);
```
Initialize the serial port for communication with the servos. The baud rate is fixed at 115200.

```cpp
void setMaxRetries(int retries);
```
Set the maximum number of retries for read operations.

### Movement Control

```cpp
void moveTo(int servoId, float angle, int time);
```
Move the servo to a specific angle over a specified time. Angle range is -120° to 120°. Time is in milliseconds.

```cpp
void moveToRaw(int servoId, int position, int time);
```
Move the servo to a raw position (0-1000) over a specified time. Time is in milliseconds.

```cpp
void setVelocity(int servoId, int velocity);
```
Set the velocity for continuous rotation mode. Range is -100 to 100, where 0 stops the servo.

### Status Reading

```cpp
bool readPosition(int servoId, float &angle);
```
Read the current angle of the servo in degrees. Returns true if successful.

```cpp
bool readPositionRaw(int servoId, int &position);
```
Read the current raw position (0-1000) of the servo. Returns true if successful.

```cpp
bool readVoltage(int servoId, float &voltage);
```
Read the current voltage of the servo in volts. Returns true if successful.

```cpp
bool readTemperature(int servoId, int &temperature);
```
Read the current temperature of the servo in degrees Celsius. Returns true if successful.

```cpp
bool isConnected(int servoId);
```
Check if a servo with the specified ID is connected. Returns true if the servo is connected.

### Servo Configuration

```cpp
bool setServoMode(int servoId, uint8_t mode, int16_t speed = 0);
```
Set the servo operating mode. Mode 0 is position control, mode 1 is continuous rotation. Speed parameter is used for continuous rotation mode. Returns true if successful.

```cpp
bool setAngleLimits(int servoId, int16_t minAngle, int16_t maxAngle);
```
Set the minimum and maximum angle limits in servo units (0-1000). Returns true if successful.

```cpp
bool setVinLimits(int servoId, uint16_t minVin, uint16_t maxVin);
```
Set the minimum and maximum voltage limits in millivolts. Returns true if successful.

```cpp
bool setTempMaxLimit(int servoId, uint8_t maxTemp);
```
Set the maximum temperature limit in degrees Celsius. Returns true if successful.

```cpp
bool setAngleOffset(int servoId, int8_t offset);
```
Set the angle offset calibration value (-125 to +125). Returns true if successful.

```cpp
bool setID(int servoId, int newId);
```
Change the ID of a servo. Returns true if successful.

### Servo Control

```cpp
void loadServo(int servoId);
```
Enable the servo motor. When loaded, the servo holds its position and responds to movement commands.

```cpp
void unloadServo(int servoId);
```
Disable the servo motor. When unloaded, the servo can be moved freely by hand and consumes less power.

```cpp
bool setLEDControl(int servoId, bool ledOn);
```
Control the servo status LED. Set to true to turn the LED on, false to turn it off. Returns true if successful.

```cpp
bool setLEDErrorControl(int servoId, bool errorLedOn);
```
Control the servo error LED indication. Set to true to enable error LED indication, false to disable. Returns true if successful.

### Configuration Reading

```cpp
bool getServoMode(int servoId, uint8_t &mode, int16_t &speed);
```
Read the current servo operating mode and speed. Mode 0 is position control, mode 1 is continuous rotation. Returns true if successful.

```cpp
bool getAngleLimits(int servoId, int16_t &minAngle, int16_t &maxAngle);
```
Read the minimum and maximum angle limits in servo units (0-1000). Returns true if successful.

```cpp
bool getVinLimits(int servoId, uint16_t &minVin, uint16_t &maxVin);
```
Read the minimum and maximum voltage limits in millivolts. Returns true if successful.

```cpp
bool getTempMaxLimit(int servoId, uint8_t &maxTemp);
```
Read the maximum temperature limit in degrees Celsius. Returns true if successful.

```cpp
bool getAngleOffset(int servoId, int8_t &offset);
```
Read the angle offset calibration value (-125 to +125). Returns true if successful.

```cpp
bool getLEDControl(int servoId, bool &ledOn);
```
Read the current status LED setting. Returns true if successful.

```cpp
bool getLEDErrorControl(int servoId, bool &errorLedOn);
```
Read the current error LED indication setting. Returns true if successful.

## Examples

The library includes several examples to help you get started:

### OneServoSweep
A simple example that demonstrates basic servo control with a single servo. The servo sweeps left, right, and center in a continuous cycle while displaying position, voltage, and temperature status.

**Features:**
- Basic position control
- Status monitoring (position, voltage, temperature)
- Beginner-friendly with clear comments
- Easy to modify timing and positions

### MultipleServos
An advanced example that automatically detects all connected servos and controls them simultaneously. Perfect for projects with multiple servos that need coordinated movement.

**Features:**
- Automatic servo scanning and detection
- Coordinated multi-servo control
- Status monitoring for all connected servos
- Robust error handling

**Usage:** Connect multiple servos to the same data bus with unique IDs, and the example will find and control them all together.

## Protocol Details

This library implements the Hiwonder intelligent servo communication protocol:
- **Baud Rate**: 115200
- **Frame Format**: Header(2) + ID(1) + Length(1) + Command(1) + Data(0-N) + Checksum(1)
- **Header**: 0x55 0x55
- **ID Range**: 0-253 (0x00-0xFD), with 254 (0xFE) as broadcast ID
- **Length**: The total number of bytes in the packet from the `ID` field to the last parameter byte. Formula: `Length = 3 + number of parameter bytes`.
- **Command Types**:
  - Write commands (typically with parameters)
  - Read commands (typically without parameters, returns data)
- **Checksum**: The bitwise NOT of the sum of all bytes from `ID` to the last parameter byte.

> This protocol is based on the official Hiwonder documentation.

## Credits
- [Hiwonder](https://www.hiwonder.com) (formerly known as Lewansoul, and even older as Lobot) for the original code and documentation.
- [madhephaestus](https://github.com/madhephaestus) for their library, pretty sure I designed my OG cpp and h file based on it.
- [Windsurf](https://windsurf.com) (formerly known as Codeium) for their AI code completion and refactoring tools used to make my modifications into a library.
