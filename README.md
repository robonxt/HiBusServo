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

// Create a servo controller instance with default maxRetries (3)
HiBusServo servo; // or HiBusServo servo(5); // for custom retry count

void setup() {
  Serial.begin(9600);  // For debugging output
  
  // Initialize communication with the servos on Serial2
  // The library uses a fixed baud rate of 115200
  servo.begin(Serial2);
  
  // Optionally set max retries
  servo.setMaxRetries(5);
}

void loop() {
  // Move servo ID 1 to 90 degrees over 500ms
  servo.moveTo(1, 90, 500);
  delay(600);  // Wait for movement to complete
  
  // Read and print servo status
  int pos = servo.readPosition(1);
  int vin = servo.readVin(1);
  int temp = servo.readTemperature(1);
  
  Serial.print("Position: ");
  Serial.print(pos);
  Serial.print(" | Voltage: ");
  Serial.print(vin);
  Serial.print("mV | Temp: ");
  Serial.print(temp);
  Serial.println("°C");
  
  // Spin servo in wheel mode
  servo.spinAt(1, 50);          // Spin at speed 50 (automatically sets wheel mode)
  delay(2000);                  // Spin for 2 seconds
  servo.stopMove(1);            // Stop the servo
  delay(1000);                  // Stop before looping again.
}
```

## API Reference

### Constructor & Setup

```cpp
HiBusServo(int maxRetries = 3);
```
Create a servo controller. `maxRetries` sets the retry count for communication (default 3).

```cpp
void begin(HardwareSerial &serial);
```
Initialize communication with the servos (e.g., `Serial2`).

```cpp
void setMaxRetries(int retries);
```
Change the retry count after construction.

### Basic Movement

```cpp
void moveTo(int servoId, float destination, uint16_t time = 200);
```
Move servo to angle (`-120` to `120` degrees) in `time` ms.

```cpp
void spinAt(int servoId, float velocity);
```
Spin servo in wheel mode (`-100` to `100` speed).

```cpp
void stopMove(int servoId);
```
Stop servo movement.

### Status Reading

```cpp
int readPosition(int servoId);
```
Get current position in servo units (0-1000) or -1 on error. Note: This returns raw servo units, not degrees.

```cpp
int readVin(int servoId);
```
Get input voltage (mV) or -2048 on error.

```cpp
int readTemperature(int servoId);
```
Get temperature (°C) or -1 on error.

### Servo Configuration

```cpp
void setServoID(int oldID, int newID);
```
Change servo ID.

```cpp
void setServoMode(int servoId, uint8_t mode, int16_t speed);
```
Set mode: `0` (position), `1` (wheel).

```cpp
void setAngleLimits(int servoId, int16_t minAngle, int16_t maxAngle);
```
Set angle limits in servo units (0-1000). Note: These are raw servo units, not degrees.

```cpp
void setVinLimits(int servoId, uint16_t minVin, uint16_t maxVin);
```
Set voltage limits (mV).

```cpp
void setTempMaxLimit(int servoId, uint8_t maxTemp);
```
Set max temperature (°C).

```cpp
void setAngleOffset(int servoId, int8_t offset);
```
Set angle offset.

### Servo Control

```cpp
void loadServo(int servoId);
```
Enable torque. When loaded, the servo holds its position and responds to movement commands.

```cpp
void unloadServo(int servoId);
```
Disable torque. When unloaded, the servo can be moved freely by hand and consumes less power.

```cpp
void setLEDControl(int servoId, bool ledOn);
```
Control the servo status LED. Set to true to turn the LED on, false to turn it off.

```cpp
void setLEDErrorControl(int servoId, bool errorLedOn);
```
Control the servo error LED indication. Set to true to enable error LED indication, false to disable.

### Parameter Getters

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
