# HiBusServo Library

A modern Arduino library for controlling Hiwonder serial bus servos.

## Features

- **Position Control**: Move servos to specific angles with precise timing
- **Continuous Rotation**: Use servos in motor mode with speed control
- **Status Monitoring**: Read servo position, voltage, and temperature
- **Parameter Configuration**: Set angle limits, voltage limits, temperature limits
- **LED Control**: Control servo status and error LEDs
- **Robust Communication**: Built-in retry mechanism for reliable serial communication

## Installation

**Note**: This library is not yet available through the Arduino Library Manager. Please use one of the manual installation methods below.

### Option 1: Manual ZIP Installation (Recommended)
1. Download this library as a ZIP file
2. Open Arduino IDE
3. Go to **Sketch** → **Include Library** → **Add .ZIP Library**
4. Select the downloaded ZIP file

### Option 2: Development Installation
1. Clone or download this repository
2. Copy the entire folder to your Arduino libraries directory. Default library locations include:
   - **Windows**: `Documents/Arduino/libraries/`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`

## Hardware Setup

### Connections
- Connect the BusLinker to your Arduino's hardware serial port (e.g., Serial2 TX and RX pins)
- Connect servo bus cables to the BusLinker's servo ports
- Provide appropriate power supply to the BusLinker/servos (typically 6V to 8.4V)
- Ensure common ground between Arduino and BusLinker power supply

### Servo Configuration
- Set each servo to a unique ID (1-253) using Hiwonder's configuration software
- Note the servo ID for use in your code

## Quick Start

```cpp
#include <HiBusServo.h>

// Create servo controller instance
HiBusServo servo;

void setup() {
    Serial.begin(115200);
    
    // Initialize servo communication on Serial2
    servo.begin(Serial2);
    
    // Move servo ID 1 to 0 degrees over 1 second
    servo.moveTo(1, 0.0, 1000);
}

void loop() {
    // Read current position
    int position = servo.readPosition(1);
    if (position >= 0) {
        float degrees = (position - 500) * (120.0 / 500.0);
        Serial.print("Position: ");
        Serial.print(degrees, 1);
        Serial.println("°");
    }
    
    delay(100);
}
```

## API Reference

### Constructor
```cpp
HiBusServo()
```
Creates a servo controller instance.

### Setup
```cpp
void begin(HardwareSerial &serial)
```
Initialize communication with the specified hardware serial port.

### Basic Movement
```cpp
void moveTo(int servoId, float destination, uint16_t time = 200)
void spinAt(int servoId, float velocity)
void stopMove(int servoId)
```
- `moveTo`: Move to angle in degrees (-120° to +120°, 0° = center)
- `spinAt`: Continuous rotation (-100 to +100, negative = clockwise)
- `stopMove`: Stop any current movement

### Status Reading
```cpp
int readPosition(int servoId)
int readVin(int servoId)
int readTemperature(int servoId)
```
- `readPosition`: Current position in servo units (0-1000)
- `readVin`: Supply voltage in millivolts
- `readTemperature`: Temperature in Celsius

### Configuration
```cpp
void setServoID(int oldID, int newID)
void setAngleLimits(int servoId, int16_t minAngle, int16_t maxAngle)
void setVinLimits(int servoId, uint16_t minVin, uint16_t maxVin)
void setTempMaxLimit(int servoId, uint8_t maxTemp)
void setAngleOffset(int servoId, int8_t offset)
```
- `setServoID`: Change servo ID
- `setAngleLimits`: Set angle limits in servo units (0-1000)
- `setVinLimits`: Set voltage limits in millivolts
- `setTempMaxLimit`: Set maximum temperature limit in Celsius
- `setAngleOffset`: Set angle offset calibration (-125 to +125)


### Control
```cpp
void loadServo(int servoId)
void unloadServo(int servoId)
void setLEDControl(int servoId, bool ledOn)
void setLEDErrorControl(int servoId, bool errorLedOn)
```
- `loadServo`: Enable servo motor (holds position, responds to commands)
- `unloadServo`: Disable servo motor (free movement, power saving)
- `setLEDControl`: Control servo status LED on/off
- `setLEDErrorControl`: Enable/disable error LED indication

### Parameter Reading
```cpp
bool getServoMode(int servoId, uint8_t &mode, int16_t &speed)
bool getAngleLimits(int servoId, int16_t &minAngle, int16_t &maxAngle)
bool getVinLimits(int servoId, uint16_t &minVin, uint16_t &maxVin)
bool getTempMaxLimit(int servoId, uint8_t &maxTemp)
bool getAngleOffset(int servoId, int8_t &offset)
bool getLEDControl(int servoId, bool &ledOn)
bool getLEDErrorControl(int servoId, bool &errorLedOn)
```
- `getServoMode`: Read current mode (0=servo, 1=motor) and speed
- `getAngleLimits`: Read min/max angle limits in servo units
- `getVinLimits`: Read voltage limits in millivolts
- `getTempMaxLimit`: Read maximum temperature limit in Celsius
- `getAngleOffset`: Read angle calibration offset (-125 to +125)
- `getLEDControl`: Read status LED setting
- `getLEDErrorControl`: Read error LED setting.

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

## Troubleshooting

### Common Issues

**Servo not responding:**
- Check wiring connections
- Verify servo ID is correct
- Ensure adequate power supply (min 4.5V to max 14V)
- Check baud rate (should be 115200)
- Servo may be disconnected
- Wrong servo ID
- Communication interference
- Microcontroller logic level not matched to servo logic level, step up board required

### Debug Mode
Enable debug mode in the constructor to see communication details:
```cpp
HiBusServo servo(true);  // Enable debug output
```

## Protocol Details

This library implements the Hiwonder intelligent servo communication protocol:
- **Baud Rate**: 115200
- **Frame Format**: Header(2) + ID(1) + Length(1) + Command(1) + Data(0-N) + Checksum(1)
- **Header**: 0x55 0x55
- **ID Range**: 0-253 (0x00-0xFD), with 254 (0xFE) as broadcast ID
- **Length**: Equal to the data length (including the Length byte itself)
- **Command Types**:
  - Write commands (typically with parameters)
  - Read commands (typically without parameters, returns data)
- **Checksum**: `Checksum = ~(ID + Length + Cmd + Prm1 + ... + PrmN)` where ~ is negation

> This protocol is based on the official Hiwonder documentation.

## Credits
- [Hiwonder](https://www.hiwonder.com) (formerly known as Lewansoul, and even older as Lobot) for the original code and documentation.
- [madhephaestus](https://github.com/madhephaestus) for their library, pretty sure I designed my OG cpp and h file based on it.
- [Windsurf](https://windsurf.com) (formerly known as Codeium) for their AI code completion and refactoring tools used to make my modifications into a library.
