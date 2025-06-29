# HiBusServo Library

A modern, type-safe Arduino library for controlling Hiwonder serial bus servos with comprehensive error handling and precise control.


### Connections
- Connect the BusLinker to your Arduino's hardware serial port (e.g., `Serial2` TX and RX pins).
- Connect servo bus cables to the BusLinker's servo ports.
- Provide an appropriate power supply to the BusLinker/servos (typically 6V to 8.4V).
- Ensure a common ground between the Arduino and BusLinker power supply.

### Servo Configuration
- Set each servo to a unique ID (1-253) using Hiwonder's configuration software.
- Note the servo ID for use in your code.


## Credits
- [Hiwonder](https://www.hiwonder.com) (formerly known as Lewansoul, and even older as Lobot) for the original code and documentation.
- [madhephaestus](https://github.com/madhephaestus) for the OG Arduino library (used as reference).
- [ethanlipson](https://github.com/ethanlipson) and [maximkulkin](https://github.com/maximkulkin) for Python libraries (used as references)
- [Windsurf](https://windsurf.com) (formerly known as Codeium) for their AI code completion and refactoring tools used to make my modifications into a library.
