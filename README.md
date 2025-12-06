# HiBusServo Library

A modern library for controlling Hiwonder serial bus servos. Supports position control, continuous rotation, parameter configuration, and status monitoring including temperature and voltage readings.

## Breaking Changes / Upgrade Notes

- **Serial initialization is now the user's responsibility.**
  - `HiBusServo::begin(long speed)` no longer calls `Serial.begin()` / `HardwareSerial::begin()`.
  - You **must** configure the underlying serial port yourself in `setup()`, for example:
    ```cpp 
    Serial2.begin(115200);
    HiBusServo myServo(Serial2);
    // myServo.begin(115200); // now effectively a no-op, safe to remove eventually
    ```
  - Existing sketches that relied on `HiBusServo::begin()` to initialize the serial port need to be updated accordingly.

## Requirements
- Microcontroller with multiple hardware serial ports (e.g., Arduino Mega, ESP32, or Raspberry Pi Pico)
- [Hiwonder BusLinker (also known as Hiwonder TTL / USB Debugging Board)](https://www.hiwonder.com/collections/servo-controller/products/hiwonder-ttl-usb-debugging-board)
- [Hiwonder Serial Bus Servos (e.g., LX-16A)](https://www.hiwonder.com/collections/bus-servo/products/lx-16a)

## Connections
- Connect the BusLinker to your microcontroller's hardware serial port (e.g., `Serial2` TX and RX pins).
- Connect servos to the BusLinker's servo ports.
- Provide an appropriate power supply to the BusLinker/servos (typically 6V to 8.4V).
- Ensure a common ground between the microcontroller and BusLinker power supply.

## Credits
Created by [robonxt](https://github.com/robonxt)

Thanks to (In no particular order):
- [Hiwonder](https://www.hiwonder.com) (formerly known as Lewansoul, and even older as Lobot) for the original code and documentation.
- [madhephaestus](https://github.com/madhephaestus) for the OG lx16a Arduino library (used as reference).
- [ethanlipson](https://github.com/ethanlipson) and [maximkulkin](https://github.com/maximkulkin) for Python libraries (used as references)
- [Windsurf](https://windsurf.com) (formerly known as Codeium) for their AI code completion and refactoring tools.
- [Google](https://www.google.com) for [Google AI Studio](https://aistudio.google.com), used for cross-checking with official and third-party libraries and documentation.
