#ifndef HIBUS_SERVO_H
#define HIBUS_SERVO_H

#include "Arduino.h"

class HiBusServo
{
public:
    HiBusServo(bool debug = false, int maxRetries = 3);
    
    void begin(HardwareSerial &serial);
    void setMaxRetries(int retries);
    
    // Basic movement commands
    void moveTo(int servoId, float destination, uint16_t time = 200);
    void spinAt(int servoId, float velocity);
    void stopMove(int servoId);
    
    // Position and status reading
    int readPosition(int servoId);
    int readVin(int servoId);
    int readTemperature(int servoId);
    
    // Servo configuration
    void setServoID(int oldID, int newID);
    void setServoMode(int servoId, uint8_t mode, int16_t speed);
    void setAngleLimits(int servoId, int16_t minAngle, int16_t maxAngle);
    void setVinLimits(int servoId, uint16_t minVin, uint16_t maxVin);
    void setTempMaxLimit(int servoId, uint8_t maxTemp);
    void setAngleOffset(int servoId, int8_t offset);
    
    // Servo control
    void loadServo(int servoId);
    void unloadServo(int servoId);
    void setLEDControl(int servoId, bool ledOn);
    void setLEDErrorControl(int servoId, bool errorLedOn);
    
    // Getters for servo parameters
    bool getServoMode(int servoId, uint8_t &mode, int16_t &speed);
    bool getAngleLimits(int servoId, int16_t &minAngle, int16_t &maxAngle);
    bool getVinLimits(int servoId, uint16_t &minVin, uint16_t &maxVin);
    bool getTempMaxLimit(int servoId, uint8_t &maxTemp);
    bool getAngleOffset(int servoId, int8_t &offset);
    bool getLEDControl(int servoId, bool &ledOn);
    bool getLEDErrorControl(int servoId, bool &errorLedOn);

private:
    bool _debug;
    int _maxRetries;
    HardwareSerial *_serialPort;
    
    // Protocol constants
    static const uint8_t FRAME_HEADER = 0x55;
    static const uint8_t CMD_MOVE_TIME_WRITE = 1;
    static const uint8_t CMD_MOVE_TIME_READ = 2;
    static const uint8_t CMD_MOVE_TIME_WAIT_WRITE = 7;
    static const uint8_t CMD_MOVE_TIME_WAIT_READ = 8;
    static const uint8_t CMD_MOVE_START = 11;
    static const uint8_t CMD_MOVE_STOP = 12;
    static const uint8_t CMD_ID_WRITE = 13;
    static const uint8_t CMD_ID_READ = 14;
    static const uint8_t CMD_ANGLE_OFFSET_ADJUST = 17;
    static const uint8_t CMD_ANGLE_OFFSET_WRITE = 18;
    static const uint8_t CMD_ANGLE_OFFSET_READ = 19;
    static const uint8_t CMD_ANGLE_LIMIT_WRITE = 20;
    static const uint8_t CMD_ANGLE_LIMIT_READ = 21;
    static const uint8_t CMD_VIN_LIMIT_WRITE = 22;
    static const uint8_t CMD_VIN_LIMIT_READ = 23;
    static const uint8_t CMD_TEMP_MAX_LIMIT_WRITE = 24;
    static const uint8_t CMD_TEMP_MAX_LIMIT_READ = 25;
    static const uint8_t CMD_TEMP_READ = 26;
    static const uint8_t CMD_VIN_READ = 27;
    static const uint8_t CMD_POS_READ = 28;
    static const uint8_t CMD_OR_MOTOR_MODE_WRITE = 29;
    static const uint8_t CMD_OR_MOTOR_MODE_READ = 30;
    static const uint8_t CMD_LOAD_OR_UNLOAD_WRITE = 31;
    static const uint8_t CMD_LOAD_OR_UNLOAD_READ = 32;
    static const uint8_t CMD_LED_CTRL_WRITE = 33;
    static const uint8_t CMD_LED_CTRL_READ = 34;
    static const uint8_t CMD_LED_ERROR_WRITE = 35;
    static const uint8_t CMD_LED_ERROR_READ = 36;
    
    // Utility macros as inline functions
    inline uint8_t _get_low_byte(uint16_t value) { return (uint8_t)(value); }
    inline uint8_t _get_high_byte(uint16_t value) { return (uint8_t)(value >> 8); }
    inline uint16_t _bytes_to_word(uint8_t high, uint8_t low) { return (((uint16_t)(high)) << 8) | (uint8_t)(low); }
    
    // Private helper methods
    uint8_t _calculate_checksum(uint8_t buf[]);
    void _serial_servo_move(uint8_t id, int16_t position, uint16_t time);
    void _serial_servo_stop_move(uint8_t id);
    void _serial_servo_set_id(uint8_t oldID, uint8_t newID);
    void _serial_servo_set_mode(uint8_t id, uint8_t mode, int16_t speed);
    void _serial_servo_load(uint8_t id);
    void _serial_servo_unload(uint8_t id);
    int _serial_servo_receive_handle(uint8_t *ret);
    int _serial_servo_read_position(uint8_t id);
    int _serial_servo_read_vin(uint8_t id);
    
    // Generic read/write methods for new functionality
    void _write_servo_param(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t dataLen);
    bool _read_servo_param(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t expectedLen);
};

#endif // HIBUS_SERVO_H
