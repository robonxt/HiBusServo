#ifndef test_h
#define test_h
#include "Arduino.h"
const uint8_t SERVO_MOVE_TIME_WRITE = 1;
const uint8_t SERVO_MOVE_TIME_READ = 2;
const uint8_t SERVO_MOVE_STOP = 11;
const uint8_t SERVO_ID_WRITE = 13;
const uint8_t SERVO_ID_READ = 14;
const uint8_t SERVO_ANGLE_OFFSET_ADJUST = 17;
const uint8_t SERVO_ANGLE_OFFSET_WRITE = 18;
const uint8_t SERVO_ANGLE_OFFSET_READ = 19;
const uint8_t SERVO_ANGLE_LIMIT_WRITE = 20;
const uint8_t SERVO_ANGLE_LIMIT_READ = 21;
const uint8_t SERVO_VIN_LIMIT_WRITE = 22;
const uint8_t SERVO_VIN_LIMIT_READ = 23;
const uint8_t SERVO_TEMP_MAX_LIMIT_WRITE = 24;
const uint8_t SERVO_TEMP_MAX_LIMIT_READ = 25;
const uint8_t SERVO_TEMP_READ = 26;
const uint8_t SERVO_VIN_READ = 27;
const uint8_t SERVO_POS_READ = 28;
const uint8_t SERVO_OR_MOTOR_MODE_WRITE = 29;
const uint8_t SERVO_OR_MOTOR_MODE_READ = 30;
const uint8_t SERVO_LOAD_OR_UNLOAD_WRITE = 31;
const uint8_t SERVO_LOAD_OR_UNLOAD_READ = 32;
const uint8_t SERVO_LED_CTRL_WRITE = 33;
const uint8_t SERVO_LED_CTRL_READ = 34;
const uint8_t SERVO_LED_ERROR_WRITE = 35;
const uint8_t SERVO_LED_ERROR_READ = 36;
class HiBusServo {
public:
  HiBusServo(HardwareSerial& serial);
  void begin(long speed = 115200);
  void moveTo(uint8_t id, float degrees, uint16_t time = 1000);
  float getPositionInDegrees(uint8_t id);
  void setAngleLimitsInDegrees(uint8_t id, float minDegrees, float maxDegrees);
  bool getAngleLimitsInDegrees(uint8_t id, float& minDegrees, float& maxDegrees);
  bool getMode(uint8_t id, uint8_t& mode, int16_t& speed);
  int8_t getAngleOffset(uint8_t id);
  void move(uint8_t id, int16_t position, uint16_t time = 1000);
  void stop(uint8_t id);
  void setID(uint8_t old_id, uint8_t new_id);
  uint8_t getID(uint8_t id = 1);
  void setAngleOffset(uint8_t id, int8_t offset);
  void saveAngleOffset(uint8_t id);
  void setAngleLimits(uint8_t id, int16_t min_angle, int16_t max_angle);
  bool getAngleLimits(uint8_t id, int16_t& minAngle, int16_t& maxAngle);
  void setVoltageLimits(uint8_t id, int16_t min_voltage, int16_t max_voltage);
  bool getVoltageLimits(uint8_t id, int16_t& minVoltage, int16_t& maxVoltage);
  void setMaxTemperatureLimit(uint8_t id, uint8_t max_temp);
  uint8_t getMaxTemperatureLimit(uint8_t id);
  int8_t getTemperature(uint8_t id);
  int16_t getVoltage(uint8_t id);
  int16_t getPosition(uint8_t id);
  void setServoMode(uint8_t id);
  void setMotorMode(uint8_t id, int16_t speed);
  void motorOn(uint8_t id);
  void motorOff(uint8_t id);
  bool isMotorOn(uint8_t id);
  void ledOn(uint8_t id);
  void ledOff(uint8_t id);
  void setLedErrors(uint8_t id, uint8_t error);  // broken?
  uint8_t getLedErrors(uint8_t id);              // broken?
private:
  HardwareSerial* _serial;
  void sendPacket(uint8_t id, uint8_t command, const uint8_t* params, uint8_t param_len);
  int receivePacket(uint8_t id, uint8_t command, uint8_t* data, int param_len);
  uint8_t checksum(const uint8_t* buf, int len);
  int16_t _degreesToPosition(float degrees);
  float _positionToDegrees(int16_t position);
};
#endif
