#include "test.h"
LewanSoul_LX16A_Enhanced::LewanSoul_LX16A_Enhanced(HardwareSerial& serial) {
  _serial = &serial;
}
void LewanSoul_LX16A_Enhanced::begin(long speed) {
  _serial->begin(speed);
}
void LewanSoul_LX16A_Enhanced::moveTo(uint8_t id, float degrees, uint16_t time) {
  int16_t position = _degreesToPosition(degrees);
  move(id, position, time);
}
float LewanSoul_LX16A_Enhanced::getPositionInDegrees(uint8_t id) {
  int16_t position = getPosition(id);
  if (position == -1) return -999.0;
  return _positionToDegrees(position);
}
void LewanSoul_LX16A_Enhanced::setAngleLimitsInDegrees(uint8_t id, float minDegrees, float maxDegrees) {
  int16_t minPos = _degreesToPosition(minDegrees);
  int16_t maxPos = _degreesToPosition(maxDegrees);
  setAngleLimits(id, minPos, maxPos);
}
bool LewanSoul_LX16A_Enhanced::getAngleLimitsInDegrees(uint8_t id, float& minDegrees, float& maxDegrees) {
  int16_t minPos, maxPos;
  if (getAngleLimits(id, minPos, maxPos)) {
    minDegrees = _positionToDegrees(minPos);
    maxDegrees = _positionToDegrees(maxPos);
    return true;
  }
  return false;
}
bool LewanSoul_LX16A_Enhanced::getMode(uint8_t id, uint8_t& mode, int16_t& speed) {
  sendPacket(id, SERVO_OR_MOTOR_MODE_READ, NULL, 0);
  uint8_t data[4];
  if (receivePacket(id, SERVO_OR_MOTOR_MODE_READ, data, 4) > 0) {
    mode = data[0];
    speed = (int16_t)word(data[3], data[2]);
    return true;
  }
  return false;
}
int8_t LewanSoul_LX16A_Enhanced::getAngleOffset(uint8_t id) {
  sendPacket(id, SERVO_ANGLE_OFFSET_READ, NULL, 0);
  uint8_t data[1];
  if (receivePacket(id, SERVO_ANGLE_OFFSET_READ, data, 1) > 0) {
    return (int8_t)data[0];
  }
  return 127;
}
int16_t LewanSoul_LX16A_Enhanced::_degreesToPosition(float degrees) {
  degrees = constrain(degrees, -120.0, 120.0);
  return (int16_t)((degrees * (500.0 / 120.0)) + 500.0);
}
float LewanSoul_LX16A_Enhanced::_positionToDegrees(int16_t position) {
  return (float)((position - 500.0) * (120.0 / 500.0));
}
uint8_t LewanSoul_LX16A_Enhanced::checksum(const uint8_t* buf, int len) {
  uint8_t sum = 0;
  for (int i = 2; i < len - 1; i++) {
    sum += buf[i];
  }
  return ~sum;
}
void LewanSoul_LX16A_Enhanced::sendPacket(uint8_t id, uint8_t command, const uint8_t* params, uint8_t param_len) {
  while (_serial->available()) {
    _serial->read();
  }
  uint8_t tx_buf[10];
  tx_buf[0] = 0x55;
  tx_buf[1] = 0x55;
  tx_buf[2] = id;
  tx_buf[3] = 3 + param_len;
  tx_buf[4] = command;
  if (params != NULL) {
    memcpy(&tx_buf[5], params, param_len);
  }
  tx_buf[5 + param_len] = checksum(tx_buf, 6 + param_len);
  _serial->write(tx_buf, 6 + param_len);
  _serial->flush();
}
int LewanSoul_LX16A_Enhanced::receivePacket(uint8_t id, uint8_t command, uint8_t* data, int param_len) {
  const int SERVO_PROCESSING_DELAY_MS = 5;
  int expected_packet_len = param_len + 6;
  unsigned long timeout_ms = expected_packet_len + SERVO_PROCESSING_DELAY_MS;
  unsigned long start_time = millis();
  int got_bytes = 0;
  int servo_packet_len = 0;
  uint8_t checksum_val = 0;
  while (millis() - start_time < timeout_ms) {
    if (_serial->available()) {
      uint8_t ch = _serial->read();
      switch (got_bytes) {
        case 0:
        case 1:
          if (ch == 0x55) {
            got_bytes++;
          } else {
            got_bytes = 0;
          }
          break;
        case 2:
          if (ch == id) {
            checksum_val += ch;
            got_bytes++;
          } else {
            got_bytes = 0;
          }
          break;
        case 3:
          {
            servo_packet_len = ch;
            bool is_buggy_command = (command == SERVO_LED_ERROR_READ && servo_packet_len == 4);
            if (is_buggy_command) {
              servo_packet_len = 1 + 3;
            }
            if (servo_packet_len == param_len + 3) {
              checksum_val += ch;
              got_bytes++;
            } else {
              got_bytes = 0;
            }
            break;
          }
        case 4:
          if (ch == command) {
            checksum_val += ch;
            got_bytes++;
          } else {
            got_bytes = 0;
          }
          break;
        default:
          if (got_bytes < (servo_packet_len + 2)) {
            data[got_bytes - 5] = ch;
            checksum_val += ch;
            got_bytes++;
          } else {
            if ((uint8_t)ch == (uint8_t)~checksum_val) {
              return param_len;
            }
            return -1;
          }
          break;
      }
    }
  }
  return -1;
}
void LewanSoul_LX16A_Enhanced::move(uint8_t id, int16_t position, uint16_t time) {
  position = constrain(position, 0, 1000);
  uint8_t params[4];
  params[0] = lowByte(position);
  params[1] = highByte(position);
  params[2] = lowByte(time);
  params[3] = highByte(time);
  sendPacket(id, SERVO_MOVE_TIME_WRITE, params, 4);
}
void LewanSoul_LX16A_Enhanced::stop(uint8_t id) {
  sendPacket(id, SERVO_MOVE_STOP, NULL, 0);
}
void LewanSoul_LX16A_Enhanced::setID(uint8_t old_id, uint8_t new_id) {
  uint8_t params[1] = { new_id };
  sendPacket(old_id, SERVO_ID_WRITE, params, 1);
}
uint8_t LewanSoul_LX16A_Enhanced::getID(uint8_t id) {
  sendPacket(id, SERVO_ID_READ, NULL, 0);
  uint8_t data[1];
  if (receivePacket(id, SERVO_ID_READ, data, 1) > 0) {
    return data[0];
  }
  return 255;
}
void LewanSoul_LX16A_Enhanced::setAngleOffset(uint8_t id, int8_t offset) {
  uint8_t params[1] = { (uint8_t)constrain(offset, -125, 125) };
  sendPacket(id, SERVO_ANGLE_OFFSET_ADJUST, params, 1);
}
void LewanSoul_LX16A_Enhanced::saveAngleOffset(uint8_t id) {
  sendPacket(id, SERVO_ANGLE_OFFSET_WRITE, NULL, 0);
}
void LewanSoul_LX16A_Enhanced::setAngleLimits(uint8_t id, int16_t min_angle, int16_t max_angle) {
  min_angle = constrain(min_angle, 0, 1000);
  max_angle = constrain(max_angle, 0, 1000);
  uint8_t params[4];
  params[0] = lowByte(min_angle);
  params[1] = highByte(min_angle);
  params[2] = lowByte(max_angle);
  params[3] = highByte(max_angle);
  sendPacket(id, SERVO_ANGLE_LIMIT_WRITE, params, 4);
}
bool LewanSoul_LX16A_Enhanced::getAngleLimits(uint8_t id, int16_t& minAngle, int16_t& maxAngle) {
  sendPacket(id, SERVO_ANGLE_LIMIT_READ, NULL, 0);
  uint8_t data[4];
  if (receivePacket(id, SERVO_ANGLE_LIMIT_READ, data, 4) > 0) {
    minAngle = (int16_t)word(data[1], data[0]);
    maxAngle = (int16_t)word(data[3], data[2]);
    return true;
  }
  return false;
}
void LewanSoul_LX16A_Enhanced::setVoltageLimits(uint8_t id, int16_t min_voltage, int16_t max_voltage) {
  uint8_t params[4];
  params[0] = lowByte(min_voltage);
  params[1] = highByte(min_voltage);
  params[2] = lowByte(max_voltage);
  params[3] = highByte(max_voltage);
  sendPacket(id, SERVO_VIN_LIMIT_WRITE, params, 4);
}
bool LewanSoul_LX16A_Enhanced::getVoltageLimits(uint8_t id, int16_t& minVoltage, int16_t& maxVoltage) {
  sendPacket(id, SERVO_VIN_LIMIT_READ, NULL, 0);
  uint8_t data[4];
  if (receivePacket(id, SERVO_VIN_LIMIT_READ, data, 4) > 0) {
    minVoltage = (int16_t)word(data[1], data[0]);
    maxVoltage = (int16_t)word(data[3], data[2]);
    return true;
  }
  return false;
}
void LewanSoul_LX16A_Enhanced::setMaxTemperatureLimit(uint8_t id, uint8_t max_temp) {
  max_temp = constrain(max_temp, 50, 100);
  uint8_t params[1] = { max_temp };
  sendPacket(id, SERVO_TEMP_MAX_LIMIT_WRITE, params, 1);
}
uint8_t LewanSoul_LX16A_Enhanced::getMaxTemperatureLimit(uint8_t id) {
  sendPacket(id, SERVO_TEMP_MAX_LIMIT_READ, NULL, 0);
  uint8_t data[1];
  if (receivePacket(id, SERVO_TEMP_MAX_LIMIT_READ, data, 1) > 0) {
    return data[0];
  }
  return 0;
}
int8_t LewanSoul_LX16A_Enhanced::getTemperature(uint8_t id) {
  sendPacket(id, SERVO_TEMP_READ, NULL, 0);
  uint8_t data[1];
  if (receivePacket(id, SERVO_TEMP_READ, data, 1) > 0) {
    return (int8_t)data[0];
  }
  return -1;
}
int16_t LewanSoul_LX16A_Enhanced::getVoltage(uint8_t id) {
  sendPacket(id, SERVO_VIN_READ, NULL, 0);
  uint8_t data[2];
  if (receivePacket(id, SERVO_VIN_READ, data, 2) > 0) {
    return (int16_t)word(data[1], data[0]);
  }
  return -1;
}
int16_t LewanSoul_LX16A_Enhanced::getPosition(uint8_t id) {
  sendPacket(id, SERVO_POS_READ, NULL, 0);
  uint8_t data[2];
  if (receivePacket(id, SERVO_POS_READ, data, 2) > 0) {
    return (int16_t)word(data[1], data[0]);
  }
  return -1;
}
void LewanSoul_LX16A_Enhanced::setServoMode(uint8_t id) {
  uint8_t params[4] = { 0, 0, 0, 0 };
  sendPacket(id, SERVO_OR_MOTOR_MODE_WRITE, params, 4);
}
void LewanSoul_LX16A_Enhanced::setMotorMode(uint8_t id, int16_t speed) {
  speed = constrain(speed, -1000, 1000);
  uint8_t params[4];
  params[0] = 1;
  params[1] = 0;
  params[2] = lowByte(speed);
  params[3] = highByte(speed);
  sendPacket(id, SERVO_OR_MOTOR_MODE_WRITE, params, 4);
}
void LewanSoul_LX16A_Enhanced::motorOn(uint8_t id) {
  uint8_t params[1] = { 1 };
  sendPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, params, 1);
}
void LewanSoul_LX16A_Enhanced::motorOff(uint8_t id) {
  uint8_t params[1] = { 0 };
  sendPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, params, 1);
}
void LewanSoul_LX16A_Enhanced::ledOn(uint8_t id) {
  uint8_t params[1] = { 1 };
  sendPacket(id, SERVO_LED_CTRL_WRITE, params, 1);
}
void LewanSoul_LX16A_Enhanced::ledOff(uint8_t id) {
  uint8_t params[1] = { 0 };
  sendPacket(id, SERVO_LED_CTRL_WRITE, params, 1);
}
void LewanSoul_LX16A_Enhanced::setLedErrors(uint8_t id, uint8_t error) {
  uint8_t params[1] = { error };
  sendPacket(id, SERVO_LED_ERROR_WRITE, params, 1);
}
uint8_t LewanSoul_LX16A_Enhanced::getLedErrors(uint8_t id) {
  sendPacket(id, SERVO_LED_ERROR_READ, NULL, 0);
  uint8_t data[1];
  if (receivePacket(id, SERVO_LED_ERROR_READ, data, 1) > 0) {
    return data[0];
  }
  return 255;
}