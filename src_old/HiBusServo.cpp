#include "HiBusServo.h"


HiBusServo::HiBusServo(int maxRetries)
    : _maxRetries(maxRetries), _serialPort(nullptr)
{
}

void HiBusServo::begin(HardwareSerial &serial)
{
    _serialPort = &serial;
    _serialPort->begin(115200);
}

void HiBusServo::setMaxRetries(int retries)
{
    _maxRetries = retries;
}

void HiBusServo::moveTo(uint8_t servoId, float destination, uint16_t time)
{
    int16_t servo_position_units = static_cast<int16_t>((destination * (500.0 / 120.0)) + 500.0);
    _serial_servo_set_mode(servoId, 0, 0);
    _serial_servo_move(servoId, servo_position_units, time);
}

void HiBusServo::spinAt(uint8_t servoId, float velocity)
{
    _serial_servo_set_mode(servoId, 1, velocity * 10);
}

void HiBusServo::stopMove(uint8_t servoId)
{
    _serial_servo_stop_move(servoId);
}

int16_t HiBusServo::readPosition(uint8_t servoId)
{
    int position = -1;
    for (int i = 0; i < _maxRetries; ++i)
    {
        position = _serial_servo_read_position(servoId);
        if (position != -1)
        {
            break;
        }
        delayMicroseconds(500);
    }
    return position;
}

int16_t HiBusServo::readVin(uint8_t servoId)
{
    int vin = -2048;
    for (int i = 0; i < _maxRetries; ++i)
    {
        vin = _serial_servo_read_vin(servoId);
        if (vin != -2048)
        {
            break;
        }
        delayMicroseconds(500);
    }
    return vin;
}

int8_t HiBusServo::readTemperature(uint8_t servoId)
{
    int8_t temperature = -1;
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_TEMP_READ, data, 1))
        {
            temperature = (int8_t)data[0];  
            break;
        }
        delayMicroseconds(500);
    }
    return temperature;
}

void HiBusServo::setServoID(uint8_t oldID, uint8_t newID)
{
    _serial_servo_set_id(oldID, newID);
}

void HiBusServo::setServoMode(uint8_t servoId, uint8_t mode, int16_t speed)
{
    _serial_servo_set_mode(servoId, mode, speed);
}

void HiBusServo::setAngleLimits(uint8_t servoId, int16_t minAngle, int16_t maxAngle)
{
    uint8_t data[4];
    data[0] = _get_low_byte(minAngle);
    data[1] = _get_high_byte(minAngle);
    data[2] = _get_low_byte(maxAngle);
    data[3] = _get_high_byte(maxAngle);
    _write_servo_param(servoId, CMD_ANGLE_LIMIT_WRITE, data, 4);
}

void HiBusServo::setVinLimits(uint8_t servoId, uint16_t minVin, uint16_t maxVin)
{
    uint8_t data[4];
    data[0] = _get_low_byte(minVin);
    data[1] = _get_high_byte(minVin);
    data[2] = _get_low_byte(maxVin);
    data[3] = _get_high_byte(maxVin);
    _write_servo_param(servoId, CMD_VIN_LIMIT_WRITE, data, 4);
}

void HiBusServo::setTempMaxLimit(uint8_t servoId, uint8_t maxTemp)
{
    _write_servo_param(servoId, CMD_TEMP_MAX_LIMIT_WRITE, &maxTemp, 1);
}

void HiBusServo::setAngleOffset(uint8_t servoId, int8_t offset)
{
    uint8_t data = static_cast<uint8_t>(offset);
    _write_servo_param(servoId, CMD_ANGLE_OFFSET_WRITE, &data, 1);
}

void HiBusServo::loadServo(uint8_t servoId)
{
    _serial_servo_load(servoId);
}

void HiBusServo::unloadServo(uint8_t servoId)
{
    _serial_servo_unload(servoId);
}

void HiBusServo::setLEDControl(uint8_t servoId, bool ledOn)
{
    uint8_t data = ledOn ? 1 : 0;
    _write_servo_param(servoId, CMD_LED_CTRL_WRITE, &data, 1);
}

void HiBusServo::setLEDErrorControl(uint8_t servoId, bool errorLedOn)
{
    uint8_t data = errorLedOn ? 1 : 0;
    _write_servo_param(servoId, CMD_LED_ERROR_WRITE, &data, 1);
}

bool HiBusServo::getServoMode(uint8_t servoId, uint8_t &mode, int16_t &speed)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[4];
        if (_read_servo_param(servoId, CMD_OR_MOTOR_MODE_READ, data, 4))
        {
            mode = data[0];
            speed = _bytes_to_word(data[3], data[2]);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getAngleLimits(uint8_t servoId, int16_t &minAngle, int16_t &maxAngle)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[4];
        if (_read_servo_param(servoId, CMD_ANGLE_LIMIT_READ, data, 4))
        {
            minAngle = _bytes_to_word(data[1], data[0]);
            maxAngle = _bytes_to_word(data[3], data[2]);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getVinLimits(uint8_t servoId, uint16_t &minVin, uint16_t &maxVin)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[4];
        if (_read_servo_param(servoId, CMD_VIN_LIMIT_READ, data, 4))
        {
            minVin = _bytes_to_word(data[1], data[0]);
            maxVin = _bytes_to_word(data[3], data[2]);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getTempMaxLimit(uint8_t servoId, uint8_t &maxTemp)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_TEMP_MAX_LIMIT_READ, data, 1))
        {
            maxTemp = data[0];
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getAngleOffset(uint8_t servoId, int8_t &offset)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_ANGLE_OFFSET_READ, data, 1))
        {
            offset = static_cast<int8_t>(data[0]);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getLEDControl(uint8_t servoId, bool &ledOn)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_LED_CTRL_READ, data, 1))
        {
            ledOn = (data[0] != 0);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

bool HiBusServo::getLEDErrorControl(uint8_t servoId, bool &errorLedOn)
{
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_LED_ERROR_READ, data, 1))
        {
            errorLedOn = (data[0] != 0);
            return true;
        }
        delayMicroseconds(500);
    }
    return false;
}

uint8_t HiBusServo::_calculate_checksum(uint8_t buf[])
{
    uint16_t temp = 0;
    for (uint8_t i = 2; i < buf[3] + 2; i++)
    {
        temp += buf[i];
    }
    temp = ~temp;
    return (uint8_t)temp;
}

void HiBusServo::_serial_servo_move(uint8_t id, int16_t position, uint16_t time)
{
    uint8_t buf[10];
    if (position < 0) position = 0;
    if (position > 1000) position = 1000;
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = CMD_MOVE_TIME_WRITE;
    buf[5] = _get_low_byte(position);
    buf[6] = _get_high_byte(position);
    buf[7] = _get_low_byte(time);
    buf[8] = _get_high_byte(time);
    buf[9] = _calculate_checksum(buf);
    _serialPort->write(buf, 10);
}

void HiBusServo::_serial_servo_stop_move(uint8_t id)
{
    uint8_t buf[6];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = CMD_MOVE_STOP;
    buf[5] = _calculate_checksum(buf);
    _serialPort->write(buf, 6);
}

void HiBusServo::_serial_servo_set_id(uint8_t oldID, uint8_t newID)
{
    uint8_t buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = oldID;
    buf[3] = 4;
    buf[4] = CMD_ID_WRITE;
    buf[5] = newID;
    buf[6] = _calculate_checksum(buf);
    _serialPort->write(buf, 7);
}

void HiBusServo::_serial_servo_set_mode(uint8_t id, uint8_t mode, int16_t speed)
{
    uint8_t buf[10];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = CMD_OR_MOTOR_MODE_WRITE;
    buf[5] = mode;
    buf[6] = 0;
    buf[7] = _get_low_byte((uint16_t)speed);
    buf[8] = _get_high_byte((uint16_t)speed);
    buf[9] = _calculate_checksum(buf);
    _serialPort->write(buf, 10);
}

void HiBusServo::_serial_servo_load(uint8_t id)
{
    uint8_t buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = CMD_LOAD_OR_UNLOAD_WRITE;
    buf[5] = 1;
    buf[6] = _calculate_checksum(buf);
    _serialPort->write(buf, 7);
}

void HiBusServo::_serial_servo_unload(uint8_t id)
{
    uint8_t buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = CMD_LOAD_OR_UNLOAD_WRITE;
    buf[5] = 0;
    buf[6] = _calculate_checksum(buf);
    _serialPort->write(buf, 7);
}

int16_t HiBusServo::_serial_servo_receive_handle(uint8_t *ret)
{
    bool frameStarted = false;
    bool receiveFinished = false;
    uint8_t frameCount = 0;
    uint8_t dataCount = 0;
    uint8_t dataLength = 2;
    uint8_t rxBuf;
    uint8_t recvBuf[32];

    while (_serialPort->available())
    {
        rxBuf = _serialPort->read();
        delayMicroseconds(100);
        if (!frameStarted)
        {
            if (rxBuf == FRAME_HEADER)
            {
                frameCount++;
                if (frameCount == 2)
                {
                    frameCount = 0;
                    frameStarted = true;
                    dataCount = 1;
                }
            }
            else
            {
                frameStarted = false;
                dataCount = 0;
                frameCount = 0;
            }
        }
        if (frameStarted)
        {
            recvBuf[dataCount] = (uint8_t)rxBuf;
            if (dataCount == 3)
            {
                dataLength = recvBuf[dataCount];
                if (dataLength < 3 || dataCount > 7)
                {
                    dataLength = 2;
                    frameStarted = false;
                }
            }
            dataCount++;
            if (dataCount == dataLength + 3)
            {
                if (_calculate_checksum(recvBuf) == recvBuf[dataCount - 1])
                {
                    frameStarted = false;
                    memcpy(ret, recvBuf + 4, dataLength);
                    return 1;
                }
                return -1;
            }
        }
    }
    return 0;
}

int16_t HiBusServo::_serial_servo_read_position(uint8_t id)
{
    int count = 10000;
    int ret;
    uint8_t buf[6];

    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = CMD_POS_READ;
    buf[5] = _calculate_checksum(buf);

    while (_serialPort->available())
        _serialPort->read();

    _serialPort->write(buf, 6);

    while (!_serialPort->available())
    {
        count -= 1;
        if (count < 0)
        {
            return -1;
        }
    }

    if (_serial_servo_receive_handle(buf) > 0)
    {
        ret = _bytes_to_word(buf[2], buf[1]);
    }
    else
    {
        ret = -1;
    }

    return ret;
}

int16_t HiBusServo::_serial_servo_read_vin(uint8_t id)
{
    int count = 10000;
    int ret;
    uint8_t buf[6];

    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = CMD_VIN_READ;
    buf[5] = _calculate_checksum(buf);

    while (_serialPort->available())
        _serialPort->read();

    _serialPort->write(buf, 6);

    while (!_serialPort->available())
    {
        count -= 1;
        if (count < 0)
        {
            return -2048;
        }
    }

    if (_serial_servo_receive_handle(buf) > 0)
    {
        ret = (int16_t)_bytes_to_word(buf[2], buf[1]);
    }
    else
    {
        ret = -2048;
    }

    return ret;
}

void HiBusServo::_write_servo_param(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t dataLen)
{
    uint8_t buf[32];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = dataLen + 3;
    buf[4] = cmd;
    
    for (uint8_t i = 0; i < dataLen; i++)
    {
        buf[5 + i] = data[i];
    }
    
    buf[5 + dataLen] = _calculate_checksum(buf);
    _serialPort->write(buf, 6 + dataLen);
}

bool HiBusServo::_read_servo_param(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t expectedLen)
{
    uint8_t buf[32];
    int count = 10000;

    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = cmd;
    buf[5] = _calculate_checksum(buf);

    while (_serialPort->available())
        _serialPort->read();

    _serialPort->write(buf, 6);

    while (!_serialPort->available())
    {
        count -= 1;
        if (count < 0)
        {
            return false;
        }
    }

    if (_serial_servo_receive_handle(buf) > 0)
    {
        memcpy(data, buf, expectedLen);
        return true;
    }

    return false;
}
