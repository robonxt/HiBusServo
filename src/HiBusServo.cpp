#include "HiBusServo.h"

/**
 * @brief Construct a new HiBusServo::HiBusServo object
 * 
 * @param debug Enable debug output
 * @param maxRetries Maximum number of retries for reading from servo
 */
HiBusServo::HiBusServo(int maxRetries)
    : _maxRetries(maxRetries), _serialPort(nullptr)
{
}

/**
 * @brief Initialize the serial port for communication with the servo
 * 
 * @param serial The serial port to use
 */
void HiBusServo::begin(HardwareSerial &serial)
{
    _serialPort = &serial;
    _serialPort->begin(115200);
}

/**
 * @brief Set the maximum number of retries for reading from the servo
 * 
 * @param retries The maximum number of retries
 */
void HiBusServo::setMaxRetries(int retries)
{
    _maxRetries = retries;
}

// Basic movement commands
/**
 * @brief Move the servo to a specific position in a given time
 * 
 * @param servoId The ID of the servo
 * @param destination The destination angle (-120 to 120 degrees)
 * @param time The time to complete the movement in milliseconds
 */
void HiBusServo::moveTo(int servoId, float destination, uint16_t time)
{
    // Input is in angle relative to center as zero, from -120 to 120.
    // Negative numbers clockwise (when looking at face of servo.)
    int16_t servo_position_units = static_cast<int16_t>((destination * (500.0 / 120.0)) + 500.0);
    
    // Set to servo mode first, then move
    _serial_servo_set_mode(servoId, 0, 0);
    _serial_servo_move(servoId, servo_position_units, time);
}

/**
 * @brief Spin the servo at a specific velocity
 * 
 * @param servoId The ID of the servo
 * @param velocity The velocity to spin at (-100 to 100)
 */
void HiBusServo::spinAt(int servoId, float velocity)
{
    // Input should be from -100 to 100, multiply by 10 to get -1000 to 1000 range
    _serial_servo_set_mode(servoId, 1, velocity * 10);
}

/**
 * @brief Stop the servo's movement
 * 
 * @param servoId The ID of the servo
 */
void HiBusServo::stopMove(int servoId)
{
    _serial_servo_stop_move(servoId);
}

// Position and status reading
/**
 * @brief Read the current position of the servo
 * 
 * @param servoId The ID of the servo
 * @return int The current position of the servo, or -1 on failure
 */
int HiBusServo::readPosition(int servoId)
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

/**
 * @brief Read the input voltage of the servo
 * 
 * @param servoId The ID of the servo
 * @return int The input voltage in mV, or -2048 on failure
 */
int HiBusServo::readVin(int servoId)
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

/**
 * @brief Read the temperature of the servo
 * 
 * @param servoId The ID of the servo
 * @return int The temperature in degrees Celsius, or -1 on failure
 */
int HiBusServo::readTemperature(int servoId)
{
    int temperature = -1;
    for (int i = 0; i < _maxRetries; ++i)
    {
        uint8_t data[1];
        if (_read_servo_param(servoId, CMD_TEMP_READ, data, 1))
        {
            temperature = data[0];
            break;
        }
        delayMicroseconds(500);
    }
    return temperature;
}

// Servo configuration
/**
 * @brief Set the ID of the servo
 * 
 * @param oldID The current ID of the servo
 * @param newID The new ID for the servo
 */
void HiBusServo::setServoID(int oldID, int newID)
{
    _serial_servo_set_id(oldID, newID);
}

/**
 * @brief Set the mode of the servo
 * 
 * @param servoId The ID of the servo
 * @param mode The mode to set (0 for position, 1 for wheel)
 * @param speed The speed for wheel mode
 */
void HiBusServo::setServoMode(int servoId, uint8_t mode, int16_t speed)
{
    _serial_servo_set_mode(servoId, mode, speed);
}

/**
 * @brief Set the angle limits for the servo
 * 
 * @param servoId The ID of the servo
 * @param minAngle The minimum angle limit
 * @param maxAngle The maximum angle limit
 */
void HiBusServo::setAngleLimits(int servoId, int16_t minAngle, int16_t maxAngle)
{
    uint8_t data[4];
    data[0] = _get_low_byte(minAngle);
    data[1] = _get_high_byte(minAngle);
    data[2] = _get_low_byte(maxAngle);
    data[3] = _get_high_byte(maxAngle);
    _write_servo_param(servoId, CMD_ANGLE_LIMIT_WRITE, data, 4);
}

/**
 * @brief Set the input voltage limits for the servo
 * 
 * @param servoId The ID of the servo
 * @param minVin The minimum voltage limit in mV
 * @param maxVin The maximum voltage limit in mV
 */
void HiBusServo::setVinLimits(int servoId, uint16_t minVin, uint16_t maxVin)
{
    uint8_t data[4];
    data[0] = _get_low_byte(minVin);
    data[1] = _get_high_byte(minVin);
    data[2] = _get_low_byte(maxVin);
    data[3] = _get_high_byte(maxVin);
    _write_servo_param(servoId, CMD_VIN_LIMIT_WRITE, data, 4);
}

/**
 * @brief Set the maximum temperature limit for the servo
 * 
 * @param servoId The ID of the servo
 * @param maxTemp The maximum temperature limit in degrees Celsius
 */
void HiBusServo::setTempMaxLimit(int servoId, uint8_t maxTemp)
{
    _write_servo_param(servoId, CMD_TEMP_MAX_LIMIT_WRITE, &maxTemp, 1);
}

/**
 * @brief Set the angle offset for the servo
 * 
 * @param servoId The ID of the servo
 * @param offset The angle offset
 */
void HiBusServo::setAngleOffset(int servoId, int8_t offset)
{
    uint8_t data = static_cast<uint8_t>(offset);
    _write_servo_param(servoId, CMD_ANGLE_OFFSET_WRITE, &data, 1);
}

// Servo control
/**
 * @brief Enable the servo's torque
 * 
 * @param servoId The ID of the servo
 */
void HiBusServo::loadServo(int servoId)
{
    _serial_servo_load(servoId);
}

/**
 * @brief Disable the servo's torque
 * 
 * @param servoId The ID of the servo
 */
void HiBusServo::unloadServo(int servoId)
{
    _serial_servo_unload(servoId);
}

/**
 * @brief Set the servo's LED state
 * 
 * @param servoId The ID of the servo
 * @param ledOn True to turn the LED on, false to turn it off
 */
void HiBusServo::setLEDControl(int servoId, bool ledOn)
{
    uint8_t data = ledOn ? 1 : 0;
    _write_servo_param(servoId, CMD_LED_CTRL_WRITE, &data, 1);
}

/**
 * @brief Set the servo's LED error indicator state
 * 
 * @param servoId The ID of the servo
 * @param errorLedOn True to turn the error LED on, false to turn it off
 */
void HiBusServo::setLEDErrorControl(int servoId, bool errorLedOn)
{
    uint8_t data = errorLedOn ? 1 : 0;
    _write_servo_param(servoId, CMD_LED_ERROR_WRITE, &data, 1);
}

// Getters for servo parameters
/**
 * @brief Get the servo's mode
 * 
 * @param servoId The ID of the servo
 * @param mode Variable to store the mode
 * @param speed Variable to store the speed
 * @return true if successful, false otherwise
 */
bool HiBusServo::getServoMode(int servoId, uint8_t &mode, int16_t &speed)
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

/**
 * @brief Get the servo's angle limits
 * 
 * @param servoId The ID of the servo
 * @param minAngle Variable to store the minimum angle
 * @param maxAngle Variable to store the maximum angle
 * @return true if successful, false otherwise
 */
bool HiBusServo::getAngleLimits(int servoId, int16_t &minAngle, int16_t &maxAngle)
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

/**
 * @brief Get the servo's input voltage limits
 * 
 * @param servoId The ID of the servo
 * @param minVin Variable to store the minimum voltage
 * @param maxVin Variable to store the maximum voltage
 * @return true if successful, false otherwise
 */
bool HiBusServo::getVinLimits(int servoId, uint16_t &minVin, uint16_t &maxVin)
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

/**
 * @brief Get the servo's maximum temperature limit
 * 
 * @param servoId The ID of the servo
 * @param maxTemp Variable to store the maximum temperature
 * @return true if successful, false otherwise
 */
bool HiBusServo::getTempMaxLimit(int servoId, uint8_t &maxTemp)
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

/**
 * @brief Get the servo's angle offset
 * 
 * @param servoId The ID of the servo
 * @param offset Variable to store the angle offset
 * @return true if successful, false otherwise
 */
bool HiBusServo::getAngleOffset(int servoId, int8_t &offset)
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

/**
 * @brief Get the servo's LED state
 * 
 * @param servoId The ID of the servo
 * @param ledOn Variable to store the LED state
 * @return true if successful, false otherwise
 */
bool HiBusServo::getLEDControl(int servoId, bool &ledOn)
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

/**
 * @brief Get the servo's LED error indicator state
 * 
 * @param servoId The ID of the servo
 * @param errorLedOn Variable to store the LED error state
 * @return true if successful, false otherwise
 */
bool HiBusServo::getLEDErrorControl(int servoId, bool &errorLedOn)
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

// Private helper methods
/**
 * @brief Calculate the checksum for a command packet
 * 
 * @param buf The buffer containing the command packet
 * @return uint8_t The calculated checksum
 */
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

/**
 * @brief Send a move command to the servo
 * 
 * @param id The ID of the servo
 * @param position The target position (0-1000)
 * @param time The time for the movement in milliseconds
 */
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

/**
 * @brief Send a stop move command to the servo
 * 
 * @param id The ID of the servo
 */
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

/**
 * @brief Send a set ID command to the servo
 * 
 * @param oldID The current ID of the servo
 * @param newID The new ID for the servo
 */
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

/**
 * @brief Send a set mode command to the servo
 * 
 * @param id The ID of the servo
 * @param mode The mode to set
 * @param speed The speed for wheel mode
 */
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

/**
 * @brief Send a load (enable torque) command to the servo
 * 
 * @param id The ID of the servo
 */
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

/**
 * @brief Send an unload (disable torque) command to the servo
 * 
 * @param id The ID of the servo
 */
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

/**
 * @brief Handle receiving data from the servo
 * 
 * @param ret Buffer to store the received data
 * @return int 1 on success, -1 on checksum error, 0 if no data
 */
int HiBusServo::_serial_servo_receive_handle(uint8_t *ret)
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

/**
 * @brief Read the position from the servo
 * 
 * @param id The ID of the servo
 * @return int The position, or -1 on failure
 */
int HiBusServo::_serial_servo_read_position(uint8_t id)
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

/**
 * @brief Read the input voltage from the servo
 * 
 * @param id The ID of the servo
 * @return int The input voltage in mV, or -2048 on failure
 */
int HiBusServo::_serial_servo_read_vin(uint8_t id)
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

// Generic read/write methods for new functionality
/**
 * @brief Generic method to write a parameter to the servo
 * 
 * @param id The ID of the servo
 * @param cmd The command to send
 * @param data The data to send
 * @param dataLen The length of the data
 */
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

/**
 * @brief Generic method to read a parameter from the servo
 * 
 * @param id The ID of the servo
 * @param cmd The command to send
 * @param data Buffer to store the received data
 * @param expectedLen The expected length of the data
 * @return true if successful, false otherwise
 */
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
