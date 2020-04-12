/**
 * @file i2c_device.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-01
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#include "device.h"
#include "CommManager/i2c_bus.h"
#include "devices/hardware.h"
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

class I2CDevice : public Device
{
private:
  I2CBus* _i2c_bus;
  uint8_t _chip_id = 0x00;
  int _address;

protected:
#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  diagnostic_msgs::KeyValue _diagnostic_chip_id;
#endif
#endif
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
/**
 * @brief Construct a new I2CDevice object
 *
 * @param address
 * @param bus_Id
 * @param dev_index
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  I2CDevice(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
            uint8_t dev_index = 0, const char* dev_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : Device(dev_index, nh, dev_name, topic_name, refresh_rate)
    , _address(address << 1)
    , _i2c_bus(&i2c_bus)
  {
#else
  I2CDevice(uint8_t address, I2CBus& i2c_bus, ros::NodeHandle& nh,
            uint8_t dev_index = 0, const char* dev_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : Device(dev_index, nh, dev_name, topic_name, refresh_rate)
    , _address(address)
    , _i2c_bus(&i2c_bus)
  {
#endif
    // _i2c_bus = &i2c_bus;
    setIndex(dev_index);
#ifndef DISABLE_DIAGNOSTICS
    _diagnostic_chip_id.key = "Chip Id";
    _diagnostic_chip_id.value = (char*)0x00;
    this->setDiagnosticsData(_diagnostic_chip_id);
#endif
  }
#else
/**
 * @brief Construct a new I2CDevice object
 *
 * @param address
 * @param bus_Id
 * @param dev_index
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  I2CDevice(int address, I2CBus& i2c_bus, uint8_t dev_index = 0,
            int refresh_rate = 1)
    : Device(dev_index, refresh_rate), _address(address << 1)
  {
#else
  I2CDevice(int address, I2CBus& i2c_bus, uint8_t dev_index = 0,
            int refresh_rate = 1)
    : Device(dev_index, refresh_rate), _address(address)
  {
#endif
    _i2c_bus = &i2c_bus;
    setIndex(dev_index);
  }
#endif

  /** DESTRUCTOR */

  /**
   * @brief Destroy the I2CDevice object
   *
   */
  ~I2CDevice()
  {
  }

  /** METHODS */

#ifdef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual bool initialize()
  {
#ifdef DISABLE_ROS
    print("Entered I2CDevice Initialize\n");
#endif
    _i2c_bus->initialize();
#ifdef DISABLE_ROS
    print("Exited I2CDevice Initialize\n");
#endif
    return true;
  }
#endif

/**
 * @brief Ping the device to check if connection can be established
 *
 * @param chip_id_reg_address
 * @return true
 * @return false
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual bool ping(int chip_id_reg_address = 0x00, int delay_ms = 0)
  {
#else
  virtual bool ping(uint8_t chip_id_reg_address = 0x00, int delay_ms = 0)
  {
#endif
    uint8_t buffer;
    if (readRegister(chip_id_reg_address, &buffer, 1))
    {
      setHealthStatus(true);
      setConfiguredStatus(true);
      setEnabledStatus(true);
      _chip_id = buffer;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
      _diagnostic_chip_id.value = (char*)buffer;
      this->setDiagnosticsData(_diagnostic_chip_id);
#endif
#else
      char str[100];
      sprintf(str, "Chip Id is: %x", buffer);
      print(str);
#endif
      return true;
    }
    else
      return false;
  }

  /**
   * @brief Read register from device
   *
   * @detail This function also calls a write before the read.
   *
   * @param register address
   * @param buffer
   * @param buffer_size
   * @return true if read successful
   * @return false if write to register unsuccessfull
   */

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual bool readRegister(int address, uint8_t* buffer, int buffer_size,
                            bool poll = false, int delay_ms = 0)
  {
    char reg_address[1] = { address };
    int write_state = !_i2c_bus->write(_address, reg_address, 1, true);
    wait_ms(delay_ms);
    int read_state = this->readBytes((char*)buffer, buffer_size);

#ifdef DISABLE_ROS
    char str[100];
    sprintf(str, "Write State: %d\n", write_state);
    print(str);
    sprintf(str, "Read State: %d\n", read_state);
    print(str);
#endif

    return (write_state && read_state);
#else
  virtual bool readRegister(uint8_t address, uint8_t* buffer,
                            uint8_t buffer_size, bool poll = false,
                            int delay_ms = 0)
  {
    if (writeBytes(&address, 1))
    {
      delay(delay_ms);
      return readBytes(buffer, buffer_size) == buffer_size;
    }

    return false;

#endif
  }

/**
 * @brief Read bytes from a device, no write before the read.
 *
 * @param buffer
 * @param buffer_size
 * @return true if ACK
 * @return false if NACK
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual bool readBytes(char* buffer, int buffer_size, bool poll = false)
  {
    return _i2c_bus->read(_address, buffer, buffer_size, poll) == 0;
  }
#else
  virtual bool readBytes(uint8_t* buffer, int buffer_size, bool poll = false)
  {
    _i2c_bus->requestFrom(_address, buffer_size);
    int read_ready = 0;
    while (_i2c_bus->available() && (read_ready < buffer_size))
    {
      *(buffer + read_ready) = (uint8_t)_i2c_bus->read();
      read_ready++;
    }

    return read_ready == buffer_size;
  }
#endif

  /**
   * @brief Reset the pin for the Device and wait for a delay in ms before
   * restarting it
   *
   * @param pin
   * @param delay_ms
   */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual void reset(PinName pin, const uint8_t delay_ms = 100)
  {
    DigitalOut resetPin(pin, false);
    wait_ms(delay_ms);
    resetPin = true;
#else
  virtual void reset(int pin, const uint8_t delay_ms = 100)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, false);
    delay(delay_ms);
    digitalWrite(pin, true);
#endif
  }

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  /**
   * @brief Write to a register on a device
   *
   * @param uint8_t register address
   * @param char* buffer
   * @param int buffer_size
   * @return write Status
   * @return true if ACK
   * @return false if NACK
   */
  virtual int writeRegister(uint8_t address, char* buffer, int buffer_size,
                            bool poll = false)
  {
    char new_buffer[buffer_size + 1];
    new_buffer[0] = (char)address;
    memcpy(&new_buffer[1], buffer, buffer_size + 1);

    return _i2c_bus->write(_address, new_buffer, buffer_size + 1, poll) == 0;
  }
#else
  virtual int writeRegister(uint8_t address, uint8_t* buffer,
                            uint8_t buffer_size, bool poll = false)
  {
    uint8_t new_buffer[buffer_size + 1];
    new_buffer[0] = address;
    memcpy(&new_buffer[1], buffer, buffer_size + 1);

    return writeBytes(new_buffer, buffer_size + 1);
  }
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  /**
   * @brief Writes bytes to the i2c device address
   *
   * @param buffer
   * @param buffer_size
   * @param poll
   * @return write Status
   * @return true if ACK
   * @return false if NACK
   */
  virtual int writeBytes(char* buffer, int buffer_size, bool poll = false)
  {
    return _i2c_bus->write(_address, buffer, buffer_size, poll) == 0;
  }
#else
  virtual int writeBytes(uint8_t* buffer, uint8_t buffer_size,
                         bool poll = false)
  {
    _i2c_bus->beginTransmission(_address);
    _i2c_bus->write(buffer, buffer_size);
    return _i2c_bus->endTransmission() == 0;
  }
#endif

  /** GETTERS */

  /**
   * @brief Get the Chip Id object
   *
   * @return uint8_t
   */
  uint8_t getChipId()
  {
    return _chip_id;
  }

  /** SETTERS */

  /**
   * @brief Set the Chip Id object
   *
   * @param chip_id
   */
  void setChipId(uint8_t chip_id)
  {
    _chip_id = chip_id;
  }

  /**
   * @brief Set the Device Address object
   *
   * @param address
   */
  void setDeviceAddress(int address)
  {
    _address = address;
  }

#ifdef PIO_FRAMEWORK_ARDUINO_PRESENT
  bool customPing()
  {
    _i2c_bus->beginTransmission(_address);
    int a = _i2c_bus->endTransmission() == 0;
    char str[20];
    sprintf(str, (a == 1) ? "Ping True" : "Ping False");
    Serial.println(str);
    return a;
  }
#endif
};

#endif  // I2C_DEVICE_H