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
#include "i2c/i2c_bus.h"
#include "mbed.h"

extern I2CBus *I2CBusses;

class I2CDevice : Device
{
private:
  I2CBus *i2cBus;
  uint8_t _chip_id = 0x00;
  int _address;

protected:
public:
  /** CONSTRUCTORS */

  /**
   * @brief Construct a new I2CDevice object
   *
   * @param address
   * @param bus_Id
   * @param dev_index
   */
  I2CDevice(int address, uint8_t bus_Id, uint8_t dev_index = 0)
      : Device(dev_index), _address(address << 1)
  {
    for (I2CBus *ptr = I2CBusses; ptr != nullptr; ptr++)
    {
      if (ptr->getId() == bus_Id)
      {
        i2cBus = ptr;
        break;
      }
    }
    setIndex(dev_index);
  }

  /** DESTRUCTOR */

  /**
   * @brief Destroy the I2CDevice object
   *
   */
  ~I2CDevice()
  {
  }

  /** METHODS */

  /**
   * @brief Ping the device to check if connection can be established
   *
   * @param chip_id_reg_address
   * @return true
   * @return false
   */
  virtual bool ping(uint8_t chip_id_reg_address = 0x00)
  {
    char buffer;
    if (readByteStream(chip_id_reg_address, &buffer, (int)sizeof(buffer)) == 0)
    {
      setHealthStatus(true);
      setConfiguredStatus(true);
      setEnabledStatus(true);
      _chip_id = (uint8_t)buffer;
      return true;
    }
    else
      return false;
  }

  /**
   * @brief Read bytestream from device
   *
   * @detail Reads the bytestream over the respective interface for each device.
   *
   * @param register address
   * @param buffer
   * @param buffer_size
   * @return true if read successful
   * @return false if write to register unsuccessfull
   */
  virtual bool readByteStream(uint8_t address, char *buffer, int buffer_size)
  {
    if (i2cBus->write(_address, (char *)address, 1) == 0)
    {
      return i2cBus->read(_address, buffer, buffer_size) == 0;
    }
    else
      return false;
  }

  /**
   * @brief Reset the pin for the Device and wait for a delay in ms before
   * restarting it
   *
   * @param pin
   * @param delay_ms
   */
  virtual void reset(PinName pin, const uint16_t delay_ms = 100)
  {
    DigitalOut resetPin(pin, false);
    wait_ms(delay_ms);
    resetPin = true;
  }

  virtual int writeByteStream(uint8_t address, char *buffer, int buffer_size)
  {
    char new_buffer[buffer_size + 1];
    new_buffer[0] = (char)address;
    memcpy(&new_buffer[1], buffer, buffer_size + 1);

    return i2cBus->write(_address, new_buffer, buffer_size + 1);
  }

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

  /**
   * @brief Get the Address object
   * 
   * @return int _address
   */
  int getAddress()
  {
    return _address;
  }
};

#endif // I2C_DEVICE_H