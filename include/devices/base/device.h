/**
 * @file device.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-01
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef DEVICE_H
#define DEVICE_H

#include "mbed.h"

class Device
{
private:
  bool _en_status;
  bool _conf_status;
  bool _health_status;

  static const int DEVICE_ID_SIZE = 8;

  uint8_t* _dev_Id;
  uint8_t _dev_index;

protected:
public:
  /** CONSTRUCTORS */

  /**
   * @brief Construct a new Device object
   *
   */
  Device()
    : _en_status(true)
    , _conf_status(false)
    , _health_status(false)
    , _dev_index(0)
  {
    _dev_Id = static_cast<uint8_t*>(malloc(DEVICE_ID_SIZE));
  }

  /**
   * @brief Construct a new Device object
   *
   * @param uint8_t dev_index
   */
  Device(uint8_t dev_index)
    : _en_status(true)
    , _conf_status(false)
    , _health_status(false)
    , _dev_index(dev_index)
  {
    _dev_Id = static_cast<uint8_t*>(malloc(DEVICE_ID_SIZE));
  }

  /** DESTRUCTOR */

  /**
   * @brief Destroy the Device object
   *
   */
  ~Device()
  {
  }

  /** GETTERS */

  /**
   * @brief Get the Id Size object
   *
   * @return int DEVICE_ID_SIZE
   */
  int getIdSize()
  {
    return DEVICE_ID_SIZE;
  }

  /**
   * @brief Get the Id object
   *
   * @return uint8_t* _dev_Id
   */
  uint8_t* getId()
  {
    return _dev_Id;
  }

  /**
   * @brief Get the Index object
   *
   * @return uint8_t _dev_index
   */
  uint8_t getIndex()
  {
    return _dev_index;
  }

  /**
   * @brief Get the Health Status object
   *
   * @return true _health_status
   * @return false _health_status
   */
  bool getHealthStatus()
  {
    return _health_status;
  }

  /**
   * @brief Get the Configured Status object
   *
   * @return true _conf_status
   * @return false _conf_status
   */
  bool getConfiguredStatus()
  {
    return _conf_status;
  }

  /**
   * @brief Get the Enabled Status object
   *
   * @return true _en_status
   * @return false _en_status
   */
  bool getEnabledStatus()
  {
    return _en_status;
  }

  /** METHODS */

  /**
   * @brief Enable the Device
   *
   */
  virtual void enable()
  {
    _en_status = true;
  }

  /**
   * @brief Disable the Device
   *
   */
  virtual void disable()
  {
    _en_status = false;
  }

  /**
   * @brief Read bytestream from device
   *
   * @detail Reads the bytestream over the respective interface for each device.
   *
   * @param uint8_t address
   * @param uint8_t* buffer
   * @param int buffer_size
   * @return true
   * @return false
   */
  virtual bool readByteStream(uint8_t address, uint8_t* buffer, int buffer_size)
  {
    return false;
  }

  /**
   * @brief Write data through stream
   *
   * @param uint8_t address
   * @param uint8_t* buffer
   * @param int buffer_size
   */
  virtual void writeByteStream(uint8_t address, uint8_t* buffer,
                               int buffer_size)
  {
  }

  /**
   * @brief Reset the pin for the Device and wait for a delay in ms before
   * restarting it
   *
   * @param PinName pin
   * @param int delay_ms
   */
  virtual void reset(PinName pin, int delay_ms = 100)
  {
    DigitalOut resetPin(pin, false);
    wait_ms(delay_ms);
    resetPin = true;
  }

  /**
   * @brief Set the Pin State object
   *
   * @param PinName pin
   * @param bool state
   */
  virtual void setPinState(PinName pin, bool state)
  {
    int value = (int)state;
    DigitalOut Pin(pin, value);
  }

  /** SETTERS */

  /**
   * @brief Set the Index object
   *
   * @param uint8_t index
   */
  void setIndex(uint8_t index)
  {
    _dev_index = index;
  }

  /**
   * @brief Set the Id object
   *
   * @param uint8_t id
   */
  void setId(uint8_t id)
  {
    *_dev_Id = id;
  }

  /**
   * @brief Set the Health Status object
   *
   * @param state
   */
  void setHealthStatus(bool state)
  {
    _health_status = state;
  }

  /**
   * @brief Set the Enabled Status object
   *
   * @param state
   */
  void setEnabledStatus(bool state)
  {
    _en_status = state;
  }

  /**
   * @brief Set the Configured Status object
   *
   * @param state
   */
  void setConfiguredStatus(bool state)
  {
    _conf_status = state;
  }
};

#endif  // DEVICE_H