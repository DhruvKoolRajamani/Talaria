/**
 * @file i2c_bus.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-01
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "mbed.h"

class I2CBus : public I2C
{
private:
  bool _init_status;

  uint8_t _id;

  PinName _sda;
  PinName _scl;

  int _clock_speed;

public:
  /** CONSTRUCTORS */

  /**
   * @brief Construct a new I2CBus object
   *
   * @param id
   * @param scl
   * @param sda
   */
  I2CBus(uint8_t id, PinName sda, PinName scl, int clock_speed = 400000)
    : I2C(sda, scl), _id(id), _sda(sda), _scl(scl), _clock_speed(clock_speed)
  {
    frequency(_clock_speed);
    _init_status = true;
  }

  /** DESTRUCTORS */

  /**
   * @brief Destroy the I2CBus object
   *
   */
  virtual ~I2CBus()
  {
  }

  /** GETTERS */

  /**
   * @brief Get the Id object
   *
   * @return uint8_t
   */
  uint8_t getId()
  {
    return _id;
  }

  /**
   * @brief Get the Init Status object
   *
   * @return uint8_t
   */
  uint8_t getInitStatus()
  {
    return _init_status;
  }

  /**
   * @brief Get the Clock Speed object
   *
   * @return uint8_t
   */
  int getClockSpeed()
  {
    return _clock_speed;
  }

  /** SETTERS */

  /**
   * @brief Set the Init Status object
   *
   * @param state
   */
  void setInitStatus(bool state)
  {
    _init_status = state;
  }

  /**
   * @brief Set the Id object
   *
   * @param id
   */
  void setId(uint8_t id)
  {
    _id = id;
  }

  /**
   * @brief Set the Clock Speed object
   *
   * @param clock_speed
   */
  void setClockSpeed(int clock_speed)
  {
    _clock_speed = clock_speed;
    frequency(_clock_speed);
    _init_status = true;
  }
};

#endif  // I2C_BUS_H