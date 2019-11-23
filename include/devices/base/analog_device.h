/**
 * @file analog_device.h
 * @author Hao Yang (hyang6@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-10
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef ANALOG_DEVICE_H
#define ANALOG_DEVICE_H

#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
#include "mbed.h"

#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
#include "Arduino.h"
#endif

#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
class AnalogDevice : public AnalogIn, public Device
#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
class AnalogDevice : public Device
#endif
{
private:
  bool readReady = false;

  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  uint8_t _id;
  PinName _a0;
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  uint8_t _id;
  int _a0;
  pinMode(_a0,INPUT);
  #endif

public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  AnalogDevice(uint8_t id, PinName a0, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* topic_name = NULL, int refresh_rate = 1)
    : AnalogIn(a0)
    , Device(dev_index, nh, dev_name, topic_name, refresh_rate)
    , _id(id)
    , _a0(a0)
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  AnalogDevice(uint8_t id, int a0, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* topic_name = NULL, int refresh_rate = 1)
    : Device(dev_index, nh, dev_name, topic_name, refresh_rate)
    , _id(id)
    , _a0(a0)
  #endif
  {
    setIndex(dev_index);
    setHealthStatus(true);
    setEnabledStatus(true);
  }
#else
  /**
   * @brief Construct a new AnalogDevice object
   *
   * @param id
   * @param a0
   */
  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  AnalogDevice(uint8_t id, PinName a0, uint8_t dev_index = 0
                int refresh_rate = 1 )
    : AnalogIn(a0), Device(dev_index), _id(id), _a0(a0)
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  AnalogDevice(uint8_t id, int a0, uint8_t dev_index = 0
                int refresh_rate = 1 )
    : Device(dev_index), _id(id), _a0(a0)
  #endif
  {
    setIndex(dev_index);
    setHealthStatus(true);
    setEnabledStatus(true);
  }
#endif

  /** DESTRUCTORS */

  /**
   * @brief Destroy the AnalogDevice object
   *
   */
  virtual ~AnalogDevice()
  {
  }

  /** METHODS */

  // /**
  //  * @brief Ping the device to check if connection can be established
  //  *
  //  * @param chip_id_reg_address
  //  * @return true
  //  * @return false
  //  */
  // virtual bool ping(uint8_t chip_id_reg_address = 0x00)
  // {
  //   if (readReady == true)
  //   {
  //     setHealthStatus(true);
  //     setConfiguredStatus(true);
  //     setEnabledStatus(true);
  //     readReady = false;
  //     return true;
  //   }
  //   else
  //     return false;
  // }
  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  virtual float readAnalogData(uint16_t delay_ms = 100)
  {
    float analogData;
    analogData = this->read();
    readReady = true;
    // wait_ms(delay_ms);

    return analogData;
  }
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual float readAnalogData(uint16_t delay_ms = 100)
  {
    float temp,analogData;
    temp = analogRead(a0);
    analogData = temp*5.0/1023.0;
    readReady = true;
    // wait_ms(delay_ms);

    return analogData;
  }
  #endif
};

#endif  // ANALOG_DEVICE_H