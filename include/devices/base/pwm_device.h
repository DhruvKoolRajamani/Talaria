/**
 * @file analog_device.h
 * @author Raunak Hosangadi (rphosangadi@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-05
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
#include "mbed.h"

#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
#include "Arduino.h"
#endif

#ifndef PWM_DEVICE_H
#define PWM_DEVICE_H

#include "device.h"


#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
class PwmDevice : public PwmOut, public Device
#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
class PwmDevice : public Device
#endif
{

#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
private:
  bool _init_status;
  bool writeReady = false;

  uint8_t _id;
  PinName _a0;

#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
private:
  bool _init_status;
  bool writeReady = false;

  uint8_t _id;
  int _a0;
  pinMode(_a0, OUTPUT);
#endif

public:
  /** CONSTRUCTORS */
  #ifndef DISABLE_ROS
  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  PwmDevice(uint8_t id, PinName a0, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* topic_name = NULL)
    : PwmOut(a0)
    , Device(dev_index, nh, dev_name, topic_name)
    , _id(id)
    , _a0(a0)
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  PwmDevice(uint8_t id, int a0, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* topic_name = NULL)
    : Device(dev_index, nh, dev_name, topic_name)
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
   * @brief Construct a new PwmDevice object
   *
   * @param id
   * @param a0
   */

  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  PwmDevice(uint8_t id, PinName a0, uint8_t dev_index = 0)
    : PwmOut(a0), Device(dev_index), _id(id), _a0(a0)
  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  PwmDevice(uint8_t id, int a0, uint8_t dev_index = 0)
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
   * @brief Destroy the PwmDevice object
   *
   */
  virtual ~PwmDevice()
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
  // virtual bool ping(uint8_t chip_id_reg_address = 0x00)
  // {
  //   if (writeReady == true)
  //   {
  //     setHealthStatus(true);
  //     setConfiguredStatus(true);
  //     setEnabledStatus(true);
  //     writeReady = false;
  //     return true;
  //   }
  //   else
  //     return false;
  // }
  // }
  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  virtual float writePWMData(float dutyCycle = 50.0, float pulsePeriod = 0.1)
  {
    float pwmData = dutyCycle*0.01;
    this->period_ms(pulsePeriod);
    this->write(pwmData);
    writeReady = true;
    // wait_ms(delay_ms);

    return writeReady;
  }

  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual float writePWMData(float outVolt = 1)
  { 
    int pwmData = map(outVolt*1000,0,3300,0,255);
    
    analogWrite(a0,pwmData);
    writeReady = true;
    // wait_ms(delay_ms);

    return writeReady;
  }
  #endif
};

#endif  // ANALOG_DEVICE_H