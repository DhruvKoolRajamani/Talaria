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

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#include "devices/hardware.h"
#include "devices/base/device.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
class AnalogDevice : public AnalogIn, public Device
{
#else
class AnalogDevice : public Device
{

#endif
private:
  bool readReady = false;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PinName _pin_name;
#else
  uint8_t _id;
  int _pin_name;
#endif

public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  AnalogDevice(uint8_t id, PinName pin_name, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* frame_name = NULL, const char* topic_name = NULL,
               int refresh_rate = 1)
    : AnalogIn(pin_name)
    , Device(id, dev_index, nh, dev_name, frame_name, topic_name, refresh_rate)
    , _pin_name(pin_name)
  {
#else
  AnalogDevice(uint8_t id, int pin_name, ros::NodeHandle& nh,
               uint8_t dev_index = 0, const char* dev_name = NULL,
               const char* topic_name = NULL, int refresh_rate = 1)
    : Device(id, dev_index, nh, dev_name, topic_name, refresh_rate)
    , _pin_name(pin_name)
  {
    pinMode(_pin_name, INPUT);
#endif
    setIndex(dev_index);
    setHealthStatus(true);
    setEnabledStatus(true);
  }
#else
/**
 * @brief Construct a new AnalogDevice object
 *
 * @param id
 * @param pin_name
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  AnalogDevice(uint8_t id, PinName pin_name, uint8_t dev_index,
               int refresh_rate = 1)
    : AnalogIn(pin_name)
    , Device(id, dev_index, refresh_rate)
    , _pin_name(pin_name){
#else
  AnalogDevice(uint8_t id, int pin_name, uint8_t dev_index,
               int refresh_rate = 1)
    : Device(id, dev_index, refresh_rate), _pin_name(pin_name)
  {
    pinMode(_pin_name, INPUT);
#endif

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
  virtual ~AnalogDevice() {}

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
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual float readAnalogData(uint16_t delay_ms = 100)
  {
    float analogData;
    analogData = this->read();
    readReady = true;
    // wait_ms(delay_ms);

    return analogData;
  }
#else
virtual float readAnalogData(uint16_t delay_ms = 100)
{
  float temp, analogData;
  temp = analogRead(_pin_name);
  analogData = temp * 5.0 / 1023.0;
  readReady = true;
  // wait_ms(delay_ms);

  return analogData;
}
#endif
};

#endif  // ANALOG_DEVICE_H