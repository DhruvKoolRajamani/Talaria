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

#ifndef PWM_DEVICE_H
#define PWM_DEVICE_H

#include "device.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
class PwmDevice : public PwmOut
{
private:
  PinName _pwm_pin;
#else
class PwmDevice
{
private:
  int _pwm_pin;
#endif

public:
/** CONSTRUCTORS */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PwmDevice(PinName pwm_pin)
    : PwmOut(pwm_pin)
    , _pwm_pin(pwm_pin){
#else
  PwmDevice(int pwm_pin) : _pwm_pin(pwm_pin)
  {
    pinMode(_pwm_pin, OUTPUT);
#endif
}

/** DESTRUCTORS */

/**
 * @brief Destroy the PwmDevice object
 *
 */
virtual ~PwmDevice()
  {
  }

  /** METHODS */

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual void writePWMData(float dutyCycle = 50.0, float pulsePeriod = 0.1)
  {
    float pwmData = dutyCycle * 0.01;
    this->period_ms(pulsePeriod);
    this->write(pwmData);
  }

#else
  virtual bool writePWMData(float outVolt = 1)
  {
    int pwmData = map(outVolt * 1000, 0, 3300, 0, 255);

    analogWrite(_pwm_pin, pwmData);
  }
#endif
};

#endif  // ANALOG_DEVICE_H