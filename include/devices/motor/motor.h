/**
 * @file strain_gauge.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-24
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#include "devices/hardware.h"
#include "devices/base/analog_device.h"
#include "devices/base/pwm_device.h"

#ifndef DISABLE_ROS
// Add header file for custom ros message for bend sensor
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#endif

class Motor : public AnalogDevice
{
private:
  float aRSense = 0.1;
  float torqueConst = 10.9;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  // pwm pins
  PinName _aEnable = p25;  // p136 -- p8 _aEnable
  PinName _vRef = p26;     // p137 -- p4 + jumper _vRef
  // analog pin
  PinName _aVSense = p19;  // p90 -- across R3 _aVSense

  // digital pins
  PinName _aPhase = p5;   // p101 -- p7 M0
  PinName _nSleep = p6;   // p94 -- p3 _nSleep
  PinName _nConfig = p7;  // p96 -- p5 + jumper _nConfig
  PinName _nFault = p8;   // p95 -- p7r _nFault
  float _measuredI, _desiredTorque, _error;

#else
  PwmDevice _aEnable = PwmDevice(3);  // p136 -- p8
  PwmDevice _vRef = PwmDevice(6);     // p137 -- p4 + jumper
  int _aVSense = 13;                  // p90 -- across R3 A10
  int _aPhase = 8;                    // p101 -- p7 br 44
  int _nSleep = 12;                   // p94 -- p3 or 51
  int _nConfig = 11;                  // p96 -- p5 + jumper bla 49
  int _nFault = 10;                   // p95 -- p7r gr 50
  float _measuredI, _desiredTorque, _error;
#endif

#ifndef DISABLE_ROS
  ros::Publisher _pub_motor;
  // Create a custom ros message for motor but for now using standard
  std_msgs::Float32 _msg_motor;
#else
  char str[100];
#endif

protected:
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
        PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
        ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
        const char* topic_name, int refresh_rate);
#else
  Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep, int nFault,
        int nConfig, int aPhase, ros::NodeHandle& nh, uint8_t dev_index,
        const char* dev_name, const char* topic_name, int refresh_rate);
#endif
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
        PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
        uint8_t dev_index, int refresh_rate);
#else
  Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep, int nFault,
        int nConfig, int aPhase, uint8_t dev_index, int refresh_rate);
#endif
#endif

  // DESTRUCTORS
  virtual ~Motor();

  // GETTERS

  // SETTERS

  // METHODS
  /**
   * @brief Initialize motor
   *
   * @return true
   * @return false
   */
  bool initialize() override;

  void update(int loop_counter = 1) override;

  /**
   * @brief Control motor speed
   *
   */
  void setPwm();

  /**
   * @brief Read current across sense resistor
   *
   * @return current
   */
  float getISense();

  /**
   * @brief Set the Torque object
   *
   * @param desired_torque
   */
  void setTorque(float desired_torque);

  /**
   * @brief Initialize motor
   *
   * @param _measuredI current measured across sense resistor
   * @param desiredTorque desired torque value
   * @return measured torque from current sense feedback
   *
   */
  float setVRef();
};

#endif  // MOTOR_H