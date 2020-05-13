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
// #include "motor_msg/motor_desired.h"
// #include "motor_msg/motor_measured.h"
#include "motor_msg/cmd_light.h"
#include "std_msgs/String.h"
#endif

class Motor : public AnalogDevice
{
private:
  float aRSense = 1;
  float _torqueConst;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  // pwm pins
  PinName _aEnable;  // p136 -- p8 _aEnable
  PinName _vRef;     // p137 -- p4 + jumper _vRef
// analog pin
#ifdef CURRENT_SENSE
  PinName _aVSense;  // p90 -- across R3 _aVSense
  float _measuredI, _desiredTorque, _error;
#else
  PinName _aVEnc;
  float _measuredPos, _desiredPos;
#endif
  PwmOut* _ref = nullptr;

  // digital pins
  PinName _aPhase;   // p101 -- p7 M0
  PinName _nSleep;   // p94 -- p3 _nSleep
  PinName _nConfig;  // p96 -- p5 + jumper _nConfig
  PinName _nFault;   // p95 -- p7r _nFault

  bool _desiredDir;

#else
  PwmDevice _aEnable = PwmDevice(3);  // p136 -- p8
  PwmDevice _vRef = PwmDevice(6);     // p137 -- p4 + jumper
  int _aVSense = 13;                  // p90 -- across R3 A10
  int _aPhase = 8;                    // p101 -- p7 br 44
  int _nSleep = 12;                   // p94 -- p3 or 51
  int _nConfig = 11;                  // p96 -- p5 + jumper bla 49
  int _nFault = 10;                   // p95 -- p7r gr 50
  float _measuredI, _desiredTorque, _error;
  bool _desiredDir;
#endif

#ifndef DISABLE_ROS
#ifdef CURRENT_SENSE
  ros::Publisher _pub_motor;
  ros::Subscriber<motor_msg::motor_desired, Motor> _sub_motor;
  motor_msg::motor_measured _msg_motor_measured;
#else
  ros::Subscriber<motor_msg::cmd_light, Motor> _sub_motor;
#endif
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
        const char* frame_name, const char* meas_topic_name = NULL,
        const char* des_topic_name = NULL, int refresh_rate = 1);
#else
  Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep, int nFault,
        int nConfig, int aPhase, ros::NodeHandle& nh, uint8_t dev_index,
        const char* dev_name, const char* meas_topic_name,
        const char* des_topic_name, int refresh_rate);
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

  void update() override;

#ifdef CURRENT_SENSE
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
  void setTorque(float desired_torque, float torque_constant = 0.0f);

  /**
   * @brief Initialize motor
   *
   * @param _measuredI current measured across sense resistor
   * @param desiredTorque desired torque value
   *
   * @return measured torque from current sense feedback
   *
   */
  float setVRef(float desired_torque);
#else

  /**
   * @brief Get the Position object
   *
   * @return float
   */
  float getPosition();

  /**
   * @brief Set the Position object
   *
   * @param desired_pos
   */
  void setPosition(float desired_pos);
#endif

  /**
   * @brief Set the Dir object
   *
   */
  void setDir();

#ifndef DISABLE_ROS
  /**
   * @brief Callback for subscriber to desired (cmd) motor value
   *
   * @param msg_motor_desired
   */
#ifdef CURRENT_SENSE
  void motorDesiredCb(const motor_msg::motor_desired& msg_motor_desired);
#else
  void motorDesiredCb(const motor_msg::cmd_light& msg_motor_desired);
#endif
#endif
};

#endif  // MOTOR_H