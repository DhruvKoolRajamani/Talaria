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

#define aRSense 0.1
#define torqueConst 10.9

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PwmDevice aEnable(p25);     // p136 -- p8 aEnable
  PwmDevice vRef(p26);        // p137 -- p4 + jumper VRef
  PinName aVSense = p19;  // p90 -- across R3 aVSense
  DigitalOut aPhase(p5);            // p101 -- p7 M0
  DigitalOut nSleep(p6);            // p94 -- p3 nSleep
  DigitalOut nConfig(p7);           // p96 -- p5 + jumper nConfig
  DigitalIn nFault(p8);             // p95 -- p7r nFault
  float measuredI, desiredTorque, error;
  char str[100];
  
#else
  PwmDevice aEnable(3);     // p136 -- p8
  PwmDevice vRef(6);        // p137 -- p4 + jumper
  int aVSense = 13;         // p90 -- across R3 A10
  int aPhase = 8;           // p101 -- p7 br 44
  int nSleep = 12;          // p94 -- p3 or 51
  int nConfig = 11;         // p96 -- p5 + jumper bla 49
  int nFault = 10;          // p95 -- p7r gr 50
  float measuredI, desiredTorque, error;
  char str[100];
  
#endif
  ros::Publisher _pub_motor;
  // Create a custom ros message for motor but for now using standard
  std_msgs::Float32 _msg_motor; 
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PwmDevice aEnable(1, p25, 1);     // p136 -- p8 aEnable
  PwmDevice vRef(1, p26, 1);        // p137 -- p4 + jumper VRef
  PinName aVSense = p19;  // p90 -- across R3 aVSense
  DigitalOut aPhase(p5);            // p101 -- p7 M0
  DigitalOut nSleep(p6);            // p94 -- p3 nSleep
  DigitalOut nConfig(p7);           // p96 -- p5 + jumper nConfig
  DigitalIn nFault(p8);             // p95 -- p7r nFault
  float measuredI, desiredTorque, error;
  char str[100];
#else
  PwmDevice aEnable(3);     // p136 -- p8
  PwmDevice vRef(6);        // p137 -- p4 + jumper
  int aVSense = 13;         // p90 -- across R3 A10
  int aPhase = 8;           // p101 -- p7 br 44
  int nSleep = 12;          // p94 -- p3 or 51
  int nConfig = 11;         // p96 -- p5 + jumper bla 49
  int nFault = 10;          // p95 -- p7r gr 50
  float measuredI, desiredTorque, error;
  char str[100];
#endif
#endif  

protected:
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  Motor(uint8_t id, PinName aVSense, ros::NodeHandle& nh, uint8_t dev_index,
              const char* dev_name, const char* topic_name, int refresh_rate=1)
    : AnalogDevice(id, aVSense, nh, dev_index, dev_name, topic_name, refresh_rate)
    , _pub_motor(topic_name, &(this->_msg_motor))
  {
#else
  Motor(uint8_t id, int aVSense, ros::NodeHandle& nh, uint8_t dev_index,
              const char* dev_name, const char* topic_name, int refresh_rate=1)
    :AnalogDevice(id, aVSense, nh, dev_index, dev_name, topic_name, refresh_rate)
    , _pub_motor(topic_name, &(this->_msg_motor))
  {
#endif

    setIsTopicAdvertised(nh.advertise(_pub_motor));
  }
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  Motor(uint8_t id, PinName aVSense, uint8_t dev_index, int refresh_rate=1)
  : AnalogDevice(id, aVSense, dev_index, refresh_rate)
  {
#else
  Motor(uint8_t id, int aVSense, uint8_t dev_index, int refresh_rate=1)
  : AnalogDevice(id, aVSense, dev_index, refresh_rate)
  {
#endif
  }
#endif

  // DESTRUCTORS
  virtual ~Motor()
  {
  }

  // GETTERS
  
  // SETTERS

  // METHODS
  /**
   * @brief Initialize motor
   *
   * @return true
   * @return false
   */
  bool initialize();
  
  void update(int loop_counter = 1);

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
   * @brief Get desired torque value
   *
   * @return torque
   */
  float getTorque();

  /**
   * @brief Initialize motor
   *
   * @param measuredI current measured across sense resistor
   * @param desiredTorque desired torque value
   * @return measured torque from current sense feedback
   * 
   */
  float setVRef(float measuredI, float desiredTorque);

};

#endif  // MOTOR_H