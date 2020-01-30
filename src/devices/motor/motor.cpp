#include "devices/motor/motor.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
Motor::Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
             PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
             ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
             const char* meas_topic_name, const char* des_topic_name,
             int refresh_rate)
  : AnalogDevice(id, aVSense, nh, dev_index, dev_name, meas_topic_name,
                 refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _pub_motor(meas_topic_name, &(this->_msg_motor_measured))
  , _sub_motor(des_topic_name, &Motor::motorDesiredCb, this)
  , _ref(new PwmDevice(vRef))
{
#else
Motor::Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep,
             int nFault, int nConfig, int aPhase, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name,
             const char* meas_topic_name, const char* des_topic_name,
             int refresh_rate)
  : AnalogDevice(id, aVSense, nh, dev_index, dev_name, meas_topic_name,
                 refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _pub_motor(meas_topic_name, &(this->_msg_motor_measured))
  , _sub_motor(des_topic_name, &Motor::motorDesiredCb, this)
{
  pinMode(_aVSense, INPUT);
  pinMode(_aPhase, OUTPUT);
  pinMode(_nSleep, OUTPUT);
  pinMode(_nConfig, OUTPUT);
  pinMode(_nFault, INPUT);
  // analogWriteResolution(12);
  pinMode(13, OUTPUT);
#endif
  setIsTopicAdvertised(nh.advertise(_pub_motor));
  this->getNodeHandle()->subscribe(_sub_motor);
}
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
Motor::Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
             PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
             uint8_t dev_index, int refresh_rate)
  : AnalogDevice(id, aVSense, dev_index, refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
{

#else
Motor::Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep,
             int nFault, int nConfig, int aPhase, uint8_t dev_index,
             int refresh_rate)
  : AnalogDevice(id, aVSense, dev_index, refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
{
  pinMode(_aVSense, INPUT);
  pinMode(_aPhase, OUTPUT);
  pinMode(_nSleep, OUTPUT);
  pinMode(_nConfig, OUTPUT);
  pinMode(_nFault, INPUT);
  // analogWriteResolution(12);
  pinMode(13, OUTPUT);
#endif
}
#endif

// DESTRUCTORS
Motor::~Motor()
{
}

// GETTERS

// SETTERS

// METHODS

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
bool Motor::initialize()
{
#ifdef DISABLE_ROS
  sprintf(str, "Entered Motor Init\n");
  print(str);
#endif
  DigitalOut sleep(_nSleep, true);     // enable driver
  DigitalOut config(_nConfig, false);  // enable phase mode (DC motor)
  DigitalOut phase(_aPhase, true);
  DigitalIn fault(_nFault);
  // fault.mode(PullUp);

#ifdef DISABLE_ROS
  sprintf(str, (_nFault) ? "fault : True\n" : "fault : False\n");
  print(str);
#endif

  if (true)
  {
    setPwm();
    // 5.2 for test, 6.3 for main
    // setTorque(5);
#ifndef DISABLE_ROS
    _msg_motor_measured.header.frame_id = "index";
    _msg_motor_measured.header.stamp = this->getNodeHandle()->now();
#endif
    return true;
  }
  else
  {
    return false;
  }
}

#else
bool Motor::initialize()
{
  digitalWrite(_nSleep, HIGH);  // enable driver
  digitalWrite(_nConfig, LOW);  // enable phase mode (DC motor)
  digitalWrite(_aPhase, HIGH);  // enable output to motor

  // motor error status
  if (digitalRead(_nFault))
  {
    // setPwm();
    // 5.2 for test, 6.3 for main
    // setTorque(1);
#ifndef DISABLE_ROS
    // _msg_motor_measured.header.stamp = this->getNodeHandle()->now();
#endif
    return true;
  }
  else
  {
    return false;
  }
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
void Motor::setPwm()
{
  float speed = 1;  // get desired speed from usb
  PwmDevice enable(_aEnable);
  enable.writePWMData(speed);
}

#else
void Motor::setPwm()
{
  float speed = 1.00;  // get desired speed from usb
  _aEnable.writePWMData(speed);
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float Motor::getISense()
{
  float iSense = this->readAnalogData();
  return (iSense)-0.5;  // *10
}

#else
float Motor::getISense()
{
  int val = analogRead(_aVSense);
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
}
#endif

void Motor::setTorque(float desired_torque, float torque_constant)
{
  torqueConst = torque_constant;
  _desiredTorque = desired_torque;
  _desiredDir = (desired_torque > 0) ? 1 : 0;
  setDir();
}

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
void Motor::setDir()
{
  DigitalOut phase(_aPhase, _desiredDir);
}

float Motor::setVRef()
{
  float measuredTorque = (_measuredI * 6 / 5) * torqueConst;
  float t_error = abs(_desiredTorque) - measuredTorque;
  float new_vRef =
      (abs(_desiredTorque) * 0.5 * 80 / 6) / torqueConst;  //+ t_error;
  _ref->writePWMData(new_vRef);
  return measuredTorque;
}

#else
float Motor::setVRef()
{
  float measuredTorque = (_measuredI)*torqueConst;
  float new_vRef = _desiredTorque * 0.5 / torqueConst;

  _vRef.writePWMData(new_vRef);
  return measuredTorque;
}
#endif

void Motor::update(int loop_counter)
{
  // Only update if update rate for the sensor is the same as the sampling
  // rate

#ifdef DISABLE_ROS
  char cstr[100];
  // sprintf(cstr, "loop_counter: %d\n", loop_counter);
  print(cstr);
#endif
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  uint64_t current_time = get_ms_count();
#else
  unsigned long current_time = millis();
#endif
  if (first_update || (current_time - _prev_update_time) >= _refresh_rate)
  {
    first_update = false;
    _prev_update_time = current_time;
    // Publish Diagnostic messages
    Device::update(loop_counter);
    // setPwm();
    _measuredI = getISense();
    // setTorque(1);  // Remove once subscriber works
    _error = setVRef();
#ifdef DISABLE_ROS
    sprintf(cstr, "Measured torque = %f\n", _error);
    print(cstr);
    sprintf(cstr, "Measured torque = %f\n", _error);
    print(cstr);
#else
    motor_msg::motor_measured temp = motor_msg::motor_measured();
    temp.header.frame_id = "index";
    temp.header.stamp = this->getNodeHandle()->now();
    temp.motor_id.data = 0;
    temp.desired_force.data = _desiredTorque;
    temp.measured_force.data = _error;

    // _msg_motor_measured = temp;

    _pub_motor.publish(&temp);
#endif
  }
}

#ifndef DISABLE_ROS
void Motor::motorDesiredCb(const motor_msg::motor_desired& msg)
{
  _measuredI = getISense();
  // _measuredI = 0.5;
  setTorque(msg.desired_force.data, msg.torque_constant.data);
  _error = setVRef();
}
#endif