#include "devices/motor/motor.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#ifdef CURRENT_SENSE
Motor::Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
             PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
             ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
             const char* frame_name, const char* meas_topic_name,
             const char* des_topic_name, int refresh_rate)
  : AnalogDevice(id, aVSense, nh, dev_index, dev_name, frame_name,
                 meas_topic_name, refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _sub_motor(des_topic_name, &Motor::motorDesiredCb, this)
  , _pub_motor(meas_topic_name, &(this->_msg_motor_measured))
{
#else
Motor::Motor(uint8_t id, PinName aVEnc, PinName aEnable, PinName vRef,
             PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
             ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
             const char* frame_name, const char* meas_topic_name,
             const char* des_topic_name, int refresh_rate)
  : AnalogDevice(id, aVEnc, nh, dev_index, dev_name, frame_name,
                 meas_topic_name, refresh_rate)
  , _aVEnc(aVEnc)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _sub_motor(des_topic_name, &Motor::motorDesiredCb, this)
  , _ref(new PwmOut(vRef))
{
#endif
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
  , _sub_motor(des_topic_name, &Motor::motorDesiredCb, this)
#ifdef CURRENT_SENSE
  , _pub_motor(meas_topic_name, &(this->_msg_motor_measured))
#endif
{
  pinMode(_aVSense, INPUT);
  pinMode(_aPhase, OUTPUT);
  pinMode(_nSleep, OUTPUT);
  pinMode(_nConfig, OUTPUT);
  pinMode(_nFault, INPUT);
  // analogWriteResolution(12);
  pinMode(13, OUTPUT);
#endif
#ifdef CURRENT_SENSE
  setIsTopicAdvertised(nh.advertise(_pub_motor));
#endif
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
  // if (_ref) free(_ref);
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
  _ref->period_ms(1.0);
  // while (true)
  // {
  //   _desiredDir = 1;
  //   setDir();
  //   _ref->write(0.5f);
  //   wait_ms(200);
  //   break;
  // }
  // fault.mode(PullUp);

#ifdef DISABLE_ROS
  sprintf(str, (_nFault) ? "fault : True\n" : "fault : False\n");
  print(str);
#endif

  if (true)
  {
    // setPwm();
    // 5.2 for test, 6.3 for main
    // setTorque(5);
#ifndef DISABLE_ROS
#ifdef CURRENT_SENSE
    _msg_motor_measured.header.frame_id = this->getFrameName();
    _msg_motor_measured.header.stamp = this->getNodeHandle()->now();
#endif
#endif
    this->setEnabledStatus(true);
    this->setConfiguredStatus(true);
    this->setHealthStatus(true);
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
void Motor::setDir() { DigitalOut phase(_aPhase, _desiredDir); }
#ifdef CURRENT_SENSE
float Motor::getISense()
{
  AnalogIn input(_aVSense);
  float iSense = input;  // this->readAnalogData();
  if (_desiredTorque > 0)
  {
    return iSense;
  }
  else
  {
    return -1 * iSense;
  }
  // return (iSense);//*(_desiredTorque > 0) ? 1 : -1);  // *10
}

void Motor::setTorque(float desired_torque, float torque_constant)
{
  _torqueConst = torque_constant;
  _desiredTorque = desired_torque;
  _desiredDir = (desired_torque > 0) ? 1 : 0;
  setDir();
}

float Motor::setVRef(float desiredTorque)
{
  float new_vRef = (abs(desiredTorque) * 0.5 * 80 / 6) / _torqueConst;
  PwmOut vrefOut(_vRef);
  vrefOut.write(new_vRef);
  // _ref->writePWMData(new_vRef);
}
#else
float Motor::getPosition()
{
  AnalogIn input(_aVEnc);
  float encValue = input;
  return encValue;
}

void Motor::setPosition(float desired_pos)
{
  // float cur_pos = getPosition();
  // float delta = desired_pos - cur_pos;
  // do
  // {
  //   _desiredDir = (delta > 0) ? 1 : 0;
  //   setDir();
  //   // Write pwm here!!!
  //   // _ref->write(abs(delta));
  //   cur_pos = getPosition();
  // } while (abs(delta) <= 0.01);
  if (desired_pos > 550)
  {
    // this->getNodeHandle()->loginfo("Enter grasp posn");

    _desiredDir = 1;
    setDir();
    _ref->write(0.1f);
  }
  else if (desired_pos < 450)
  {
    _desiredDir = 0;
    setDir();
    _ref->write(0.1f);
  }
  else
  {
    _desiredDir = 0;
    setDir();
    _ref->write(0.0f);
  }
}
#endif
#else
float Motor::getISense()
{
  int val = analogRead(_aVSense);
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
}

float Motor::setVRef(float desiredTorque)
{
  float new_vRef = desiredTorque * 0.5 / _torqueConst;

  _vRef.writePWMData(new_vRef);
}
#endif

void Motor::update()
{
  // Only update if update rate for the sensor is the same as the sampling
  // rate
  if (this->getEnabledStatus())
  {
#ifdef DISABLE_ROS
    char cstr[100];
    print(cstr);
#endif
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    uint64_t current_time = get_ms_count();
#else
    unsigned long current_time = millis();
#endif
    if (first_update ||
        (double)(current_time - _prev_update_time) >= _refresh_rate &&
            this->getConfiguredStatus())
    {
      first_update = false;
      _prev_update_time = current_time;
      // Publish Diagnostic messages
      Device::update();

#ifdef DISABLE_ROS
      sprintf(cstr, "Measured torque = %f\n", _error);
      print(cstr);
      sprintf(cstr, "Measured torque = %f\n", _error);
      print(cstr);
#else
#ifdef CURRENT_SENSE
      // setTorque(1);  // Remove once subscriber works
      _error = (_measuredI)*_torqueConst;

      motor_msg::motor_measured temp;
      temp.header.frame_id = this->getDeviceName();
      temp.header.stamp = this->getNodeHandle()->now();
      temp.motor_id.data = 0;
      temp.desired_force.data = _desiredTorque;
      temp.measured_force.data = _error;

      _msg_motor_measured = temp;
      if (this->getIsTopicAdvertised()) _pub_motor.publish(&temp);
#endif
#endif
    }
  }
}

#ifndef DISABLE_ROS
#ifdef CURRENT_SENSE
void Motor::motorDesiredCb(const motor_msg::motor_desired& msg)
#else
void Motor::motorDesiredCb(const motor_msg::cmd_light& msg)
#endif
{
// _measuredI = 0.5;
#ifdef CURRENT_SENSE
  setTorque(msg.desired_force.data, msg.torque_constant.data);
  setVRef(msg.desired_force.data);
  _measuredI = getISense();
#else
  _desiredPos = msg.cmd / 1.;
  setPosition(_desiredPos);
#endif
}
#endif