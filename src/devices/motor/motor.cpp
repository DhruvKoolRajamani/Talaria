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
             const char* topic_name, int refresh_rate)
  : AnalogDevice(id, aVSense, nh, dev_index, dev_name, topic_name, refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _pub_motor(topic_name, &(this->_msg_motor))
{
#else
Motor::Motor(uint8_t id, int aVSense, int aEnable, int vRef, int nSleep,
             int nFault, int nConfig, int aPhase, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name,
             int refresh_rate)
  : AnalogDevice(id, aVSense, nh, dev_index, dev_name, topic_name, refresh_rate)
  , _aVSense(aVSense)
  , _aEnable(aEnable)
  , _vRef(vRef)
  , _nSleep(nSleep)
  , _nFault(nFault)
  , _nConfig(nConfig)
  , _aPhase(aPhase)
  , _pub_motor(topic_name, &(this->_msg_motor))
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
  // ;  // enable output to motor
  DigitalIn fault(_nFault);

#ifdef DISABLE_ROS
  sprintf(str, (_nFault) ? "fault : True\n" : "fault : False\n");
  print(str);
#endif

  if (fault)
  {
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
    setPwm();
    // 5.2 for test, 6.3 for main
    setTorque(5);
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
  float speed = 50;  // get desired speed from usb
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
  int val = this->readAnalogData();
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
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

void Motor::setTorque(float desired_torque)
{
  _desiredTorque = desired_torque;
}

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float Motor::setVRef()
{
  float measuredTorque = (_measuredI)*torqueConst;
  float new_vRef = (_desiredTorque * 0.5) / torqueConst;
  PwmDevice ref(_vRef);
  ref.writePWMData(new_vRef);
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
  char cstr[100];
#ifdef DISABLE_ROS
  // sprintf(cstr, "loop_counter: %d\n", loop_counter);
  // print(cstr);
#endif
  if (this->getRefreshRate() == loop_counter)
  {
    // Publish Diagnostic messages
    Device::update(loop_counter);

    _measuredI = getISense();
    setTorque(500);
    _error = setVRef();
#ifdef DISABLE_ROS
    // sprintf(cstr, "Measured current = %f\nMeasured torque = %f\n",
    // _measuredI,
    //         _error);
    // print(cstr);
    // sprintf(cstr, "Measured current = %f\nMeasured torque = %f\n",
    // _measuredI,
    //         _error);
    // print(cstr);
#endif
  }
}