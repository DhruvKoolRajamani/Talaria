#include "devices/motor/motor.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
Motor::Motor(uint8_t id, PinName pin_name, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name,
             int refresh_rate)
  : AnalogDevice(id, pin_name, nh, dev_index, dev_name, topic_name,
                 refresh_rate)
  , _pub_motor(topic_name, &(this->_msg_motor))
{
#else
Motor::Motor(uint8_t id, int pin_name, ros::NodeHandle& nh, uint8_t dev_index,
             const char* dev_name, const char* topic_name, int refresh_rate)
  : AnalogDevice(id, pin_name, nh, dev_index, dev_name, topic_name,
                 refresh_rate)
  , _pub_motor(topic_name, &(this->_msg_motor))
{
  pinMode(aVSense, INPUT);
  pinMode(aPhase, OUTPUT);
  pinMode(nSleep, OUTPUT);
  pinMode(nConfig, OUTPUT);
  pinMode(nFault, INPUT);
  // analogWriteResolution(12);
  pinMode(13, OUTPUT);
#endif

  setIsTopicAdvertised(nh.advertise(_pub_motor));
}
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
Motor::Motor(uint8_t id, PinName pin_name, uint8_t dev_index, int refresh_rate)
  : AnalogDevice(id, pin_name, dev_index, refresh_rate)
{
#else
Motor::Motor(uint8_t id, int pin_name, uint8_t dev_index, int refresh_rate)
  : AnalogDevice(id, pin_name, dev_index, refresh_rate)
{
  pinMode(aVSense, INPUT);
  pinMode(aPhase, OUTPUT);
  pinMode(nSleep, OUTPUT);
  pinMode(nConfig, OUTPUT);
  pinMode(nFault, INPUT);
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
  nSleep = 1;   // enable driver
  nConfig = 0;  // enable phase mode (DC motor)
  aPhase = 1;   // enable output to motor

  if (nFault.read())
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
  digitalWrite(nSleep, HIGH);  // enable driver
  digitalWrite(nConfig, LOW);  // enable phase mode (DC motor)
  digitalWrite(aPhase, HIGH);  // enable output to motor

  // motor error status
  if (digitalRead(nFault))
  {
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
  aEnable.writePWMData(speed);
}

#else
void Motor::setPwm()
{
  float speed = 100;  // get desired speed from usb
  aEnable.writePWMData(speed);
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
  int val = analogRead(aVSense);
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float Motor::getTorque()
{
  float torque = 5;  // get desired torque from usb
  return torque;
}

#else
float Motor::getTorque()
{
  float torque = 5;  // get desired torque from usb
  return torque;
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float Motor::setVRef(float measuredI, float desiredTorque)
{
  float measuredTorque = (measuredI)*torqueConst;
  float newVRef = (desiredTorque * 0.5) / torqueConst;

  vRef.writePWMData(newVRef);
  return measuredTorque;
}

#else
float Motor::setVRef(float measuredI, float desiredTorque)
{
  float measuredTorque = (measuredI)*torqueConst;
  float newVRef = desiredTorque * 0.5 / torqueConst;

  vRef.writePWMData(newVRef);
  return measuredTorque;
}
#endif

void Motor::update(int loop_counter)
{
  // Only update if update rate for the sensor is the same as the sampling
  // rate
  initialize();
  if (this->_refresh_rate == loop_counter)
  {
    // Publish Diagnostic messages
    Device::update(loop_counter);

    setPwm();
    measuredI = getISense();
    desiredTorque = getTorque();
    error = setVRef(measuredI, desiredTorque);

    sprintf(str, "Measured current = %f\nMeasured torque = %f\n", measuredI,
            error);
    print(str);
  }
}