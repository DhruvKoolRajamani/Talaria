#include "devices/base/analog_device.h"
#include "devices/base/pwm_device.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
PwmDevice aEnable(1, p25, 1);     // p136 -- p8
PwmDevice vRef(1, p26, 1);        // p137 -- p4 + jumper
AnalogDevice aVSense(0, p19, 1);  // p90 -- across R3
DigitalOut aPhase(p5);            // p101 -- p7
DigitalOut nSleep(p6);            // p94 -- p3
DigitalOut nConfig(p7);           // p96 -- p5 + jumper
DigitalIn nFault(p8);             // p95 -- p7r
#else
PwmDevice aEnable(3);  // p136 -- p8
PwmDevice vRef(6);     // p137 -- p4 + jumper
int aVSense = A0;      // p90 -- across R3 A10
int aPhase = 8;        // p101 -- p7 br 44
int nSleep = 12;       // p94 -- p3 or 51
int nConfig = 11;      // p96 -- p5 + jumper bla 49
int nFault = 10;       // p95 -- p7r gr 50
#endif

#define aRSense 0.1
#define torqueConst 10.9

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
int initMotor()
{
  nSleep = 1;   // enable driver
  nConfig = 0;  // enable phase mode (DC motor)
  aPhase = 1;   // enable output to motor

  return nFault.read();
}

#else
int initMotor()
{
  digitalWrite(nSleep, HIGH);  // enable driver
  digitalWrite(nConfig, LOW);  // enable phase mode (DC motor)
  digitalWrite(aPhase, HIGH);  // enable output to motor

  return digitalRead(nFault);  // motor error status
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
void setPwm()
{
  float speed = 50;  // get desired speed from usb
  aEnable.writePWMData(speed);
}

#else
void setPwm()
{
  float speed = 100;  // get desired speed from usb
  aEnable.writePWMData(speed);
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float getISense()
{
  int val = aVSense.read();
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
}

#else
float getISense()
{
  int val = analogRead(aVSense);
  float vSense = val * 5.0 / 1023.0;
  float iSense = vSense * aRSense;
  return iSense;
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float getTorque()
{
  float torque = 5;  // get desired torque from usb
  return torque;
}

#else
float getTorque()
{
  float torque = 5;  // get desired torque from usb
  return torque;
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float setVRef(float measuredI, float desiredTorque)
{
  float measuredTorque = (measuredI)*torqueConst;
  float newVRef = (desiredTorque * 0.5) / torqueConst;

  vRef.writePWMData(newVRef);
  return measuredTorque;
}

#else
float setVRef(float measuredI, float desiredTorque)
{
  float measuredTorque = (measuredI)*torqueConst;
  float newVRef = desiredTorque * 0.5 / torqueConst;

  vRef.writePWMData(newVRef);
  return measuredTorque;
}
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
void main()
{
  float measuredI, desiredTorque, error;
  char str[100];
  sprintf(str, "Motor Status: %d", initMotor());
  print(str);
  while (1)
  {
    setPwm();
    measuredI = getISense();
    desiredTorque = getTorque();
    error = setVRef(measuredI, desiredTorque);

    sprintf(str, "Measured current = %f\nMeasured torque = %f\n", measuredI,
            error);
    print(str);
    // sendSerial(speed[0],measuredI)
  }
}

#else
void setup()
{
  pinMode(aEnable, OUTPUT);
  pinMode(vRef, OUTPUT);
  pinMode(aVSense, INPUT);
  pinMode(aPhase, OUTPUT);
  pinMode(nSleep, OUTPUT);
  pinMode(nConfig, OUTPUT);
  pinMode(nFault, INPUT);

  // analogWriteResolution(12);

  pinMode(13, OUTPUT);
  int init_status = initMotor();
  Serial.begin(9600);
  Serial.println("Motor Status:");
  Serial.println(init_status);
}
void loop()
{
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
  int init_status = initMotor();

  Serial.println("Motor Status:");
  Serial.println(init_status);

  float measuredI, desiredTorque, error;
  setPwm();
  measuredI = getISense();
  desiredTorque = getTorque();
  error = setVRef(measuredI, desiredTorque);

  Serial.println("Measured current = ");
  Serial.println(measuredI);

  Serial.println("Measured Torque = ");
  Serial.println(error);
}
#endif