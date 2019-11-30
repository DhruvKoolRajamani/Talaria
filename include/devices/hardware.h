/**
 * @file hardware.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief Global definitions required by all files
 * @version 0.1
 * @date 2019-10-22
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#ifdef HAND
const int NUM_DEVICES = 2;
#elif defined(LLE)
const int NUM_DEVICES = 1;
#endif

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#ifndef DISABLE_ROS
#include "ros.h"
#include <std_msgs/String.h>
#else
static void print(const char* str)
{
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  printf(str);
#else
  Serial.print(str);
#endif
}
#endif
#include <limits.h>

typedef enum
{
  BEND_SENSOR_ID,
  AD_IMU_SENSOR_ID,
  MOTOR_ID,
  STRAIN_GAUGE_ID
} hand_devices_id_enum;

typedef enum
{
  BMI_IMU_SENSOR_ID,
  FSR_ID
} lle_devices_id_enum;

#endif  // HARDWARE_H