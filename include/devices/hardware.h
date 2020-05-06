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
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
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
  ADI_IMU_ID,
  MOTOR_ID
} hand_devices_id_enum;

typedef enum
{
  BMI_IMU_SENSOR_ID,
  FSR_ID
} lle_devices_id_enum;

#ifndef DISABLE_ROS
const float RATE = 10;
#else
const float RATE = 1;
#endif

template <typename T>
char* concatenate(T s1, T s2, T s3 = nullptr, T s4 = nullptr, T s5 = nullptr,
                  T s6 = nullptr)
{
  int count = 1;
  int len_s1 = strlen(s1);
  int len_s2 = strlen(s2);
  int len_s3 = 0;
  if (s3 != NULL)
  {
    len_s3 = strlen(s3);
    count++;
  }
  int len_s4 = 0;
  if (s4 != NULL)
  {
    len_s4 = strlen(s4);
    count++;
  }
  int len_s5 = 0;
  if (s5 != NULL)
  {
    len_s5 = strlen(s5);
    count++;
  }
  int len_s6 = 0;
  if (s6 != NULL)
  {
    len_s6 = strlen(s6);
    count++;
  }

  int len_total = len_s1 + len_s2 + len_s3 + len_s4 + len_s5 + len_s6 + 1;

  char* str = static_cast<char*>(malloc(len_total));

  T s_ptr[] = { s2, s3, s4, s5, s6 };
  int len_ptr[] = { len_s2, len_s3, len_s4, len_s5, len_s6 };

  snprintf(str, len_s1 + 1, s1);
  for (int i = 0; i < count; i++)
  {
    strncat(str, s_ptr[i], len_ptr[i] + 1);
  }

  return str;
}

#endif  // HARDWARE_H