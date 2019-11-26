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
#elif LLE
const int NUM_DEVICES = 1;
#endif

#ifndef DISABLE_ROS
#include "ros.h"
#include <std_msgs/String.h>
#endif
#include <limits.h>

typedef enum
{
  BEND_SENSOR_ID,
  AD_IMU_SENSOR_ID,
  CURRENT_SENSE_ID,
  STRAIN_GAUGE_ID
} hand_devices_id_enum;

typedef enum
{
  BMI_IMU_SENSOR_ID,
  FSR_ID
} lle_devices_id_enum;

#endif  // HARDWARE_H