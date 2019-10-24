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

const int NUM_DEVICES = 2;

#ifndef DISABLE_ROS
#include <ros.h>
#include <std_msgs/String.h>
#endif

#endif  // HARDWARE_H