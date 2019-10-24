/**
 * @file device_manager.cpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "devices/device_manager/device_manager.h"

#ifndef DISABLE_ROS
DeviceManager::DeviceManager(ros::NodeHandle& nh) : _nh(&nh)
{
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    _devices[i] = NULL;
  }
}
#else
DeviceManager::DeviceManager()
{
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    _devices[i] = NULL;
  }
}
#endif

bool DeviceManager::initialize()
{
  // Serial read and write initialize
}

bool DeviceManager::addDevice(Device* device, int index)
{
  if (index > NUM_DEVICES)
    return false;
  _devices[index] = device;
  return true;
}

bool DeviceManager::readByteStream(/* Add callback ptr*/)  // for motors
{
  // Perform CRC here
  // Make this interrupt based Serial
}

void DeviceManager::writeByteStream()
{
#ifndef DISABLE_ROS
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    // _devices.publish();
  }
// #else
// Perform CRC here
#endif
}

void DeviceManager::initializeDevices()
{
}

void DeviceManager::updateDevices()
{
  // Add functionality to store time?
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    if (_devices[i] == NULL)
      continue;
    else
    {
      _devices[i]->update();
    }
  }
}