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

DeviceManager::DeviceManager()
{
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    _devices[i] = NULL;
  }
}

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

bool DeviceManager::readByteStream(/* Add callback ptr*/)
{
  // Perform CRC here
  // Make this interrupt based Serial
}

void DeviceManager::writeByteStream()
{
  // Perform CRC here
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