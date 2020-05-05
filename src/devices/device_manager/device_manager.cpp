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
  if (index > NUM_DEVICES) return false;
  _devices[index] = device;
  _init_status[index] = false;
  return true;
}

bool DeviceManager::readByteStream(/** Add Callback here */)  // for motors
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

bool DeviceManager::getInitStatus()
{
  bool status = true;
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    if (!_init_status[i]) status = false;
  }

  return status;
}

bool DeviceManager::initializeDevices()
{
  // Add functionality to store time?
  bool init_flag = false;
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    if (_devices[i] == NULL)
    {
#ifdef DISABLE_ROS
      char str[100];
      sprintf(str, "No Devices yet\n");
      print(str);
#endif
      continue;
    }
    else
    {
#ifdef DISABLE_ROS
      char str[100];
      sprintf(str, "device: %d\n", _devices[i]->getIndex());
      print(str);
#endif
      if (!_init_status[i]) init_flag = _devices[i]->initialize();
      _init_status[i] = init_flag;
    }
  }
  return init_flag;
}

void DeviceManager::initializeDevice(int device_id)
{
  _devices[device_id]->initialize();
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
      if (_init_status[i]) _devices[i]->update();
    }
  }
}

Device* DeviceManager::getDevice(int i) { return _devices[i]; }

int DeviceManager::getMaxRefreshRate()
{
  for (int i = 0; i < NUM_DEVICES; i++)
  {
    if (_devices[i] == NULL)
      continue;
    else
    {
#ifdef DISABLE_ROS
      char str[50];
      sprintf(str, "Device: %d\tRate: %d\n", _devices[i]->getIndex(),
              _devices[i]->_refresh_rate);
      print(str);
#endif
      if (_max(this->_max_refresh_rate, _devices[i]->getRefreshRate()) !=
          this->_max_refresh_rate)
        this->_max_refresh_rate = _devices[i]->getRefreshRate();
    }
  }
  return this->_max_refresh_rate;
}

#ifdef COMPUTE_CRC
uint8_t DeviceManager::makePacket(float* measuredData)
{
  uint8_t startByte, stopByte, crcWidth = 0;
  startByte = 0xAA;
  stopByte = 0x0F;
  uint32_t checksum;
  int32_t crcStatus;
  bool makePacketStatus;
  uint8_t bytes[PACKET_SIZE];

  for (int i = 0; i < PACKET_SIZE; i++)
  {
    float2Bytes(measuredData[i], &bytes[i]);
  }

  checksum, crcWidth, crcStatus = fetchCrcChecksum(bytes);
  int packetLength = PACKET_SIZE + crcWidth;
  if (crcStatus == 1)
  {
    bytes[0] = startByte;
    bytes[packetLength - 1] = stopByte;

    if (crcWidth != 0)
    {
      bytes[packetLength - 1 - crcWidth] = checksum;
    }
  }

  else
  {
    makePacketStatus = false;
  }
}
#endif

#ifdef COMPUTE_CRC
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
float DeviceManager::fetchCrcChecksum(uint8_t data[])
{
  MbedCRC<POLY_32BIT_ANSI, 32> ct;

  uint32_t crcPolynomial, checksum;
  uint8_t crcWidth;
  int32_t crcStatus;

  checksum = 0;

  crcPolynomial = ct.get_polynomial();
  crcWidth = ct.get_width();
  crcStatus = ct.compute((void*)data, strlen((const char*)data), &checksum);

  return checksum, crcWidth, crcStatus;
}

#else
float DeviceManager::calcCRC(uint8_t data[])
{
  uint8_t crcWidth;
  int32_t crcStatus;
  uint32_t c = hecksum0;  // starting value as you like, must be the same
                          // before each calculation
  for (int i = 0; i < strlen(data); i++)  // for each character in the string
  {
    checksum = _crc16_update(crc, data[i]);  // update the crc value
  }
  crcWidth = 1;
  crcStatus = 1;
  return checksum, crcWidth, crcStatus;
}

#endif
#endif
