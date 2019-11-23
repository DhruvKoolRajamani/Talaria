/**
 * @file device_manager.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#define PACKET_SIZE 58

#ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
#include "mbed.h"

#elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
#include "Arduino.h"
#include "util/crc16.h"

#endif
#include "devices/hardware.h"

#include "devices/base/device.h"

#include "devices/imu/bmi_160.h"
#include "devices/bend_sensor/bend_sensor.h"
#include "devices/strain_gauge/strain_gauge.h"

class DeviceManager
{
private:
  static const int BYTE_STREAM_PUB_SIZE = 100;
  static const int BYTE_STREAM_SUB_SIZE = 20;
  Device* _devices[NUM_DEVICES];

  uint8_t _byte_stream_pub[BYTE_STREAM_PUB_SIZE];
  uint8_t _byte_stream_sub[BYTE_STREAM_SUB_SIZE];

#ifndef DISABLE_ROS
  ros::NodeHandle* _nh;
  int _max_refresh_rate = 1;
#endif

protected:
public:
  /** VARIABLES */
  enum DEV_INDEX
  {
    IMU = 0,
    ENCODER = 1
  };

  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
  /**
   * @brief Construct a new Device Manager object
   *
   * @param nodehandle
   */
  DeviceManager(ros::NodeHandle& nh);
#else
  /**
   * @brief Construct a new Device Manager object
   *
   */
  DeviceManager();
#endif

  /** DESTRUCTOR */
  virtual ~DeviceManager()
  {
  }

  /** GETTERS */
  int getMaxRefreshRate();

  /** SETTERS */

  /** METHODS */

  bool initialize();

  bool addDevice(Device* device, int index);

  bool readByteStream();

  uint8_t makePacket(float* measuredData)
  {
    uint8_t startByte, stopByte, crcWidth=0;
    startByte = 0xAA;
    stopByte = 0x0F;
    uint32_t checksum;
    int32_t crcStatus;
    bool makePacketStatus;      
    uint8_t bytes[PACKET_SIZE];

    for (int i = 0; i < PACKET_SIZE; i++)
    {
      float2Bytes(measuredData[i],&bytes[i]);
    }

    checksum, crcWidth, crcStatus = fetchCrcChecksum(bytes);
    int packetLength = PACKET_SIZE + crcWidth;
    if (crcStatus == 1)
    {
      
      bytes[0] = startByte;
      bytes[packetLength-1] = stopByte;

      if (crcWidth!=0)
      {bytes[packetLength-1-crcWidth] = checksum;}
      
    }

    else 
    {
      makePacketStatus = false;
    }

  }

  void writeByteStream();

  bool initializeDevices();

  void updateDevices(int loop_counter = 1);

  void float2Bytes(float val, uint8_t* bytes_array)
  {
    // Create union of shared memory space
    union 
    {
      float float_variable;
      uint8_t temp_array[sizeof(float)];
    } u;
    // Overite bytes of union with float variable
    u.float_variable = val;
    // Assign bytes to input array
    memcpy(bytes_array, u.temp_array, sizeof(float));

  }

  #ifdef PIO_FRAMEWORK_MBED_RTOS_PRESENT
  float fetchCrcChecksum(uint8_t data[])
  {
    MbedCRC<POLY_32BIT_ANSI, 32> ct;
    
    uint32_t crcPolynomial,checksum;
    uint8_t crcWidth;
    int32_t crcStatus;

    checksum = 0;

    crcPolynomial = ct.get_polynomial();
    crcWidth = ct.get_width();
    crcStatus = ct.compute((void *)data, strlen((const char*)data), &checksum);

    return checksum, crcWidth, crcStatus;

  }

  #elif defined PIO_FRAMEWORK_ARDUINO_PRESENT
  float calcCRC(uint8_t data[])
  {
    uint8_t crcWidth;
    int32_t crcStatus;
    uint32_t c=hecksum0; // starting value as you like, must be the same before each calculation
    for (int i=0;i<strlen(data);i++) // for each character in the string
    {
      checksum = _crc16_update (crc, data[i]); // update the crc value
    }
    crcWidth = 1;
    crcStatus = 1;
    return checksum, crcWidth, crcStatus;
  }

  #endif
};

extern DeviceManager device_manager;

#endif  // DEVICE_MANAGER_H