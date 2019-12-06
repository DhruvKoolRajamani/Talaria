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

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#ifdef COMPUTE_CRC
#include "util/crc16.h"
#endif
#endif

#include "devices/hardware.h"

#include "devices/base/device.h"

#include "devices/imu/bmi_160.h"
#include "devices/bend_sensor/bend_sensor.h"
#include "devices/strain_gauge/strain_gauge.h"
#include "devices/base/pwm_device.h"
#include "devices/base/analog_device.h"
#include "devices/motor/motor.h"

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
#endif
  int _max_refresh_rate = 1;

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

#ifdef COMPUTE_CRC
  uint8_t makePacket(float* measuredData);
#endif

  void writeByteStream();

  bool initializeDevices();

  void updateDevices(int loop_counter = 1);

  inline void float2Bytes(float val, uint8_t* bytes_array)
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

  inline int _max(int a, int b)
  {
    return ((a) > (b) ? (a) : (b));
  }

#ifdef COMPUTE_CRC
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  float fetchCrcChecksum(uint8_t data[]);
#else
  float calcCRC(uint8_t data[]);
#endif
#endif
};

extern DeviceManager device_manager;

#endif  // DEVICE_MANAGER_H