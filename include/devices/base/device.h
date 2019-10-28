/**
 * @file device.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-01
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef DEVICE_H
#define DEVICE_H

#include "mbed.h"

#include "devices/hardware.h"

#ifndef DISABLE_ROS
#include "ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#endif
class Device
{
private:
  bool _en_status;
  bool _conf_status;
  bool _health_status;

  static const int DEVICE_ID_SIZE = 8;

  uint8_t* _dev_Id;
  uint8_t _dev_index;

#ifndef DISABLE_ROS
  ros::NodeHandle* _nh;

  static const int DEVICE_NAME_SIZE = 20;
  static const int DEVICE_TOPIC_NAME_SIZE = 30 + DEVICE_NAME_SIZE;

#ifndef DISABLE_DIAGNOSTICS
  static ros::Publisher _pub_diagnostics;
  static bool _is_diagnostic_published;

  diagnostic_msgs::DiagnosticStatus _msg_diagnostic_status;
  diagnostic_msgs::DiagnosticArray _msg_diagnostic_array;
#endif

  char* _dev_name;
  char* _topic_name;
#else
  // uint8_t* _shared_data_stream;
  bool _raise_write_complete_flag = false;
#endif

protected:
public:
  /** CONSTRUCTORS */

  /**
   * @brief Construct a new Device object
   *
   */
  Device();

#ifndef DISABLE_ROS
  /**
   * @brief Construct a new Device object
   *
   * @param uint8_t dev_index
   * @param const char* dev_name
   */
  Device(uint8_t dev_index, ros::NodeHandle& nh, const char* dev_name = NULL,
         const char* prefix_path = NULL);
#else
  /**
   * @brief Construct a new Device object
   *
   * @param uint8_t dev_index
   * @param const char* dev_name
   */
  Device(uint8_t dev_index);
#endif

  /** DESTRUCTOR */

  /**
   * @brief Destroy the Device object
   *
   */
  ~Device();

  /** GETTERS */

#ifndef DISABLE_ROS
  /**
   * @brief Get the Node Handle object
   *
   * @return ros::NodeHandle*
   */
  ros::NodeHandle* getNodeHandle();
#endif

  /**
   * @brief Get the Id Size object
   *
   * @return int DEVICE_ID_SIZE
   */
  int getIdSize();

  /**
   * @brief Get the Id object
   *
   * @return uint8_t* _dev_Id
   */
  uint8_t* getId();

  /**
   * @brief Get the Index object
   *
   * @return uint8_t _dev_index
   */
  uint8_t getIndex();

  /**
   * @brief Get the Health Status object
   *
   * @return true _health_status
   * @return false _health_status
   */
  bool getHealthStatus();

  /**
   * @brief Get the Configured Status object
   *
   * @return true _conf_status
   * @return false _conf_status
   */
  bool getConfiguredStatus();

  /**
   * @brief Get the Enabled Status object
   *
   * @return true _en_status
   * @return false _en_status
   */
  bool getEnabledStatus();

#ifndef DISABLE_ROS
  /**
   * @brief Get the Device Name
   *
   * @return char* _dev_name
   */
  char* getDeviceName();
#endif

#ifndef DISABLE_ROS
  /**
   * @brief Get the Topic Name object
   *
   * @return char* _topic_name
   */
  char* getTopicName();
#endif

  /** METHODS */

  /**
   * @brief Initialize the sensor and set up ROS topics if any
   *
   */
  virtual bool initialize();

  /**
   * @brief Enable the Device
   *
   */
  virtual void enable();

  /**
   * @brief Disable the Device
   *
   */
  virtual void disable();

  /**
   * @brief Update the values of the device whether read or write
   *
   */
  virtual void update();

  /**
   * @brief Reset the pin for the Device and wait for a delay in ms before
   * restarting it
   *
   * @param PinName pin
   * @param int delay_ms
   */
  virtual void reset(PinName pin, int delay_ms = 100);

  /** SETTERS */

#ifndef DISABLE_ROS
  /**
   * @brief Set the Node Handle object
   *
   * @param ptrnh
   */
  void setNodeHandle(ros::NodeHandle* nh);
#endif

  /**
   * @brief Set the Pin State object
   *
   * @param PinName pin
   * @param bool state
   */
  virtual void setPinState(PinName pin, bool state);

  /**
   * @brief Set the Index object
   *
   * @param uint8_t index
   */
  void setIndex(uint8_t index);

  /**
   * @brief Set the Id object
   *
   * @param uint8_t id
   */
  void setId(uint8_t id);

  /**
   * @brief Set the Health Status object
   *
   * @param state
   */
  void setHealthStatus(bool state);

  /**
   * @brief Set the Enabled Status object
   *
   * @param state
   */
  void setEnabledStatus(bool state);

  /**
   * @brief Set the Configured Status object
   *
   * @param state
   */
  void setConfiguredStatus(bool state);

#ifndef DISABLE_ROS
  /**
   * @brief Set the Name of the Device
   *
   * @param const char* dev_name
   */
  void setDeviceName(const char* dev_name);
#endif

#ifndef DISABLE_ROS
  /**
   * @brief Set the Debug Data object
   *
   * @param data
   */
  void setDiagnosticsData(diagnostic_msgs::KeyValue key_value);
#endif

  /**
   * @brief Compare two strings
   *
   * @param str1
   * @param str2
   * @return true if comparison is true
   * @return false if comparison fails
   */
  bool strcmp(const char* str1, const char* str2);
};

#endif  // DEVICE_H