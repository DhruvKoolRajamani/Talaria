/**
 * @file device.cpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-21
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "devices/base/device.h"

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
diagnostic_msgs::DiagnosticArray temp_msg_diagnostic_array;
ros::Publisher Device::_pub_diagnostics("/diagnostics",
                                        &temp_msg_diagnostic_array);

bool Device::_is_diagnostic_published = false;
#endif
#endif

/** CONSTRUCTORS */

/**
 * @brief Construct a new Device object
 *
 */
Device::Device()
  : _en_status(true), _conf_status(false), _health_status(false), _dev_index(0)
{
  _dev_Id = static_cast<uint8_t*>(malloc(DEVICE_ID_SIZE));
#ifndef DISABLE_ROS
  _dev_name = static_cast<char*>(malloc(DEVICE_NAME_SIZE));
  _topic_name = static_cast<char*>(malloc(DEVICE_TOPIC_NAME_SIZE));
#endif
}

#ifndef DISABLE_ROS
/**
 * @brief Construct a new Device object
 *
 * @param uint8_t dev_index
 * @param const char* dev_name
 */
Device::Device(uint8_t dev_index, ros::NodeHandle& nh, const char* dev_name,
               const char* topic_name)
  : _en_status(true)
  , _conf_status(false)
  , _health_status(false)
  , _dev_index(dev_index)
  , _nh(&nh)
{
  _dev_Id = static_cast<uint8_t*>(malloc(DEVICE_ID_SIZE));
  _dev_name = static_cast<char*>(malloc(DEVICE_NAME_SIZE));
  _topic_name = static_cast<char*>(malloc(DEVICE_TOPIC_NAME_SIZE));
  if (dev_name != NULL)
    memcpy(_dev_name, dev_name, sizeof(dev_name));
  if (topic_name != NULL)
    memcpy(_topic_name, topic_name, sizeof(topic_name));

#ifndef DISABLE_DIAGNOSTICS
  if (!_is_diagnostic_published)
  {
    for (int i = 0; i < NUM_DEVICES; i++)
    {
      _msg_diagnostic_array.status = new diagnostic_msgs::DiagnosticStatus();
      _msg_diagnostic_array.status_length++;
    }

    _nh->advertise(_pub_diagnostics);
    _is_diagnostic_published = true;
  }

  if (dev_name != NULL)
  {
    _msg_diagnostic_status.name = _dev_name;
    _msg_diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    _msg_diagnostic_status.values = new diagnostic_msgs::KeyValue();
    _msg_diagnostic_status.values_length++;
    _msg_diagnostic_status.values->key = "enable";
    _msg_diagnostic_status.values->value = "true";
    _msg_diagnostic_status.values = new diagnostic_msgs::KeyValue();
    _msg_diagnostic_status.values_length++;
    _msg_diagnostic_status.values->key = "health";
    _msg_diagnostic_status.values->value = "false";
    _msg_diagnostic_status.values = new diagnostic_msgs::KeyValue();
    _msg_diagnostic_status.values_length++;
    _msg_diagnostic_status.values->key = "conf";
    _msg_diagnostic_status.values->value = "false";
    _msg_diagnostic_status.values = new diagnostic_msgs::KeyValue();
    _msg_diagnostic_status.values_length++;
    _msg_diagnostic_status.values->key = "dev_index";
    char tmp_dev_index[2];
    sprintf(tmp_dev_index, "%d", _dev_index);
    _msg_diagnostic_status.values->value = tmp_dev_index;

    _msg_diagnostic_array.status[_dev_index] = _msg_diagnostic_status;
  }
#endif
}
#else
/**
 * @brief Construct a new Device object
 *
 * @param uint8_t dev_index
 * @param const char* dev_name
 */
Device::Device(uint8_t dev_index)
  : _en_status(true)
  , _conf_status(false)
  , _health_status(false)
  , _dev_index(dev_index)
{
  _dev_Id = static_cast<uint8_t*>(malloc(DEVICE_ID_SIZE));
}
#endif

/** DESTRUCTOR */

/**
 * @brief Destroy the Device object
 *
 */
Device::~Device()
{
  free(_dev_Id);
#ifndef DISABLE_ROS
  free(_topic_name);
  free(_dev_name);
#endif
}

/** GETTERS */

#ifndef DISABLE_ROS
/**
 * @brief Get the Node Handle object
 *
 * @return ros::NodeHandle*
 */
ros::NodeHandle* Device::getNodeHandle()
{
  return _nh;
}
#endif

/**
 * @brief Get the Id Size object
 *
 * @return int DEVICE_ID_SIZE
 */
int Device::getIdSize()
{
  return DEVICE_ID_SIZE;
}

/**
 * @brief Get the Id object
 *
 * @return uint8_t* _dev_Id
 */
uint8_t* Device::getId()
{
  return _dev_Id;
}

/**
 * @brief Get the Index object
 *
 * @return uint8_t _dev_index
 */
uint8_t Device::getIndex()
{
  return _dev_index;
}

/**
 * @brief Get the Health Status object
 *
 * @return true _health_status
 * @return false _health_status
 */
bool Device::getHealthStatus()
{
  return _health_status;
}

/**
 * @brief Get the Configured Status object
 *
 * @return true _conf_status
 * @return false _conf_status
 */
bool Device::getConfiguredStatus()
{
  return _conf_status;
}

/**
 * @brief Get the Enabled Status object
 *
 * @return true _en_status
 * @return false _en_status
 */
bool Device::getEnabledStatus()
{
  return _en_status;
}

#ifndef DISABLE_ROS
/**
 * @brief Get the Device Name
 *
 * @return char* _dev_name
 */
char* Device::getDeviceName()
{
  return _dev_name;
}
#endif

#ifndef DISABLE_ROS
/**
 * @brief Get the Topic Name object
 *
 * @return char* _topic_name
 */
char* Device::getTopicName()
{
  return _topic_name;
}
#endif

/** METHODS */

/**
 * @brief Initialize the sensor and set up ROS topics if any
 *
 */
bool Device::initialize()
{
  enable();

  _conf_status = true;
  _health_status = true;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[0].key = "enable";
  _msg_diagnostic_status.values[0].value = "true";

  _msg_diagnostic_status.values[1].key = "health";
  _msg_diagnostic_status.values[1].value = "true";

  _msg_diagnostic_status.values[2].key = "conf";
  _msg_diagnostic_status.values[2].value = "true";
#endif
#endif

  return _en_status && _conf_status && _health_status;
}

/**
 * @brief Enable the Device
 *
 */
void Device::enable()
{
  _en_status = true;
}

/**
 * @brief Disable the Device
 *
 */
void Device::disable()
{
  _en_status = false;
  _conf_status = false;
  _health_status = false;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[0].key = "enable";
  _msg_diagnostic_status.values[0].value = "false";

  _msg_diagnostic_status.values[1].key = "health";
  _msg_diagnostic_status.values[1].value = "false";

  _msg_diagnostic_status.values[2].key = "conf";
  _msg_diagnostic_status.values[2].value = "false";
#endif
#endif
}

/**
 * @brief Update the values of the device whether read or write
 *
 */
void Device::update()
{
  // Run all functions to get/set data

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _pub_diagnostics.publish(&(this->_msg_diagnostic_array));
#endif
// #else
#endif
}

/**
 * @brief Reset the pin for the Device and wait for a delay in ms before
 * restarting it
 *
 * @param PinName pin
 * @param int delay_ms
 */
void Device::reset(PinName pin, int delay_ms)
{
  DigitalOut resetPin(pin, false);
  wait_ms(delay_ms);
  resetPin = true;
}

bool Device::strcmp(const char* str1, const char* str2)
{
  int cnt1 = 0, cnt2 = 0;
  if (str1 != NULL && str2 != NULL)
  {
    while (str1[cnt1] != '\0')
    {
      cnt1++;
    }
    while (str2[cnt2] != '\0')
    {
      cnt2++;
    }

    if (cnt1 != cnt2)
      return false;

    for (int i = 0; i < cnt1; i++)
    {
      if (str1[i] != str2[i])
        return false;
    }
    return true;
  }
  else
    return false;
}

/** SETTERS */

#ifndef DISABLE_ROS
/**
 * @brief Set the Node Handle object
 *
 * @param ptrnh
 */
void Device::setNodeHandle(ros::NodeHandle* nh)
{
  _nh = nh;
}
#endif

/**
 * @brief Set the Pin State object
 *
 * @param PinName pin
 * @param bool state
 */
void Device::setPinState(PinName pin, bool state)
{
  int value = (int)state;
  DigitalOut Pin(pin, value);
}

/**
 * @brief Set the Index object
 *
 * @param uint8_t index
 */
void Device::setIndex(uint8_t index)
{
  _dev_index = index;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[3].key = "dev_index";
  char tmp_dev_index[2];
  sprintf(tmp_dev_index, "%d", _dev_index);
  _msg_diagnostic_status.values[3].value = tmp_dev_index;
#endif
#endif
}

/**
 * @brief Set the Id object
 *
 * @param uint8_t id
 */
void Device::setId(uint8_t id)
{
  *_dev_Id = id;
}

/**
 * @brief Set the Health Status object
 *
 * @param state
 */
void Device::setHealthStatus(bool state)
{
  _health_status = state;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[1].key = "health";
  if (state)
    _msg_diagnostic_status.values[1].value = "true";
  else
    _msg_diagnostic_status.values[1].value = "false";
#endif
#endif
}

/**
 * @brief Set the Enabled Status object
 *
 * @param state
 */
void Device::setEnabledStatus(bool state)
{
  _en_status = state;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[0].key = "enable";
  if (state)
    _msg_diagnostic_status.values[0].value = "true";
  else
    _msg_diagnostic_status.values[0].value = "false";
#endif
#endif
}

/**
 * @brief Set the Configured Status object
 *
 * @param state
 */
void Device::setConfiguredStatus(bool state)
{
  _conf_status = state;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.values[2].key = "conf";
  if (state)
    _msg_diagnostic_status.values[2].value = "true";
  else
    _msg_diagnostic_status.values[2].value = "false";
#endif
#endif
}

#ifndef DISABLE_ROS
/**
 * @brief Set the Name of the Device
 *
 * @param const char* dev_name
 */
void Device::setDeviceName(const char* dev_name)
{
  memcpy(_dev_name, dev_name, sizeof(dev_name));
#ifndef DISABLE_DIAGNOSTICS
  _msg_diagnostic_status.name = _dev_name;
  _msg_diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
#endif
}
#endif

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
/**
 * @brief Set the Debug Data object
 *
 * @param data
 */
void Device::setDiagnosticsData(diagnostic_msgs::KeyValue key_value)
{
  bool key_check = false;
  int i = 0;
  while (i < _msg_diagnostic_status.values_length)
  {
    if (!strcmp(_msg_diagnostic_status.values[i].key, key_value.key))
    {
      i++;
      continue;
    }
    else
    {
      key_check = true;
      break;
    }
  }
  if (key_check)
  {
    _msg_diagnostic_status.values[i].key = key_value.key;
    _msg_diagnostic_status.values[i].value = key_value.value;
  }
  else
  {
    _msg_diagnostic_status.values = new diagnostic_msgs::KeyValue(key_value);
    _msg_diagnostic_status.values_length++;
  }
  _msg_diagnostic_array.status[_dev_index] = _msg_diagnostic_status;
}
#endif
#endif