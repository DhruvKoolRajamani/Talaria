/**
 * @file strain_gauge.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-24
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef STRAIN_GAUGE_H
#define STRAIN_GAUGE_H

#include "devices/base/analog_device.h"

#ifndef DISABLE_ROS
// Add header file for custom ros message for bend sensor

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#endif

class StrainGauge : public AnalogDevice
{
private:
#ifndef DISABLE_ROS
  ros::Publisher _pub_strain_gauge;
  // Create a custom ros message for Bend but for now using standard
  std_msgs::Float32 _msg_strain_gauge;  // REPLACE THIS TO sensor_msgs::Float or
                                        // whatever. Google sensor msgs and
                                        // other ros messages and see if any
                                        // other message fits the requirements
#endif
  float _Vin = 3.3;
  float _Vout_unstrained = 0.0f;
  float _Vout_strained = 0.0f;
  float _strain = 0.0f;

protected:
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  StrainGauge(uint8_t id, PinName pin_name, ros::NodeHandle& nh,
              uint8_t dev_index, const char* dev_name, const char* topic_name,
              int refresh_rate)
    : AnalogDevice(id, pin_name, nh, dev_index, dev_name, topic_name,
                   refresh_rate)
    , _pub_strain_gauge(topic_name, &(this->_msg_strain_gauge))
  {
#else
  StrainGauge(uint8_t id, int pin_name, ros::NodeHandle& nh, uint8_t dev_index,
              const char* dev_name, const char* topic_name, int refresh_rate)
    : AnalogDevice(id, pin_name, nh, dev_index, dev_name, topic_name,
                   refresh_rate)
    , _pub_strain_gauge(topic_name, &(this->_msg_strain_gauge))
  {
#endif

    setIsTopicAdvertised(nh.advertise(_pub_strain_gauge));
  }
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  StrainGauge(uint8_t id, PinName pin_name, uint8_t dev_index, int refresh_rate)
    : AnalogDevice(id, pin_name, dev_index, refresh_rate)
  {
#else
  StrainGauge(uint8_t id, int pin_name, uint8_t dev_index, int refresh_rate)
    : AnalogDevice(id, pin_name, dev_index, refresh_rate)
  {
#endif
  }
#endif

  // DESTRUCTORS
  virtual ~StrainGauge() {}

  // GETTERS
  /**
   * @brief Get the Unstrained Voltage object
   *
   * @return float _Vout_unstrained
   */
  float getUnstrainedVoltage() { return _Vout_unstrained; }

  /**
   * @brief Get the Strain object
   *
   * @return float _strain
   */
  float getStrain() { return _strain; }
  // SETTERS

  // METHODS
  void calibrate()
  {
    // Take an average of upto 50 values with breaks of 10ms
    for (int i = 0; i < 50; i++)
    {
      _Vout_unstrained += this->readAnalogData();

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
      wait_ms(10);
#else
      delay(10);
#endif
    }
    _Vout_unstrained /= 50;
    setConfiguredStatus(true);
  }

  void update()
  {
    if (this->getEnabledStatus())
    {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
      uint64_t current_time = get_ms_count();
#else
      unsigned long current_time = millis();
#endif
      if ((first_update ||
           (current_time - _prev_update_time) >= _refresh_rate) &&
          this->getConfiguredStatus())
      {
        first_update = false;
        _prev_update_time = current_time;
        // Publish Diagnostic messages
        Device::update();

        float GF = 2;
        _Vout_strained = this->readAnalogData();
        _strain = (_Vout_strained - _Vout_unstrained) / _Vin;

#ifndef DISABLE_ROS
        _msg_strain_gauge.data = _strain;
        if (this->getIsTopicAdvertised())
          _pub_strain_gauge.publish(&(this->_msg_strain_gauge));
#endif
      }
    }
  }
};

#endif  // STRAIN_GAUGE_H