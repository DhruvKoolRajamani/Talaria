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

protected:
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS
  StrainGauge(uint8_t id, PinName a0, ros::NodeHandle& nh, uint8_t dev_index,
              const char* dev_name, const char* topic_name)
    : AnalogDevice(id, a0, nh, dev_index, dev_name, topic_name)
    // Change this->msg_chip_id to this-><custom_bend_message>
    , _pub_strain_gauge(this->getTopicName(), &(this->_msg_strain_gauge))
  {
    this->getNodeHandle()->advertise(_pub_strain_gauge);
  }
#else
  StrainGauge(uint8_t id, PinName a0, uint8_t dev_index)
    : AnalogDevice(id, a0, dev_index)
  {
  }
#endif

  // DESTRUCTORS
  virtual ~StrainGauge()
  {
  }

  // GETTERS

  // SETTERS

  // METHODS
  void calibrate()
  {
    // Take an average of upto 50 values with breaks of 10ms
    for (int i = 0; i < 50; i++)
    {
      _Vout_unstrained += this->readAnalogData();
      wait_ms(10);
    }
    _Vout_unstrained /= 50;
    setConfiguredStatus(true);
  }

  void update()
  {
    // Publish Diagnostic messages
    Device::update();

    float GF = 2;
    _Vout_strained = this->readAnalogData();
    float strain = (_Vout_strained - _Vout_unstrained) / _Vin;
    // {
    //   // ads_read_polled();
    // }

#ifndef DISABLE_ROS
    // _msg_chip_id.data = this->getChipId();
    // _pub_strain_gauge.publish(&(this->_msg_chip_id));
#endif
  }
};

#endif  // STRAIN_GAUGE_H