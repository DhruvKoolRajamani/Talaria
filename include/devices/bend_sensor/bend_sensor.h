/**
 * @file bend_sensor.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-24
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef BEND_SENSOR_H
#define BEND_SENSOR_H

#include "devices/base/i2c_device.h"

#ifndef DISABLE_ROS
#include "sensor_msgs/Imu.h"

// Add header file for custom ros message for bend sensor

#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#endif

class BendSensor : public I2CDevice
{
private:
#ifndef DISABLE_ROS
  ros::Publisher _pub_bend_sensor;
  // Create a custom ros message for Bend but for now using standard
  std_msgs::Float32 _msg_bend_sensor;  // REPLACE THIS TO sensor_msgs::Float or
                                       // whatever. Google sensor msgs and other
                                       // ros messages and see if any other
                                       // message fits the requirements
  std_msgs::Byte _msg_chip_id;
#endif

protected:
public:
  enum REGISTER_ADDRESS
  {
    CHIP_ID = 10,
    POLLING_ID = 0x12
  };

  // CONSTRUCTORS
#ifndef DISABLE_ROS
  BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name)
    : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name)
    // Change this->msg_chip_id to this-><custom_bend_message>
    , _pub_bend_sensor(topic_name, &(this->_msg_chip_id))
  {
    nh.advertise(_pub_bend_sensor);
  }
#else
  BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index)
    : I2CDevice(address, i2c_bus, dev_index)
  {
  }
#endif

  // DESTRUCTORS
  virtual ~BendSensor()
  {
  }

  // GETTERS

  // SETTERS

  // METHODS
  /**
   * @brief Initialize sensor and run all calibration and setup commands.
   *
   * @return true
   * @return false
   */
  bool initialize() override
  {
    this->ping();
    if (this->getChipId() == 2)
    {
      this->enable();
      this->setHealthStatus(true);
      // Perform calibration
      this->setConfiguredStatus(true);

#ifndef DISABLE_ROS
      _msg_chip_id.data = this->getChipId();
#endif

      this->update();

      return true;
    }
    else
      return false;
  }

  bool ping(int chip_id_reg_address = REGISTER_ADDRESS::CHIP_ID,
            int delay_ms = 2) override
  {
    uint8_t buffer[3];
    if (readRegister(chip_id_reg_address, buffer, sizeof(buffer), false,
                     delay_ms))
    {
      setHealthStatus(true);
      setConfiguredStatus(true);
      setEnabledStatus(true);
      this->setChipId(buffer[0]);

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
      _diagnostic_chip_id.value = (char*)buffer;
      this->setDiagnosticsData(_diagnostic_chip_id);
#endif
#else
      printf("Chip Id is: %x", buffer);
#endif
      return true;
    }
    else
      return false;
  }

  void update()
  {
    // Publish Diagnostic messages
    Device::update();

    this->ping();
    // {
    //   // ads_read_polled();
    // }

#ifndef DISABLE_ROS
    _msg_chip_id.data = this->getChipId();
    _pub_bend_sensor.publish(&(this->_msg_chip_id));
#endif
  }
};

#endif  // BEND_SENSOR_H