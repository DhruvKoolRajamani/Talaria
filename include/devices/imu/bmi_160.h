/**
 * @file bmi_160.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-10-02
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef BMI_160_H
#define BMI_160_H

#include "devices/base/i2c_device.h"

#ifndef DISABLE_ROS
#include "sensor_msgs/Imu.h"
#include "std_msgs/Byte.h"
#endif

class BMI_160 : public I2CDevice
{
private:
#ifndef DISABLE_ROS
  ros::Publisher _pub_imu;
  // Create a custom message for IMU but for now using standard
  sensor_msgs::Imu _msg_imu;
  std_msgs::Byte _msg_chip_id;
#endif

protected:
public:
  // CONSTRUCTORS

  enum REGISTER_ADDRESS
  {
    CHIP_ID = 0x00,
    POLLING_ID = 0x12
  };

#ifndef DISABLE_ROS
  BMI_160(int address, I2CBus& i2c_bus, ros::NodeHandle& nh, uint8_t dev_index,
          const char* dev_name, const char* topic_name)
    : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name)
    , _pub_imu(topic_name, &(this->_msg_chip_id))
  {
    nh.advertise(_pub_imu);
  }
#else
  BMI_160(int address, I2CBus& i2c_bus, uint8_t dev_index)
    : I2CDevice(address, i2c_bus, dev_index)
  {
  }
#endif

  // DESTRUCTORS
  virtual ~BMI_160()
  {
  }

  // GETTERS

  // SETTERS

  // METHODS

  bool initialize() override
  {
    if (this->ping(REGISTER_ADDRESS::CHIP_ID))
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

  void update()
  {
    // Publish Diagnostic messages
    Device::update();

#ifndef DISABLE_ROS
    _msg_chip_id.data = this->getChipId();
    _pub_imu.publish(&(this->_msg_chip_id));
#endif
  }
};

#endif  // BMI_160_H