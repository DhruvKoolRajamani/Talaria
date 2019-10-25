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
// REGISTER ADDRESS
#define CMD_ADDRESS 0x7E
#define SOFT_RESET 0xB6
#define ACC_NORMAL_MODE 0x11
#define ERR_REG 0x02
#define PMU_STATUS 0x03

// Data reg addresses
#define DATA_8 0x0C //X 7:0 bits
#define DATA_9 0x0D //X 15:8 bits

#define DATA_10 0x0E //Y 7:0 bits
#define DATA_11 0x0F //Y 15:8 bits

#define DATA_12 0x10 //Z 7:0 bits
#define DATA_13 0x11 //Z 15:8 bits
    
// Accelorometer data registers
#define DATA_14 0x12 //X 7:0 bits
#define DATA_15 0x13 //X 15:8 bitss

#define DATA_16 0x14 //Y 7:0 bits
#define DATA_17 0x15 //Y 15:8 bits

#define DATA_18 0x16 //Z 7:0 bits
#define DATA_19 0x17 //Z 15:8 bits


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

  float _GYRO_X;
  float _GYRO_Y;
  float _GYRO_Z;

  float _ACC_X;
  float _ACC_Y;
  float _ACC_Z;

  char acc[6];
  char gyro[6];
  uint8_t _PMU_STATUS;


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
          const char* dev_name, const char* prefix_path):
           I2CDevice(address, i2c_bus, nh, dev_index, dev_name, prefix_path),
            _pub_imu(this->getTopicName(), &(this->_msg_chip_id))
  {
    this->getNodeHandle()->advertise(_pub_imu);
  }
#else
  BMI_160(int address, I2CBus& i2c_bus, uint8_t dev_index)
    : I2CDevice(address, i2c_bus, dev_index)
  {
  }
#endif

  // DESTRUCTORS
  virtual ~BMI_160(){}

  // GETTERS
  float getGyroX(){
    return _GYRO_X;
  }
  float getGyroY(){
    return _GYRO_Y;
  }
  float getGyroZ(){
    return _GYRO_Z;
  }

  float getAccX(){
    return _ACC_X;
  }
  float getAccY(){
    return _ACC_Y;
  }
  float getAccZ(){
    return _ACC_Z;
  }
  uint8_t getPMUstatus(){
    return _PMU_STATUS;
  }
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


  bool readGyro(){return true;}

  // bool readAcc(){
  // }


  // bool updateIMU(){
  // }

  
};

#endif // BMI_160_H