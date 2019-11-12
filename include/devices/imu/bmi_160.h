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
#include "imu_msgs/bmi160.h"
#endif


class BMI_160 : public I2CDevice
{
private:
#ifndef DISABLE_ROS
  ros::Publisher _pub_imu;
  // Create a custom message for IMU but for now using standard
  // sensor_msgs::Imu _msg_imu;
  // std_msgs::Float32 _msg_imu; // 2 float32 arrays
  imu_msgs::bmi160 _imu_msg;
  std_msgs::Byte _msg_chip_id;
#endif

  float _GYRO_X;
  float _GYRO_Y;
  float _GYRO_Z;

  float _ACC_X;
  float _ACC_Y;
  float _ACC_Z;

  uint8_t acc[6];
  uint8_t gyro[6];
  uint8_t _PMU_STATUS;
  float _GYRO[3];
  float _ACC[3];
protected:
public:
  // CONSTRUCTORS
  enum REGISTER_ADDRESS
  {
    CHIP_ID = 0x00,

    ERR_REG = 0x02,
    PMU_STATUS = 0x03,

    DATA_0 = 0x04,
    DATA_1 = 0x05,
    DATA_2 = 0x06,
    DATA_3 = 0x07,
    DATA_4 = 0x08,
    DATA_5 = 0x09, 
    DATA_6 = 0x0A, 
    DATA_7 = 0x0B, 


    // Gyroscope data registers
    DATA_8 = 0x0C,  // X 7:0 bits
    DATA_9 = 0x0D,  // X 15:8 bits

    DATA_10 = 0x0E,  // Y 7:0 bits
    DATA_11 = 0x0F,  // Y 15:8 bits

    DATA_12 = 0x10,  // Z 7:0 bits
    DATA_13 = 0x11,  // Z 15:8 bits

    // Accelorometer data registers
    DATA_14 = 0x12,  // X 7:0 bits
    DATA_15 = 0x13,  // X 15:8 bitss

    DATA_16 = 0x14,  // Y 7:0 bits
    DATA_17 = 0x15,  // Y 15:8 bits

    DATA_18 = 0x16,  // Z 7:0 bits
    DATA_19 = 0x17,  // Z 15:8 bits
    
    SENSORTIME_0 = 0x18,
    SENSORTIME_1 = 0x19,
    SENSORTIME_2 = 0x1A,

    STATUS = 0x1B,

    INT_STATUS_0 = 0x1C,
    INT_STATUS_1 = 0x1D,
    INT_STATUS_2 = 0x1E,
    INT_STATUS_3 = 0x1F,

    TEMPERATURE_0 = 0x20,
    TEMPERATURE_1 = 0x21,

    FIFO_LENGTH_0 = 0x22,
    FIFO_LENGTH_1 = 0x23,
    FIFO_DATA = 0x24,

    ACC_CONF = 0x40,
    ACC_RANGE = 0x41,
    GYR_CONF = 0x42,
    GYR_RANGE = 0x43,
    MAG_CONF = 0x44,

    FIFO_DOWNS = 0x45,
    FIFO_CONFIG_0 = 0x46,
    FIFO_CONFIG_1 = 0x47,

    MAG_IF_0 = 0x4B,
    MAG_IF_1 = 0x4C,
    MAG_IF_2 = 0x4D,
    MAG_IF_3 = 0x4E,
    MAG_IF_4 = 0x4F,

    INT_EN_0 = 0x50,
    INT_EN_1 = 0x51,
    INT_EN_2 = 0x52,


    CMD_ADDRESS = 0x7E
  };
  char gyr_mode_normal = 0x15;
  char acc_mode_normal = 0x11;

  char SOFT_RESET = 0xB6;

protected:
public:
#ifndef DISABLE_ROS
  BMI_160(int address, I2CBus& i2c_bus, ros::NodeHandle& nh, uint8_t dev_index,
          const char* dev_name, const char* topic_name)
    : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name)
    , _pub_imu(topic_name, &(this->_imu_msg))
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
  float getGyroX()
  {
    return _GYRO_X;
  }
  float getGyroY()
  {
    return _GYRO_Y;
  }
  float getGyroZ()
  {
    return _GYRO_Z;
  }

  float getAccX()
  {
    return _ACC_X;
  }
  float getAccY()
  {
    return _ACC_Y;
  }
  float getAccZ()
  {
    return _ACC_Z;
  }
  uint8_t getPMUstatus()
  {
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

      writeRegister(CMD_ADDRESS, &SOFT_RESET, (int)(sizeof(SOFT_RESET)));
      wait_ms(1);
      writeRegister(CMD_ADDRESS, &gyr_mode_normal, (int)sizeof(gyr_mode_normal));
      wait_ms(1);
      writeRegister(CMD_ADDRESS, &acc_mode_normal, (int)sizeof(acc_mode_normal));
      wait_ms(1);

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

    this->readGyro();

#ifndef DISABLE_ROS
    _imu_msg.gyro.data = &_GYRO;
    _imu_msg.acc.data = &_ACC;

    _pub_imu.publish(&(this->_imu_msg));
#endif
  }

  bool readGyro()
  {
    uint8_t* gyro_ptr = gyro;
    for (uint8_t data_addr = REGISTER_ADDRESS::DATA_8;
         data_addr <= REGISTER_ADDRESS::DATA_13; ++data_addr)
    {
      if (!readRegister(data_addr, gyro_ptr, (int)sizeof(gyro_ptr)))
      {
        return false;
      }

      gyro_ptr++;

      _GYRO_X = (uint8_t)gyro[1] << 8 | (uint8_t)gyro[0];
      _GYRO_Y = (uint8_t)gyro[3] << 8 | (uint8_t)gyro[2];
      _GYRO_Z = (uint8_t)gyro[5] << 8 | (uint8_t)gyro[4];
    }
    return true;
  }
  
    bool readAcc()
    {
      uint8_t* acc_ptr = acc;
      for (uint8_t data_addr = REGISTER_ADDRESS::DATA_14;
           data_addr <= REGISTER_ADDRESS::DATA_19; ++data_addr)
      {
        if (!readRegister(data_addr, acc_ptr, (int)sizeof(acc_ptr)))
        {
          return false;
        }
        acc_ptr++;
      }
      _ACC_X = (uint8_t)acc[1] << 8 | (uint8_t)acc[0];
      _ACC_Y = (uint8_t)acc[3] << 8 | (uint8_t)acc[2];
      _ACC_Z = (uint8_t)acc[5] << 8 | (uint8_t)acc[4];

      return true;
    }

    bool updateIMU()
    {
      if (readGyro() && readAcc())
      {
        _GYRO[0] = _GYRO_X;
        _GYRO[1] = _GYRO_Y;
        _GYRO[2] = _GYRO_Z;

        _ACC[0] = _ACC_X;
        _ACC[1] = _ACC_Y;
        _ACC[2] = _ACC_Z;
        return true;
      }

      return false;
    }
  };
#endif  // BMI_160_H