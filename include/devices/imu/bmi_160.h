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

class BMI_160 : public I2CDevice
{
private:
 
  enum REGISTER_ADDRESS
  {
    CHIP_ID = 0x68,

    // Gyroscope data registers
    DATA_8 = 0x0C, //X 7:0 bits
    DATA_9 = 0x0D, //X 15:8 bits

    DATA_10 = 0x0E, //Y 7:0 bits
    DATA_11 = 0x0F, //Y 15:8 bits

    DATA_12 = 0x10, //Z 7:0 bits
    DATA_13 = 0x11, //Z 15:8 bits
    
    // Accelorometer data registers
    DATA_14 = 0x12, //X 7:0 bits
    DATA_15 = 0x13, //X 15:8 bitss

    DATA_16 = 0x14, //Y 7:0 bits
    DATA_17 = 0x15, //Y 15:8 bits

    DATA_18 = 0x16, //Z 7:0 bits
    DATA_19 = 0x17, //Z 15:8 bits



    POLLING_ID = 0x12
  } reg_addr;

  float _GYRO_X;
  float _GYRO_Y;
  float _GYRO_Z;

  float _ACC_X;
  float _ACC_Y;
  float _ACC_Z;

  char acc[6];
  char gyro[6];

protected:
public:
  // CONSTRUCTORS
  
  BMI_160(int address, uint8_t bus_Id, uint8_t dev_index)
      : I2CDevice(address, bus_Id, dev_index)
  {
  }

  // DESTRUCTORS
  virtual ~BMI_160()
  {
  }

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
  // SETTERS
  




  // METHODS
  
  bool readGyro(){
    char *gyro_ptr = gyro;
    for(uint8_t data_addr = REGISTER_ADDRESS::DATA_8; data_addr <= REGISTER_ADDRESS::DATA_13; ++data_addr){
      if(!readByteStream(data_addr, gyro_ptr, (int)sizeof(gyro_ptr))){
        return false;
      }
      gyro_ptr++;
    }
    _GYRO_X = (uint8_t)gyro[1] << 8 | (uint8_t)gyro[0];
    _GYRO_Y = (uint8_t)gyro[3] << 8 | (uint8_t)gyro[2];
    _GYRO_Z = (uint8_t)gyro[5] << 8 | (uint8_t)gyro[4];

    return true;
  }

  bool readAcc(){
    
    char *acc_ptr = acc;
    for(uint8_t data_addr = REGISTER_ADDRESS::DATA_14; data_addr <= REGISTER_ADDRESS::DATA_19; ++data_addr){
      if(!readByteStream(data_addr, acc_ptr, (int)sizeof(acc_ptr))){
        return false;
      }
      acc_ptr++;
    }
    _ACC_X = (uint8_t)acc[1] << 8 | (uint8_t)acc[0];
    _ACC_Y = (uint8_t)acc[3] << 8 | (uint8_t)acc[2];
    _ACC_Z = (uint8_t)acc[5] << 8 | (uint8_t)acc[4];

    return true;
  }


  bool updateIMU(){
    if (readGyro() && readAcc()){
      return true;
    }

    return false;
  }
  // reading and writing
    // to specific registers
  // initializing
  // long polling
  // just raw data and seperate for filtering and putting into objects of data 
  // using bmi filter data


  // wednesday raw data
};

#endif // BMI_160_H