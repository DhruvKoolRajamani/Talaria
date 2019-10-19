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
class BMI_160 : public I2CDevice
{
private:

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
  
  BMI_160(int address, uint8_t bus_Id, uint8_t dev_index)
      : I2CDevice(address, bus_Id, dev_index){}

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



  void initialize(){
    char soft_reset = SOFT_RESET;
    char acc_mode = ACC_NORMAL_MODE;
    writeByteStream(CMD_ADDRESS, &soft_reset, (int)sizeof(soft_reset));
    writeByteStream(CMD_ADDRESS, &acc_mode, (int)sizeof(acc_mode));
    char pmu;
    readByteStream(PMU_STATUS, &pmu, (int)sizeof(pmu));
    _PMU_STATUS = (uint8_t) pmu;
  }


  bool readGyro(){
    char *gyro_ptr = gyro;
    for(uint8_t data_addr = DATA_8; data_addr <= DATA_13; ++data_addr){
      if(readByteStream(data_addr, gyro_ptr, (int)sizeof(gyro_ptr)) != 0){
        return false;
      }
      gyro_ptr++;
    }
    _GYRO_X = (uint8_t)gyro[1] << 8 | (uint8_t)gyro[0];
    // _GYRO_Y = (uint8_t)gyro[3] << 8 | (uint8_t)gyro[2];
    // _GYRO_Z = (uint8_t)gyro[5] << 8 | (uint8_t)gyro[4];

    return true;
  }

  bool readAcc(){
    
    char *acc_ptr = acc;
    for(uint8_t data_addr = DATA_14; data_addr <= DATA_19; ++data_addr){
      if(readByteStream(data_addr, acc_ptr, (int)sizeof(acc_ptr)) != 0){
        return false;
      }
      acc_ptr++;
    }
    _ACC_X = (uint8_t)acc[1] << 8 | (uint8_t)acc[0];
    // _ACC_Y = (uint8_t)acc[3] << 8 | (uint8_t)acc[2];
    // _ACC_Z = (uint8_t)acc[5] << 8 | (uint8_t)acc[4];

    return true;
  }


  bool updateIMU(){
    if (readGyro() && readAcc()){
      return true;
    }

    return false;
  }

  
};

#endif // BMI_160_H