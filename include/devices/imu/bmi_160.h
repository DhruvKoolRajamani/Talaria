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

    POLLING_ID = 0x12
  } reg_addr;

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

  // SETTERS

  // METHODS
};

#endif  // BMI_160_H