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
#include <math.h>

#ifndef DISABLE_ROS
#include "sensor_msgs/Imu.h"

// Add header file for custom ros message for bend sensor

#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
#include "bend_sensor_msg/bend_sensor.h"
#endif

class BendSensor : public I2CDevice
{
private:
  PinName _reset_pin;
  static const int BEND_SENSOR_TRANSFER_SIZE = 3;
#ifndef DISABLE_ROS
  ros::Publisher _pub_bend_sensor;
  bend_sensor_msg::bend_sensor _msg_bend_sensor;
#endif
  uint8_t _data_ready;
  float _bend_angle = -numeric_limits<float>::max();
  float _stretch_value = -numeric_limits<float>::max();

  float _r_mcp_joint = 100.2f;
  float _r_pip_joint = 90.6f;

  float _mcp_joint = 0.0f;
  float _pip_joint = 0.0f;

  enum COMMAND_REGISTERS
  {
    BEND_SENSOR_RUN = 0,       // Place ADS in freerun interrupt mode or standby
    BEND_SENSOR_SPS,           // Update SPS on ADS in freerun interrupt mode
    BEND_SENSOR_RESET,         // Software reset command
    BEND_SENSOR_DFU,           // Reset ADS into bootloader for firmware update
    BEND_SENSOR_SET_ADDRESS,   // Update the I2C address on the ADS
    BEND_SENSOR_POLLED_MODE,   // Place ADS in polled mode or standby
    BEND_SENSOR_GET_FW_VER,    // Get firwmare version on the ADS
    BEND_SENSOR_CALIBRATE,     // Calibration command, see
                               // BEND_SENSOR_CALIBRATION_STEP_T
    BEND_SENSOR_READ_STRETCH,  // Enable simultaneous bend and stretch
                               // measurements
    BEND_SENSOR_SHUTDOWN,   // Shuts ADS down, lowest power mode, requires reset
                            // to wake
    BEND_SENSOR_GET_DEV_ID  // Gets unique device ID for ADS sensor, see
                            // BEND_SENSOR_DEV_IDS_T
  } BEND_SENSOR_COMMAND_T;

  enum PACKET_DEF
  {
    BEND_SENSOR_SAMPLE = 0,
    BEND_SENSOR_FW_VER,
    BEND_SENSOR_DEV_ID,
    BEND_SENSOR_STRETCH_SAMPLE  // Packet read is a stretch sample
  } BEND_SENSOR_PACKET_T;

  enum CALIBRATION_REGISTERS
  {
    BEND_SENSOR_CALIBRATE_FIRST = 0,
    BEND_SENSOR_CALIBRATE_FLAT,
    BEND_SENSOR_CALIBRATE_PERP,
    BEND_SENSOR_CALIBRATE_CLEAR
  } BEND_SENSOR_CALIBRATION_STEP_T;

  // Default I2C addresses
  enum DEFAULT_ADDRESSES
  {
    BEND_SENSOR_ONE_AXIS_ADDRESS = 0x12,
    BEND_SENSOR_TWO_AXIS_ADDRESS = 0x13
  } BEND_SENSOR_DEFAULT_ADDRESS;

  // There are two types of ADS - single axis and two axis
  enum DEVICE_IDS
  {
    BEND_SENSOR_ONE_AXIS = 1,
    BEND_SENSOR_TWO_AXIS = 2,
  } BEND_SENSOR_DEV_IDS_T;

  // Available output rates
  enum OUTPUT_RATES
  {
    BEND_SENSOR_1_HZ = 16384,
    BEND_SENSOR_10_HZ = 1638,
    BEND_SENSOR_20_HZ = 819,
    BEND_SENSOR_50_HZ = 327,
    BEND_SENSOR_100_HZ = 163,
    BEND_SENSOR_200_HZ = 81,
    BEND_SENSOR_333_HZ = 49,
    BEND_SENSOR_500_HZ = 32,
  } BEND_SENSOR_SPS_T;

protected:
public:
  // CONSTRUCTORS
#ifndef DISABLE_ROS
  BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name,
             PinName reset_pin)
    : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name)
    , _reset_pin(reset_pin)
    , _pub_bend_sensor(topic_name, &(this->_msg_bend_sensor))
  {
    nh.advertise(_pub_bend_sensor);
  }
#else
  BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index)
    : I2CDevice(address, i2c_bus, dev_index), _reset_pin(reset_pin)
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
    // Perform calibration
    this->setConfiguredStatus(true);

    // this->reset(_reset_pin, 2000);

    if (!this->ping())
    {
#ifndef DISABLE_ROS
      _msg_bend_sensor.debug.data = "Ping failed";
#endif
      _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
      return false;
    }

    if (!this->setSampleRate(BEND_SENSOR_50_HZ))
    {
#ifndef DISABLE_ROS
      _msg_bend_sensor.debug.data = "Set Sensor Rate Nack";
#endif
      _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
      return false;
    }

    wait_ms(2);

    if (!this->enableStretchValues(true))
    {
#ifndef DISABLE_ROS
      _msg_bend_sensor.debug.data = "Stretch Enable Nack";
#endif
      _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
      return false;
    }

    if (!this->beginPollingData(true))
    {
#ifndef DISABLE_ROS
      _msg_bend_sensor.debug.data = "Begin Polling Nack";
#endif
      _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
      return false;
    }

    wait_ms(10);

    // this->ping();
    // this->setSampleRate(BEND_SENSOR_100_HZ);
    // wait_ms(2);
    // this->enableStretchValues(true);
    // this->beginPollingData(true);

    return true;
  }

  bool enableStretchValues(bool enable = false)
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE] = { BEND_SENSOR_READ_STRETCH,
                                                  enable, 0 };

    return this->writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  bool ping(int chip_id_reg_address = BEND_SENSOR_GET_DEV_ID,
            int delay_ms = 2) override
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];
    if (readRegister(chip_id_reg_address, buffer, BEND_SENSOR_TRANSFER_SIZE,
                     false, delay_ms))
    {
      setHealthStatus(true);
      // setConfiguredStatus(true);
      setEnabledStatus(true);
      this->setChipId(buffer[0]);

#ifndef DISABLE_ROS
      _msg_bend_sensor.chip_id.data = this->getChipId();
      _msg_bend_sensor.mcp_joint.data = this->_mcp_joint;
      _msg_bend_sensor.pip_joint.data = this->_pip_joint;
#ifndef DISABLE_DIAGNOSTICS
      _diagnostic_chip_id.value = (char*)buffer;
      this->setDiagnosticsData(_diagnostic_chip_id);
#endif
#else
      printf("Chip Id is: %x", this->getChipId());
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

    // this->_bend_angle = -numeric_limits<float>::max();
    // this->_stretch_value = -numeric_limits<float>::max();

    this->readData();

#ifndef DISABLE_ROS
    if (this->_data_ready == BEND_SENSOR_SAMPLE)
    {
      processNewData();
      _msg_bend_sensor.chip_id.data = this->getChipId();
      _msg_bend_sensor.mcp_joint.data = this->_mcp_joint;
      _msg_bend_sensor.pip_joint.data = this->_pip_joint;
      _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
    }
#else
// Serialize this message
// printf("Bend Data: %f", _data[0]);
#endif
  }

  inline int16_t decodeInt16(const uint8_t* p_encoded_data)  // Convert two
                                                             // bytes of
                                                             // buffer[] to
                                                             // int16
  {
    return ((((uint16_t)(p_encoded_data)[0])) |
            (((int16_t)(p_encoded_data)[1]) << 8));
  }

  void signalFilter(float& sample)  // Low pass IIR filter
  {
    float filter_samples[6];
    filter_samples[5] = filter_samples[4];
    filter_samples[4] = filter_samples[3];
    filter_samples[3] = (float)sample;
    filter_samples[2] = filter_samples[1];
    filter_samples[1] = filter_samples[0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[0] =
        filter_samples[1] * (0.36952737735124147f) -
        0.19581571265583314f * filter_samples[2] +
        0.20657208382614792f *
            (filter_samples[3] + 2 * filter_samples[4] + filter_samples[5]);

    sample = filter_samples[0];
  }

  void deadzoneFilter(float& sample)  // Deadzone filter
  {
    float prev_sample;
    float dead_zone = 0.5f;

    if (fabs(sample - prev_sample) > dead_zone)
      prev_sample = sample;
    else
      sample = prev_sample;
  }

  // Checks to see if new data is available
  // This must be called regularly to update getX and getY functions
  // Returns true if a new sample is received
  // The sample is then parsed and run through the filters
  bool readData()
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE] = { 0, 0, 0 };
    int16_t temp;

    if (readBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE))
    {
      this->_data_ready = buffer[0];
      if (buffer[0] == BEND_SENSOR_SAMPLE)
      {
        temp = decodeInt16(&buffer[1]);
        this->_bend_angle =
            (float)((float)temp / 64.0f) * ((float)M_PI / 180.0f);
      }
      else if (buffer[0] == BEND_SENSOR_STRETCH_SAMPLE)
      {
        temp = decodeInt16(&buffer[1]);
        this->_stretch_value = (float)temp / 64.0f;
      }
      else
      {
        _msg_bend_sensor.debug.data = "ReadData buffer incorrect";
        return false;
      }
      // _msg_bend_sensor.debug.data = "ReadData Success";
      return true;
    }
    _msg_bend_sensor.debug.data = "ReadData Fail";
    return false;
  }

  // Takes the data from the latest sample and loads it into the filters
  void processNewData()
  {
    // Low pass IIR filter
    // signalFilter(this->_bend_angle);
    // signalFilter(this->_stretch_value);

    // // Deadzone filter
    // deadzoneFilter(this->_bend_angle);
    // deadzoneFilter(this->_stretch_value);

    // currentSample 0 and 1 are now ready to be read

    // bend_angle = theta1 + theta2
    // stretch = r1*theta1 + r2*theta2
    // A = [1 ,  1]
    //     [r1, r2]
    //
    // B = [bend_angle ]
    //     [stretch_val]
    //
    // x = [theta1]
    //     [theta2]
    //
    // Solve:
    // A*x = B
    //
    float A[2][2], B[2], C[2], X[2] = { 0, 0 };
    A[0][0] = 1;
    A[0][1] = 1;
    A[1][0] = this->_r_mcp_joint;
    A[1][1] = this->_r_pip_joint;

    B[0] = this->_bend_angle;
    B[1] = this->_stretch_value;

    for (int i = 0; i < 2; i++)
    {
      C[i] = B[i];
      for (int j = 0; j < 2; j++)
      {
        if (i != j)
        {
          C[i] -= A[i][j] * X[j];
        }
      }
    }
    for (int i = 0; i < 2; i++)
    {
      X[i] = C[i] / A[i][i];
    }

    this->_mcp_joint = X[0] * 180.0f / (float)M_PI;
    this->_pip_joint = X[1] * 180.0f / (float)M_PI;
  }

  void disable()
  {
    Device::disable();
    char buffer[BEND_SENSOR_TRANSFER_SIZE] = { BEND_SENSOR_SHUTDOWN, 0, 0 };

    writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  /**
   * @brief Calibrates two axis ADS. BEND_SENSOR_CALIBRATE_FIRST must be at 0
   * degrees on both AXES. BEND_SENSOR_CALIBRATE_FLAT can be at 45 - 255
   * degrees, recommended 90 degrees. When calibrating the flat axis the
   * perpendicular axis should be at 0 degrees. BEND_SENSOR_CALIBRATE_PERP can
   * be at 45 - 255 degrees, recommended 90 degrees. When calibrating the
   * perpendicular axis the flat axis should be at 0 degrees Note: The flat axis
   * is sample[0] (axis 0) perp axis is sample[1] (axis 1) from
   * ads_data_callback
   * @param ads_calibration_step  BEND_SENSOR_CALIBRATE_STEP_T to perform
   * @param degrees uint8_t angle at which sensor is bent when performing
   *         BEND_SENSOR_CALIBRATE_FLAT, and BEND_SENSOR_CALIBRATE_PERP
   * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO or
   * BEND_SENSOR_BAD_PARAM if failed
   */
  bool calibrate(uint8_t ads_calibration_step, uint8_t degrees)
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_CALIBRATE;
    buffer[1] = ads_calibration_step;
    buffer[2] = degrees;

    return writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  // Delete the current calibration values from non-volatile memory and restore
  // the factory calibration
  bool clearCalibration()
  {
    calibrate(BEND_SENSOR_CALIBRATE_CLEAR, 0);
  }

  /**
   * @brief Reset the Angular Displacement Sensor
   */
  void reset(PinName pin, int delay_ms = 100) override
  {
    // Configure reset line as an output
    this->setPinState(pin, false);
    wait_ms(10);
    this->setPinState(pin, true);
    wait_ms(delay_ms);
  }

  /**
   * @brief Places ADS in polled or sleep mode
   * @param  run true if activating ADS, false is putting in suspend mode
   * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
   */
  bool beginPollingData(bool poll = true)
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_POLLED_MODE;
    buffer[1] = poll;

    return writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  /**
   * @brief Places ADS in free run or sleep mode
   * @param  run true if activating ADS, false is putting in suspend mode
   * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
   */
  bool beginReadingData(bool run = true)
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_RUN;
    buffer[1] = run;

    return writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  /**
   * @brief Updates the I2C address of the selected ADS. The default address
   *      is 0x13. Use this function to program an ADS to allow multiple
   *      devices on the same I2C bus.
   * @param device  device number of the device that is being updated
   * @param address new address of the ADS
   * @return  True if successful false if failed
   */
  bool setAddress(uint8_t newAddress)
  {
    if (newAddress < 0x08 || newAddress > 0x77)
      return false;  // Address is out of bounds

    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_SET_ADDRESS;
    buffer[1] = newAddress;

    if (writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE) == false)
      return false;

    this->setDeviceAddress(newAddress);  // Update address only after success

    return true;
  }

  /**
   * @brief Sets the sample rate of the ADS in free run mode
   * @param  sample rate
   * @return  True if successful, false if failed
   */
  bool setSampleRate(uint16_t sps)
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_SPS;

    // SPS is loaded little endian
    buffer[1] = (uint8_t)((sps & 0x00FF) >> 0);
    buffer[2] = (uint8_t)((sps & 0xFF00) >> 8);

    return writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }

  // Send command to initiate soft reset
  bool softReset()
  {
    uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

    buffer[0] = BEND_SENSOR_RESET;

    return writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
  }
};

#endif  // BEND_SENSOR_H