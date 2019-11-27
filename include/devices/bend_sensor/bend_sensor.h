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
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PinName _reset_pin;
#else
  int _reset_pin;
#endif
  static const int BEND_SENSOR_TRANSFER_SIZE = 3;
#ifndef DISABLE_ROS
  ros::Publisher _pub_bend_sensor;
  bend_sensor_msg::bend_sensor _msg_bend_sensor;
#endif
  uint8_t _data_ready;
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  float _bend_angle = -numeric_limits<float>::max();
  float _stretch_value = -numeric_limits<float>::max();
#else
  float _bend_angle = -99999.0f;
  float _stretch_value = -99999.0f;
#endif

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
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name,
             PinName reset_pin, int refresh_rate = 1);
#else
  BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
             uint8_t dev_index, const char* dev_name, const char* topic_name,
             int reset_pin, int refresh_rate = 1);
#endif
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index, PinName reset_pin,
             int refresh_rate);
#else
  BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index, int reset_pin,
             int refresh_rate);
#endif
#endif

  // DESTRUCTORS
  virtual ~BendSensor();

  // GETTERS

  // SETTERS

  // METHODS
  /**
   * @brief Initialize sensor and run all calibration and setup commands.
   *
   * @return true
   * @return false
   */
  bool initialize() override;

  bool enableStretchValues(bool enable = false);

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  bool ping(int chip_id_reg_address = BEND_SENSOR_GET_DEV_ID,
            int delay_ms = 2) override;
#else

  bool ping(uint8_t chip_id_reg_address = BEND_SENSOR_GET_DEV_ID,
            int delay_ms = 2) override;
#endif

  void update(int loop_counter = 1);

  // Convert two bytes of buffer[] to int16
  inline int16_t decodeInt16(const uint8_t* p_encoded_data);

  /*
   *  Second order Infinite impulse response low pass filter. Sample freqency
   * 100 Hz. Cutoff freqency 20 Hz.
   */
  void signalFilter(float* sample);

  void deadzoneFilter(float* sample);

  // Checks to see if new data is available
  // This must be called regularly to update getX and getY functions
  // Returns true if a new sample is received
  // The sample is then parsed and run through the filters
  bool readData();

  // Takes the data from the latest sample and loads it into the filters
  void processNewData();

  void disable();

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
  bool calibrate(uint8_t ads_calibration_step, uint8_t degrees);

  // Delete the current calibration values from non-volatile memory and restore
  // the factory calibration
  bool clearCalibration();

  /**
   * @brief Reset the Angular Displacement Sensor
   */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  void reset(PinName pin, int delay_ms = 100) override;
#else
  void reset(int pin, int delay_ms = 100) override;
#endif

  /**
   * @brief Places ADS in polled or sleep mode
   * @param  run true if activating ADS, false is putting in suspend mode
   * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
   */
  bool beginPollingData(bool poll = true);

  /**
   * @brief Places ADS in free run or sleep mode
   * @param  run true if activating ADS, false is putting in suspend mode
   * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
   */
  bool beginReadingData(bool run = true);

  /**
   * @brief Updates the I2C address of the selected ADS. The default address
   *      is 0x13. Use this function to program an ADS to allow multiple
   *      devices on the same I2C bus.
   * @param device  device number of the device that is being updated
   * @param address new address of the ADS
   * @return  True if successful false if failed
   */

  bool setAddress(uint8_t newAddress);

  /**
   * @brief Sets the sample rate of the ADS in free run mode
   * @param  sample rate
   * @return  True if successful, false if failed
   */
  bool setSampleRate(uint16_t sps);

  // Send command to initiate soft reset
  bool softReset();
};

#endif  // BEND_SENSOR_H