/**
 * @file adis_16470.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-03-11
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef ADIS_16470
#define ADIS_16470

#include "devices/base/spi_device.h"

#ifndef DISABLE_ROS
#include "imu_msg/imu.h"
#include "std_msgs/Byte.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"
#endif

// MOVE ALL THESE TO ENUM
#define FLASH_CNT 0x00      // Flash memory write count
#define DIAG_STAT 0x02      // Diagnostic and operational status
#define X_GYRO_LOW 0x04     // X-axis gyroscope output, lword
#define X_GYRO_OUT 0x06     // X-axis gyroscope output, uword
#define Y_GYRO_LOW 0x08     // Y-axis gyroscope output, lword
#define Y_GYRO_OUT 0x0A     // Y-axis gyroscope output, uword
#define Z_GYRO_LOW 0x0C     // Z-axis gyroscope output, lword
#define Z_GYRO_OUT 0x0E     // Z-axis gyroscope output, uword
#define X_ACCL_LOW 0x10     // X-axis accelerometer output, lword
#define X_ACCL_OUT 0x12     // X-axis accelerometer output, uword
#define Y_ACCL_LOW 0x14     // Y-axis accelerometer output, lword
#define Y_ACCL_OUT 0x16     // Y-axis accelerometer output, uword
#define Z_ACCL_LOW 0x18     // Z-axis accelerometer output, lword
#define Z_ACCL_OUT 0x1A     // Z-axis accelerometer output, uword
#define TEMP_OUT 0x1C       // Temperature output (internal, not calibrated)
#define TIME_STAMP 0x1E     // PPS mode time stamp
#define X_DELTANG_LOW 0x24  // X-axis delta angle output, lword
#define X_DELTANG_OUT 0x26  // X-axis delta angle output, uword
#define Y_DELTANG_LOW 0x28  // Y-axis delta angle output, lword
#define Y_DELTANG_OUT 0x2A  // Y-axis delta angle output, uword
#define Z_DELTANG_LOW 0x2C  // Z-axis delta angle output, lword
#define Z_DELTANG_OUT 0x2E  // Z-axis delta angle output, uword
#define X_DELTVEL_LOW 0x30  // X-axis delta velocity output, lword
#define X_DELTVEL_OUT 0x32  // X-axis delta velocity output, uword
#define Y_DELTVEL_LOW 0x34  // Y-axis delta velocity output, lword
#define Y_DELTVEL_OUT 0x36  // Y-axis delta velocity output, uword
#define Z_DELTVEL_LOW 0x38  // Z-axis delta velocity output, lword
#define Z_DELTVEL_OUT 0x3A  // Z-axis delta velocity output, uword
#define XG_BIAS_LOW 0x40    // X-axis gyroscope bias offset corr, lword
#define XG_BIAS_HIGH 0x42   // X-axis gyroscope bias offset corr, uword
#define YG_BIAS_LOW 0x44    // Y-axis gyroscope bias offset corr, lword
#define YG_BIAS_HIGH 0x46   // Y-axis gyroscope bias offset corr, uword
#define ZG_BIAS_LOW 0x48    // Z-axis gyroscope bias offset corr, lword
#define ZG_BIAS_HIGH 0x4A   // Z-axis gyroscope bias offset corr, uword
#define XA_BIAS_LOW 0x4C    // X-axis accelerometer bias offset corr, lword
#define XA_BIAS_HIGH 0x4E   // X-axis accelerometer bias offset corr, uword
#define YA_BIAS_LOW 0x50    // Y-axis accelerometer bias offset corr, lword
#define YA_BIAS_HIGH 0x52   // Y-axis accelerometer bias offset corr, uword
#define ZA_BIAS_LOW 0x54    // Z-axis accelerometer bias offset corr, lword
#define ZA_BIAS_HIGH 0x56   // Z-axis accelerometer bias offset corr, uword
#define FILT_CTRL 0x5C      // Filter control
#define MSC_CTRL 0x60       // Miscellaneous control
#define UP_SCALE 0x62       // Clock scale factor, PPS mode
#define DEC_RATE 0x64       // Decimation rate control (output data rate)
#define NULL_CFG 0x66       // Auto-null configuration control
#define GLOB_CMD 0x68       // Global commands
#define FIRM_REV 0x6C       // Firmware revision
#define FIRM_DM 0x6E        // Firmware revision date, month and day
#define FIRM_Y 0x70         // Firmware revision date, year
#define PROD_ID 0x72        // Product identification
#define SERIAL_NUM 0x74     // Serial number (relative to assembly lot)
#define USER_SCR1 0x76      // User scratch register 1
#define USER_SCR2 0x78      // User scratch register 2
#define USER_SCR3 0x7A      // User scratch register 3
#define FLSHCNT_LOW 0x7C    // Flash update count, lword
#define FLSHCNT_HIGH 0x7E   // Flash update count, uword

class ADIS16470 : public SPIDevice
{
private:
#ifndef DISABLE_ROS
  ros::Publisher _pub_imu;
  std_msgs::String debugMsg;
  ros::Publisher _pub_debug =
      ros::Publisher("/devices/hand/imu/debug", &debugMsg);
  // Create a custom message for IMU but for now using standard
  // sensor_msgs::Imu _msg_imu;
  imu_msg::imu _msg_imu;
  std_msgs::Byte _msg_chip_id;
#endif

  uint16_t* _imu_burst = nullptr;
  int16_t burstChecksum = 0;

  // Accelerometer
  float _accl[3] = { 0., 0., 0. };

  // Gyro
  float _gyro[3] = { 0., 0., 0. };

  // Control registers
  int16_t MSC = 0;
  int16_t FLTR = 0;
  int16_t DECR = 0;
  int16_t _cur_bias_est_time = 0x070A;

  int16_t FIRM_REV_VAL = 0;
  int16_t PROD_ID_VAL = 0;
  int16_t SER_NO_VAL = 0;
  bool _id_check_pass = false;

  // Temperature
  float TEMPS = 0;

  // Data Ready interrupt
  InterruptIn _dr_interrupt;

protected:
public:
  // CONSTRUCTORS

  // enum REGISTER_ADDRESS
  // {
  //   CHIP_ID = 0x00,
  //   POLLING_ID = 0x12
  // };

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  /**
   * @brief Construct a new ADIS16470 object
   *
   * @param mosi
   * @param miso
   * @param sclk
   * @param cs
   * @param dr
   * @param rst
   * @param nh
   * @param clock_speed
   * @param dev_index
   * @param dev_name
   * @param topic_name
   * @param refresh_rate
   */
  ADIS16470(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dr,
            PinName rst, ros::NodeHandle& nh, int clock_speed = 1000000,
            uint8_t dev_index = 0, const char* dev_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : SPIDevice(mosi, miso, sclk, cs, dr, rst, nh, clock_speed, dev_index,
                dev_name, topic_name, refresh_rate)
    , _pub_imu(topic_name, &(this->_msg_imu))
    , _dr_interrupt(dr)
  {
    wait_us(500);
#else
  ADIS16470(int mosi, int miso, int sclk, int cs, int dr, int rst,
            ros::NodeHandle& nh, uint32_t clock_speed = 1000000,
            uint8_t dev_index = 0, const char* dev_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : SPIDevice(mosi, miso, sclk, cs, dr, rst, nh, clock_speed, dev_index,
                dev_name, topic_name, refresh_rate)
    , _pub_imu(topic_name, &(this->_msg_imu))
    , _dr_interrupt(dr)
  {
    delay(500);
#endif
    setIsTopicAdvertised(nh.advertise(_pub_imu));
    nh.advertise(_pub_debug);
    _imu_burst = (uint16_t*)malloc(20 * sizeof(uint16_t));
  }
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  ADIS16470(PinName cs, PinName dr, PinName rst, SPIBus& spi_bus,
            uint8_t dev_index, int refresh_rate = 1)
#else
  ADIS16470(int cs, int dr, int rst, SPIBus& spi_bus, uint8_t dev_index,
            int refresh_rate = 1)
#endif
    : SPIDevice(cs, dr, rst, spi_bus, dev_index, refresh_rate)
  {
  }
#endif

  // DESTRUCTORS
  virtual ~ADIS16470()
  {
    if (_imu_burst) free(_imu_burst);
  }

  // GETTERS

  // SETTERS

  // METHODS

  /**
   * @brief Intiates a burst read from the sensor. Returns a pointer to an array
   * of sensor data.
   *
   * @return uint8_t*
   */
  uint8_t* byteBurst()
  {
    static uint8_t burstdata[20];

// Trigger Burst Read
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    select();
    _spi_bus->write(GLOB_CMD);
    _spi_bus->write(0x00);

    // Read Burst Data
    burstdata[0] = (uint8_t)_spi_bus->write(0x00);  // DIAG_STAT
    burstdata[1] = (uint8_t)_spi_bus->write(0x00);
    burstdata[2] = (uint8_t)_spi_bus->write(0x00);  // XGYRO_OUT
    burstdata[3] = (uint8_t)_spi_bus->write(0x00);
    burstdata[4] = (uint8_t)_spi_bus->write(0x00);  // YGYRO_OUT
    burstdata[5] = (uint8_t)_spi_bus->write(0x00);
    burstdata[6] = (uint8_t)_spi_bus->write(0x00);  // ZGYRO_OUT
    burstdata[7] = (uint8_t)_spi_bus->write(0x00);
    burstdata[8] = (uint8_t)_spi_bus->write(0x00);  // XACCEL_OUT
    burstdata[9] = (uint8_t)_spi_bus->write(0x00);
    burstdata[10] = (uint8_t)_spi_bus->write(0x00);  // YACCEL_OUT
    burstdata[11] = (uint8_t)_spi_bus->write(0x00);
    burstdata[12] = (uint8_t)_spi_bus->write(0x00);  // ZACCEL_OUT
    burstdata[13] = (uint8_t)_spi_bus->write(0x00);
    burstdata[14] = (uint8_t)_spi_bus->write(0x00);  // TEMP_OUT
    burstdata[15] = (uint8_t)_spi_bus->write(0x00);
    burstdata[16] = (uint8_t)_spi_bus->write(0x00);  // TIME_STMP
    burstdata[17] = (uint8_t)_spi_bus->write(0x00);
    burstdata[18] = (uint8_t)_spi_bus->write(0x00);  // CHECKSUM
    burstdata[19] = (uint8_t)_spi_bus->write(0x00);

    deselect();
#else
    digitalWrite(_cs, LOW);
    _spi_bus->transfer(GLOB_CMD);
    _spi_bus->transfer(0x00);

    // Read Burst Data
    burstdata[0] = _spi_bus->transfer(0x00);  // DIAG_STAT
    burstdata[1] = _spi_bus->transfer(0x00);
    burstdata[2] = _spi_bus->transfer(0x00);  // XGYRO_OUT
    burstdata[3] = _spi_bus->transfer(0x00);
    burstdata[4] = _spi_bus->transfer(0x00);  // YGYRO_OUT
    burstdata[5] = _spi_bus->transfer(0x00);
    burstdata[6] = _spi_bus->transfer(0x00);  // ZGYRO_OUT
    burstdata[7] = _spi_bus->transfer(0x00);
    burstdata[8] = _spi_bus->transfer(0x00);  // XACCEL_OUT
    burstdata[9] = _spi_bus->transfer(0x00);
    burstdata[10] = _spi_bus->transfer(0x00);  // YACCEL_OUT
    burstdata[11] = _spi_bus->transfer(0x00);
    burstdata[12] = _spi_bus->transfer(0x00);  // ZACCEL_OUT
    burstdata[13] = _spi_bus->transfer(0x00);
    burstdata[14] = _spi_bus->transfer(0x00);  // TEMP_OUT
    burstdata[15] = _spi_bus->transfer(0x00);
    burstdata[16] = _spi_bus->transfer(0x00);  // TIME_STMP
    burstdata[17] = _spi_bus->transfer(0x00);
    burstdata[18] = _spi_bus->transfer(0x00);  // CHECKSUM
    burstdata[19] = _spi_bus->transfer(0x00);
    digitalWrite(_cs, HIGH);
#endif

    return burstdata;
  }

  /**
   * @brief Intiates a burst read from the sensor. Returns a pointer to an array
   * of sensor data.
   *
   * @return uint16_t*
   */
  uint16_t* wordBurst()
  {
    static uint16_t burstwords[10];

    // Trigger Burst Read

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    select();
    _spi_bus->write(GLOB_CMD);
    _spi_bus->write(0x00);

    burstwords[0] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // DIAG_STA
    burstwords[1] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // XGYRO
    burstwords[2] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // YGYRO
    burstwords[3] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // ZGYRO
    burstwords[4] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // XACCEL
    burstwords[5] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // YACCEL
    burstwords[6] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // ZACCEL
    burstwords[7] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // TEMP_OUT
    burstwords[8] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // TIME_STM
    burstwords[9] = (((uint8_t)_spi_bus->write(0x00) << 8) |
                     ((uint8_t)_spi_bus->write(0x00) & 0xFF));  // CHECKSUM

    deselect();
#else
    digitalWrite(_CS, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0x00);

    // Read Burst Data
    burstwords[0] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // DIAG_STAT
    burstwords[1] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // XGYRO
    burstwords[2] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // YGYRO
    burstwords[3] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // ZGYRO
    burstwords[4] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // XACCEL
    burstwords[5] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // YACCEL
    burstwords[6] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // ZACCEL
    burstwords[7] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // TEMP_OUT
    burstwords[8] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // TIME_STMP
    burstwords[9] =
        ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF));  // CHECKSUM

    digitalWrite(_CS, HIGH);
#endif

    return burstwords;
  }

  /**
   * @brief Calculates checksum based on burst data. Returns the calculated
   * checksum.
   *
   * @param burstArray - array of burst data
   * @return int16_t - signed calculated checksum
   */
  int16_t checksum(uint16_t* burstArray)
  {
    int16_t s = 0;
    for (int i = 0; i < 9; i++)  // Checksum value is not part of the sum!!
    {
      s += (burstArray[i] & 0xFF);         // Count lower byte
      s += ((burstArray[i] >> 8) & 0xFF);  // Count upper byte
    }

    return s;
  }

  /**
   * @brief Converts accelerometer data output from the regRead() function
   *
   * @param sensorData - data output from regRead()
   * @return float - signed/scaled accelerometer in m/s^2
   */
  float accelScale(int16_t sensorData)
  {
    float finalData =
        sensorData * 0.00125;  // Multiply by accel sensitivity (0.00125g/LSB)
    return finalData * 9.80665;
  }

  /**
   * @brief Converts gyro data output from the regRead() function
   *
   * @param sensorData - data output from regRead()
   * @return float - signed/scaled gyroscope in rad/s
   */
  float gyroScale(int16_t sensorData)
  {
    float finalData =
        sensorData * 0.1;  // Multiply by gyro sensitivity (0.1 deg/LSB)
    return finalData * (M_PI / 180.);
  }

  /**
   * @brief Converts temperature data output from the regRead() function
   *
   * @param sensorData - data output from regRead()
   * @return float - signed/scaled temperature in degrees Celcius
   */
  float tempScale(int16_t sensorData)
  {
    float finalData =
        (sensorData * 0.1);  // Multiply by temperature scale (0.1 deg C/LSB)
    return finalData;
  }

  /**
   * @brief Converts integrated angle data output from the regRead() function.
   *
   * @param sensorData - data output from regRead()
   * @return float - signed/scaled delta angle in radians
   */
  float deltaAngleScale(int16_t sensorData)
  {
    float finalData = sensorData * 0.061;  // Multiply by delta angle scale
                                           // (0.061 degrees/LSB)
    return finalData * (M_PI / 180.0);
  }

  /**
   * @brief Converts integrated velocity data output from the regRead() function
   *
   * @param sensorData - data output from regRead()
   * @return float - signed/scaled delta velocity in m/sec
   */
  float deltaVelocityScale(int16_t sensorData)
  {
    float finalData =
        sensorData * 0.01221;  // Multiply by velocity scale (0.01221 m/sec/LSB)
    return finalData;
  }

  /**
   * @brief Checks the Firmware revision id, product id and serial number id
   *
   * @return true   if product id is 0x4056
   * @return false  if product if is NOT 0x4056
   */
  bool checkSensorID()
  {
    FIRM_REV_VAL = ping(FIRM_REV);
    PROD_ID_VAL = ping(PROD_ID);
    SER_NO_VAL = ping(SERIAL_NUM);

    if (PROD_ID_VAL == 0x4056)
    {
      return true;
    }
    _id_check_pass = false;
    return false;
  }

  bool initialize() override
  {
    this->begin();

    // Replace with whoami?
    // if (this->ping(REGISTER_ADDRESS::CHIP_ID))
    if (checkSensorID())
    {
      this->enable();
      this->setHealthStatus(true);
      // Perform calibration
      this->setConfiguredStatus(true);

      _id_check_pass = true;

      writeWord(MSC_CTRL, 0xC1);   // Enable Data Ready, set polarity
      writeWord(FILT_CTRL, 0x04);  // Set digital filter
      writeWord(DEC_RATE, 0x00);   // Disable decimation

      // Read the control registers once to print to screen
      MSC = readWord(MSC_CTRL);
      FLTR = readWord(FILT_CTRL);
      DECR = readWord(DEC_RATE);

      // if (!setBiasEstimationTime()) this->setConfiguredStatus(true);

      _msg_imu.time.data = this->getNodeHandle()->now();
      _msg_imu.accelerometer[0] = _accl[0];
      _msg_imu.accelerometer[1] = _accl[1];
      _msg_imu.accelerometer[2] = _accl[2];
      _msg_imu.gyroscope[0] = _gyro[0];
      _msg_imu.gyroscope[1] = _gyro[1];
      _msg_imu.gyroscope[2] = _gyro[2];
      _msg_imu.control_regs[0] = MSC;
      _msg_imu.control_regs[1] = FLTR;
      _msg_imu.control_regs[2] = DECR;
      _msg_imu.status = this->getHealthStatus();
      _pub_imu.publish(&_msg_imu);

      // _dr_interrupt.rise(this, &ADIS16470::readData);

      // this->update(/** Try not to call this function during ping. */);

      return true;
    }

    this->setHealthStatus(false);
    return false;
  }

  bool setBiasEstimationTime(int16_t bias_est_time = 0x070A)
  {
    writeWord(NULL_CFG, bias_est_time);
    _cur_bias_est_time = readWord(NULL_CFG);
    return _cur_bias_est_time == bias_est_time;
  }

  bool biasCorrectionUpdate()
  {
    writeWord(GLOB_CMD, 0x01);
    return true;
  }

  void readData()
  {
    // _imu_burst = {};
    memcpy(_imu_burst, wordBurst(), 20 * sizeof(uint8_t));

    // rawData();
    // scaleData();

    // _msg_imu.time.data = this->getNodeHandle()->now();
    // _msg_imu.accelerometer[0].data = AXS;
    // _msg_imu.accelerometer[1].data = AYS;
    // _msg_imu.accelerometer[2].data = AZS;
    // _msg_imu.gyroscope[0].data = GXS;
    // _msg_imu.gyroscope[1].data = GYS;
    // _msg_imu.gyroscope[2].data = GZS;
    // _msg_imu.control_regs[0].data = MSC;
    // _msg_imu.control_regs[1].data = FLTR;
    // _msg_imu.control_regs[2].data = DECR;

    burstChecksum = checksum(_imu_burst);
    this->setHealthStatus(burstChecksum == *(_imu_burst + 9));

    // // _msg_imu.checksum.data = checksum(burstData);
    // // _msg_imu.temperature.data = TEMPS;

    // if (this->getIsTopicAdvertised()) _pub_imu.publish(&(this->_msg_imu));
  }

  void rawData()
  {
    _gyro[0] = *(_imu_burst + 1);  // Scale X Gyro
    _gyro[1] = *(_imu_burst + 2);  // Scale Y Gyro
    _gyro[2] = *(_imu_burst + 3);  // Scale Z Gyro
    _accl[0] = *(_imu_burst + 4);  // Scale X Accel
    _accl[1] = *(_imu_burst + 5);  // Scale Y Accel
    _accl[2] = *(_imu_burst + 6);  // Scale Z Accel
    TEMPS = *(_imu_burst + 7);     // Scale Temp Sensor
  }

  void scaleData()
  {
    _gyro[0] = gyroScale(*(_imu_burst + 1));   // Scale X Gyro
    _gyro[1] = gyroScale(*(_imu_burst + 2));   // Scale Y Gyro
    _gyro[2] = gyroScale(*(_imu_burst + 3));   // Scale Z Gyro
    _accl[0] = accelScale(*(_imu_burst + 4));  // Scale X Accel
    _accl[1] = accelScale(*(_imu_burst + 5));  // Scale Y Accel
    _accl[2] = accelScale(*(_imu_burst + 6));  // Scale Z Accel
    TEMPS = tempScale(*(_imu_burst + 7));      // Scale Temp Sensor
  }

  void update()
  {
    if (this->getEnabledStatus())
    {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
      uint64_t current_time = get_ms_count();
#else
      unsigned long current_time = millis();
#endif
      if ((first_update ||
           (current_time - _prev_update_time) >= _refresh_rate) &&
          this->getConfiguredStatus())
      {
        // scaleData();
        // burstChecksum = checksum(_imu_burst);
        // this->setHealthStatus(burstChecksum == *(_imu_burst + 9));

        sensorUpdate();

        first_update = false;
        _prev_update_time = current_time;
        // Publish Diagnostic messages
        Device::update();
#ifndef DISABLE_ROS
        char _debug[255];
        if (_id_check_pass)
        {
          sprintf(_debug,
                  "FIRMWARE REV: %x PROD ID: %x SERIAL NO: %x BIAS EST: %x",
                  FIRM_REV_VAL, PROD_ID_VAL, SER_NO_VAL, _cur_bias_est_time);
          debugMsg.data = _debug;
          _pub_debug.publish(&debugMsg);
        }
        else
        {
          sprintf(_debug, "Bad Sensor Address");
          debugMsg.data = _debug;
          _pub_debug.publish(&debugMsg);
        }

        _msg_imu.time.data = this->getNodeHandle()->now();
        _msg_imu.accelerometer[0] = _accl[0];
        _msg_imu.accelerometer[1] = _accl[1];
        _msg_imu.accelerometer[2] = _accl[2];
        _msg_imu.gyroscope[0] = _gyro[0];
        _msg_imu.gyroscope[1] = _gyro[1];
        _msg_imu.gyroscope[2] = _gyro[2];
        _msg_imu.control_regs[0] = MSC;
        _msg_imu.control_regs[1] = FLTR;
        _msg_imu.control_regs[2] = DECR;
        _msg_imu.status = this->getHealthStatus();
        if (this->getIsTopicAdvertised()) _pub_imu.publish(&_msg_imu);
#endif
      }
    }
  }

  void sensorUpdate()
  {
    int16_t gyro_out[3], gyro_low[3], accl_out[3], accl_low[3], temp_out;

    gyro_low[0] = readWord(X_GYRO_LOW);
    gyro_out[0] = readWord(X_GYRO_OUT);
    gyro_low[1] = readWord(Y_GYRO_LOW);
    gyro_out[1] = readWord(Y_GYRO_OUT);
    gyro_low[2] = readWord(Z_GYRO_LOW);
    gyro_out[2] = readWord(Z_GYRO_OUT);

    accl_low[0] = readWord(X_ACCL_LOW);
    accl_out[0] = readWord(X_ACCL_OUT);
    accl_low[1] = readWord(Y_ACCL_LOW);
    accl_out[1] = readWord(Y_ACCL_OUT);
    accl_low[2] = readWord(Z_ACCL_LOW);
    accl_out[2] = readWord(Z_ACCL_OUT);

    temp_out = readWord(TEMP_OUT);

    TEMPS = temp_out * 0.1;

    for (int i = 0; i < 3; i++)
    {
      _gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI /
                 180.0 / 655360.0;
      _accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 /
                 52428800.0;
    }

    // std_msgs::Header header;
    // header.frame_id = this->getDeviceName();
    // header.stamp = this->getNodeHandle()->now();
    // _msg_imu.header = header;

    // geometry_msgs::Vector3 angular_velocity;
    // geometry_msgs::Vector3 linear_acceleration;
    // geometry_msgs::Quaternion orientation;

    // angular_velocity.x = GXS;
    // angular_velocity.y = GYS;
    // angular_velocity.z = GZS;
    // linear_acceleration.x = AXS;
    // linear_acceleration.y = AYS;
    // linear_acceleration.z = AZS;

    // double unknown_cov[] = { 0., 0., 0., 0., 0., 0., 0., 0., 0. };
    // double* ukn_cov_ptr = unknown_cov;
    // double* imu_cov_ptr = _msg_imu.angular_velocity_covariance;

    // while (ukn_cov_ptr != unknown_cov + 9)
    //   *imu_cov_ptr++ = *ukn_cov_ptr++;

    // ukn_cov_ptr = unknown_cov;
    // imu_cov_ptr = _msg_imu.linear_acceleration_covariance;

    // while (ukn_cov_ptr != unknown_cov + 9)
    //   *imu_cov_ptr++ = *ukn_cov_ptr++;

    // unknown_cov[0] = -1;
    // ukn_cov_ptr = unknown_cov;
    // imu_cov_ptr = _msg_imu.orientation_covariance;

    // while (ukn_cov_ptr != unknown_cov + 9)
    //   *imu_cov_ptr++ = *ukn_cov_ptr++;

    // _msg_imu.angular_velocity = angular_velocity;
    // _msg_imu.linear_acceleration = linear_acceleration;
  }
};

#endif  // ADIS_16470