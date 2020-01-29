/**
 * @file bend_sensor.cpp
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-11-24
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "devices/bend_sensor/bend_sensor.h"

// CONSTRUCTORS
#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
BendSensor::BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
                       uint8_t dev_index, const char* dev_name,
                       const char* topic_name, PinName reset_pin,
                       int refresh_rate)
  : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name,
              refresh_rate)
  , _reset_pin(reset_pin)
  , _pub_bend_sensor(topic_name, &(this->_msg_bend_sensor))
{
#else
BendSensor::BendSensor(int address, I2CBus& i2c_bus, ros::NodeHandle& nh,
                       uint8_t dev_index, const char* dev_name,
                       const char* topic_name, int reset_pin, int refresh_rate)
  : I2CDevice(address, i2c_bus, nh, dev_index, dev_name, topic_name,
              refresh_rate)
  , _reset_pin(reset_pin)
  , _pub_bend_sensor(topic_name, &(this->_msg_bend_sensor))
{
  pinMode(_reset_pin, OUTPUT);
#endif
  setIsTopicAdvertised(nh.advertise(_pub_bend_sensor));
}
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
BendSensor::BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index,
                       PinName reset_pin, int refresh_rate)
  : I2CDevice(address, i2c_bus, dev_index, refresh_rate), _reset_pin(reset_pin)
{
}
#else
BendSensor::BendSensor(int address, I2CBus& i2c_bus, uint8_t dev_index,
                       int reset_pin, int refresh_rate)
  : I2CDevice(address, i2c_bus, dev_index, refresh_rate), _reset_pin(reset_pin)
{
}
#endif
#endif

// DESTRUCTORS
BendSensor::~BendSensor()
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
bool BendSensor::initialize()
{
#ifdef DISABLE_ROS
  print("Entered Bend Initialize\n");
#endif
  I2CDevice::initialize();
#ifdef DISABLE_ROS
  print("Starting Bend Initialize\n");
#else
  _msg_bend_sensor.header.frame_id = this->getDeviceName();
  _msg_bend_sensor.header.stamp = this->getNodeHandle()->now();
#endif
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  if (!this->ping())
#else
  if (!this->ping())
#endif
  {
#ifndef DISABLE_ROS
    _msg_bend_sensor.debug.data = "Ping failed";
    _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
#else
    print("Ping failed\n");
#endif
    setConfiguredStatus(false);
    return false;
  }

  if (!this->setSampleRate(BEND_SENSOR_100_HZ))
  {
#ifndef DISABLE_ROS
    _msg_bend_sensor.debug.data = "Set Sensor Rate Nack";
    _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
#else
    print("Set Sample Rate failed\n");
#endif
    setConfiguredStatus(false);
    return false;
  }

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  wait_ms(2);
#else
  delay(2);
#endif

  if (!this->enableStretchValues(true))
  {
#ifndef DISABLE_ROS
    _msg_bend_sensor.debug.data = "Stretch Enable Nack";
    _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
#else
    print("Stretch Enable failed\n");
#endif
    setConfiguredStatus(false);
    return false;
  }

  if (!this->beginPollingData(true))
  {
#ifndef DISABLE_ROS
    _msg_bend_sensor.debug.data = "Begin Polling Nack";
    _pub_bend_sensor.publish(&(this->_msg_bend_sensor));
#else
    print("Bend Polling failed\n");
#endif
    setConfiguredStatus(false);
    return false;
  }
  setHealthStatus(true);
  setConfiguredStatus(true);
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  wait_ms(10);
#else
  delay(10);
#endif
#ifdef DISABLE_ROS
  print("Exiting Bend Initialize\n");
#endif
  return true;
}

bool BendSensor::enableStretchValues(bool enable)
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE] = { BEND_SENSOR_READ_STRETCH,
                                                enable, 0 };

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  return this->writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  return this->writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
bool BendSensor::ping(int chip_id_reg_address, int delay_ms)
#else
bool BendSensor::ping(uint8_t chip_id_reg_address, int delay_ms)
#endif
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];
  if (readRegister(chip_id_reg_address, buffer, BEND_SENSOR_TRANSFER_SIZE,
                   false, delay_ms))
  {
    // setConfiguredStatus(true);
    setEnabledStatus(true);
    this->setChipId(buffer[0]);

#ifndef DISABLE_ROS
    _msg_bend_sensor.chip_id.data = this->getChipId();
    _msg_bend_sensor.bend.data = this->_bend_angle;
    _msg_bend_sensor.stretch.data = this->_stretch_value;
#ifndef DISABLE_DIAGNOSTICS
    _diagnostic_chip_id.value = (char*)buffer;
    this->setDiagnosticsData(_diagnostic_chip_id);
#endif
#else
    char str[100];
    sprintf(str, "Chip Id is: %x", this->getChipId());
    print(str);
#endif
    return true;
  }
  else
    return false;
}

void BendSensor::update(int loop_counter)
{
  // Only update if update rate for the sensor is the same as the sampling
  // rate
  if (this->getEnabledStatus())
  {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    uint64_t current_time = get_ms_count();
#else
    unsigned long current_time = millis();
#endif
    if ((first_update || (current_time - _prev_update_time) >= _refresh_rate) &&
        this->getConfiguredStatus())
    {
      first_update = false;
      _prev_update_time = current_time;

      // Publish Diagnostic messages
      Device::update(loop_counter);

      this->readData();

      if (this->_data_ready == BEND_SENSOR_SAMPLE)
      {
        processNewData();
#ifndef DISABLE_ROS
        _msg_bend_sensor.header.stamp = this->getNodeHandle()->now();
        _msg_bend_sensor.chip_id.data = this->getChipId();
        _msg_bend_sensor.bend.data = this->_bend_angle;
        _msg_bend_sensor.stretch.data = this->_stretch_value;
        if (this->getIsTopicAdvertised())
          _pub_bend_sensor.publish(&(this->_msg_bend_sensor));

#else
        // Serialize
        char str[100];
        sprintf(str, "Bend:\n\tChip: %x\n\tBend: %f\n\tStretch: %f\n",
                this->getChipId(), this->_bend_angle, this->_stretch_value);
        // print(str);
#endif
      }
    }
  }
  else
  {
    initialize();
  }
}

// Convert two bytes of buffer[] to int16
inline int16_t BendSensor::decodeInt16(const uint8_t* p_encoded_data)
{
  return ((((uint16_t)(p_encoded_data)[0])) |
          (((int16_t)(p_encoded_data)[1]) << 8));
}

/*
 *  Second order Infinite impulse response low pass filter. Sample freqency
 * 100 Hz. Cutoff freqency 20 Hz.
 */
void BendSensor::signalFilter(float* sample)
{
  static float filter_samples[2][6];

  for (uint8_t i = 0; i < 2; i++)
  {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) -
                           0.19581571265583314f * filter_samples[i][2] +
                           0.20657208382614792f * (filter_samples[i][3] +
                                                   2 * filter_samples[i][4] +
                                                   filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

void BendSensor::deadzoneFilter(float* sample)
{
  static float prev_sample[2];
  float dead_zone = 0.75f;

  for (uint8_t i = 0; i < 2; i++)
  {
    if (!(fabs(sample[i] - prev_sample[i]) > dead_zone) || (sample[1] > -10))
      sample[i] = prev_sample[i];
    else
      prev_sample[i] = sample[i];
  }
}

// Checks to see if new data is available
// This must be called regularly to update getX and getY functions
// Returns true if a new sample is received
// The sample is then parsed and run through the filters
bool BendSensor::readData()
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE] = { 0, 0, 0 };
  int16_t temp;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  if (readBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE))
#else
  if (readBytes(buffer, BEND_SENSOR_TRANSFER_SIZE))
#endif
  {
    this->_data_ready = buffer[0];
    if (buffer[0] == BEND_SENSOR_SAMPLE)
    {
      temp = decodeInt16(&buffer[1]);
      // Negative sign for orientation of the sensor
      // * ((float)M_PI / 180.0f);
      this->_bend_angle = (float)((float)temp / 64.0f);
    }
    else if (buffer[0] == BEND_SENSOR_STRETCH_SAMPLE)
    {
      temp = decodeInt16(&buffer[1]);
      this->_stretch_value = (float)temp / 64.0f;
    }
    else
    {
#ifndef DISABLE_ROS
      _msg_bend_sensor.debug.data = "ReadData buffer incorrect";
#endif
      return false;
    }
    // _msg_bend_sensor.debug.data = "ReadData Success";
    return true;
  }
  this->setEnabledStatus(false);
#ifndef DISABLE_ROS
  _msg_bend_sensor.debug.data = "ReadData Fail";
#endif
  // fallback to initializing again.
  return false;
}

// Takes the data from the latest sample and loads it into the filters
void BendSensor::processNewData()
{
  static float samples[] = { this->_bend_angle, this->_stretch_value };
  signalFilter(samples);
  deadzoneFilter(samples);
}

void BendSensor::disable()
{
  Device::disable();
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE] = { BEND_SENSOR_SHUTDOWN, 0, 0 };

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
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
bool BendSensor::calibrate(uint8_t ads_calibration_step, uint8_t degrees)
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_CALIBRATE;
  buffer[1] = ads_calibration_step;
  buffer[2] = degrees;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}

// Delete the current calibration values from non-volatile memory and restore
// the factory calibration
bool BendSensor::clearCalibration()
{
  calibrate(BEND_SENSOR_CALIBRATE_CLEAR, 0);
}

/**
 * @brief Reset the Angular Displacement Sensor
 */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
void BendSensor::reset(PinName pin, int delay_ms)
#else
void BendSensor::reset(int pin, int delay_ms)
#endif
{
  // Configure reset line as an output
  this->setPinState(pin, false);
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  wait_ms(10);
#else
  delay(10);
#endif
  this->setPinState(pin, true);
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  wait_ms(delay_ms);
#else
  delay(delay_ms);
#endif
}

/**
 * @brief Places ADS in polled or sleep mode
 * @param  run true if activating ADS, false is putting in suspend mode
 * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
 */
bool BendSensor::beginPollingData(bool poll)
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_POLLED_MODE;
  buffer[1] = poll;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}

/**
 * @brief Places ADS in free run or sleep mode
 * @param  run true if activating ADS, false is putting in suspend mode
 * @return  BEND_SENSOR_OK if successful BEND_SENSOR_ERR_IO if failed
 */
bool BendSensor::beginReadingData(bool run)
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_RUN;
  buffer[1] = run;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}

/**
 * @brief Updates the I2C address of the selected ADS. The default address
 *      is 0x13. Use this function to program an ADS to allow multiple
 *      devices on the same I2C bus.
 * @param device  device number of the device that is being updated
 * @param address new address of the ADS
 * @return  True if successful false if failed
 */
bool BendSensor::setAddress(uint8_t newAddress)
{
  if (newAddress < 0x08 || newAddress > 0x77)
    return false;  // Address is out of bounds

  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_SET_ADDRESS;
  buffer[1] = newAddress;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  if (writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE) == false)
#else
  if (writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE) == false)
#endif
    return false;

  this->setDeviceAddress(newAddress);  // Update address only after success

  return true;
}

/**
 * @brief Sets the sample rate of the ADS in free run mode
 * @param  sample rate
 * @return  True if successful, false if failed
 */
bool BendSensor::setSampleRate(uint16_t sps)
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_SPS;

  // SPS is loaded little endian
  buffer[1] = (uint8_t)((sps & 0x00FF) >> 0);
  buffer[2] = (uint8_t)((sps & 0xFF00) >> 8);

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}

// Send command to initiate soft reset
bool BendSensor::softReset()
{
  uint8_t buffer[BEND_SENSOR_TRANSFER_SIZE];

  buffer[0] = BEND_SENSOR_RESET;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  writeBytes((char*)buffer, BEND_SENSOR_TRANSFER_SIZE);
#else
  writeBytes(buffer, BEND_SENSOR_TRANSFER_SIZE);
#endif
}