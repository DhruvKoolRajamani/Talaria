/**
 * @file spi_device.h
 * @author Dhruv Kool Rajamani (dkoolrajamani@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2020-0.-10
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef SPI_DEVICE_H
#define SPI_DEVICE_H

#include "device.h"
#include "devices/hardware.h"
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

class SPIDevice : public Device
{
private:
protected:
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  PinName _miso;
  PinName _mosi;
  PinName _sclk;
  int _clock_speed;
  PinName _cs;
  PinName _dr;
  PinName _rst;
  SPI* _spi_bus = nullptr;

#else
  int _miso;
  int _mosi;
  int _sclk;
  uint32_t _clock_speed;
  int _cs;
  int _dr;
  int _rst;
  SPIClass* _spi_bus;
#endif

  int _stall = 25;

#ifndef DISABLE_ROS
#ifndef DISABLE_DIAGNOSTICS
  diagnostic_msgs::KeyValue _diagnostic_chip_id;
#endif
#endif
public:
  /** CONSTRUCTORS */

#ifndef DISABLE_ROS

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  /**
   * @brief Construct a new SPIDevice object
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
  SPIDevice(uint8_t id, PinName mosi, PinName miso, PinName sclk, PinName cs,
            PinName dr, PinName rst, ros::NodeHandle& nh,
            int clock_speed = 100000, uint8_t dev_index = 0,
            const char* dev_name = NULL, const char* frame_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : Device(id, dev_index, nh, dev_name, frame_name, topic_name, refresh_rate)
    , _mosi(mosi)
    , _miso(miso)
    , _sclk(sclk)
    , _clock_speed(clock_speed)
    , _cs(cs)
    , _dr(dr)
    , _rst(rst)
    , _spi_bus(new SPI(_mosi, _miso, _sclk))
  {
#else
  /**
   * @brief Construct a new SPIDevice object
   *
   * @param mosi
   * @param miso
   * @param sclk
   * @param cs
   * @param dr
   * @param rst
   * @param spi_bus
   * @param nh
   * @param clock_speed
   * @param dev_index
   * @param dev_name
   * @param topic_name
   * @param refresh_rate
   */
  SPIDevice(int mosi, int miso, int sclk, int cs, int dr, int rst,
            ros::NodeHandle& nh, uint32_t clock_speed = 1000000,
            uint8_t dev_index = 0, const char* dev_name = NULL,
            const char* topic_name = NULL, int refresh_rate = 1)
    : Device(dev_index, nh, dev_name, topic_name, refresh_rate)
    , _mosi(mosi)
    , _miso(miso)
    , _sclk(sclk)
    , _clock_speed(clock_speed)
    , _cs(cs)
    , _dr(dr)
    , _rst(rst)
    , _spi_bus(&spi_bus)
  {
#endif
    setIndex(dev_index);
#ifndef DISABLE_DIAGNOSTICS
    _diagnostic_chip_id.key = "Chip Id";
    _diagnostic_chip_id.value = (char*)0x00;
    _spi_bus->setDiagnosticsData(_diagnostic_chip_id);
#endif
  }
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  /**
   * @brief Construct a new SPIDevice object
   *
   * @param cs
   * @param dr
   * @param rst
   * @param spi_bus
   * @param dev_index
   * @param refresh_rate
   */
  SPIDevice(PinName cs, PinName dr, PinName rst, SPIBus& spi_bus,
            uint8_t dev_index = 0, int refresh_rate = 1)
    : Device(dev_index, refresh_rate)
    , _cs(cs)
    , _dr(dr)
    , _rst(rst)
    , _spi_bus(&spi_bus)
  {
#else
  /**
   * @brief Construct a new SPIDevice object
   *
   * @param cs
   * @param dr
   * @param rst
   * @param spi_bus
   * @param dev_index
   * @param refresh_rate
   */
  SPIDevice(int cs, int dr, int rst, SPIBus& spi_bus, uint8_t dev_index = 0,
            int refresh_rate = 1)
    : Device(dev_index, refresh_rate)
    , _cs(cs)
    , _dr(dr)
    , _rst(rst)
    , _spi_bus(&spi_bus)
  {
#endif
    if (_spi_bus->getInitStatus()) _spi_bus->begin(_cs, _rst);
    setIndex(dev_index);
  }
#endif

  /** DESTRUCTOR */

  /**
   * @brief Destroy the SPIDevice object
   *
   */
  ~SPIDevice()
  {
    if (_spi_bus) delete _spi_bus;
  }

  /** METHODS */

  /**
   * @brief Reset the pin for the Device and wait for a delay in ms before
   * restarting it
   *
   * @param delay
   */
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  virtual void reset(int delay = 100)
  {
    DigitalOut resetPin(_rst, false);
    wait_ms(100);
    resetPin = true;
    wait_ms(delay);
#else
  virtual void reset(const uint8_t delay_ms = 100)
  {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, false);
    delay(delay_ms);
    digitalWrite(_rst, true);
#endif
  }

  void begin()
  {
    deselect();
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    // DigitalOut csPin(_cs, 1);
    DigitalOut rstPin(_rst, 1);
    configBus();
#else
    SPIClass::begin();
    configBus();
    pinMode(_rst, OUTPUT);     // Set RST pin to be an output
    digitalWrite(_cs, HIGH);   // Initialize CS pin to be high
    digitalWrite(_rst, HIGH);  // Initialize RST pin to be high
#endif
  }

  /**
   * @brief Sets SPI bit order, clock divider, and data mode. This function is
   * useful when there are multiple SPI devices using different settings.
   *
   * @return int
   */
  int configBus()
  {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    _spi_bus->format(8, 3);
    _spi_bus->frequency(_clock_speed);
#else
    SPISettings Settings(_clock_speed, MSBFIRST, SPI_MODE3);
    _spi_bus->beginTransaction(Settings);
#endif
    return 1;
  }

  /**
   * @brief Select the slave device
   *
   */
  void select()
  {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    DigitalOut(_cs, 0);
    wait_ns(200);
#else
    pinMode(_cs, OUTPUT);  // Set CS pin to be an output
    digitalWrite(_cs, LOW);
#endif
  }

  void deselect()
  {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    DigitalOut(_cs, 1);
#else
    pinMode(_cs, OUTPUT);  // Set CS pin to be an output
    digitalWrite(_cs, HIGH);
#endif
  }

  /**
   * @brief Read word from device
   *
   * @param regAddr
   */

  int16_t readWord(uint8_t regAddr)
  {
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    // Write register address to be read
    select();                       // Select the slave
    _spi_bus->write((int)regAddr);  // Write address over SPI bus
    _spi_bus->write(0x00);  // Write 0x00 to the bus fill the 16 bit transaction
                            // requirement
    deselect();             // Deselect slave
    wait_us(_stall);        // Delay to not violate read rate

    // Read data from requested register
    select();  // Set CS low to enable device
    int _msbData =
        _spi_bus->write(0x00);  // Send (0x00), place upper byte into variable
    int _lsbData =
        _spi_bus->write(0x00);  // Send (0x00), place lower byte into variable
    deselect();                 // Set CS high to disable device
    wait_us(_stall);            // Delay to not violate read rate

    int16_t _dataOut = (_msbData << 8) |
                       (_lsbData & 0xFF);  // Concatenate upper and lower bytes
    // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.
    return _dataOut;
#else
    // Read registers using SPI

    // Write register address to be read
    digitalWrite(_cs, LOW);       // Set CS low to enable device
    _spi_bus->transfer(regAddr);  // Write address over SPI bus
    _spi_bus->transfer(0x00);     // Write 0x00 to the SPI bus fill the 16 bit
                                  // transaction requirement
    digitalWrite(cs, HIGH);       // Set CS high to disable device

    delayMicroseconds(_stall);  // Delay to not violate read rate

    // Read data from requested register
    digitalWrite(_cs, LOW);                       // Set CS low to enable device
    uint8_t _msbData = _spi_bus->transfer(0x00);  // Send (0x00) and place upper
                                                  // byte into variable
    uint8_t _lsbData = _spi_bus->transfer(0x00);  // Send (0x00) and place lower
                                                  // byte into variable
    digitalWrite(cs, HIGH);  // Set CS high to disable device

    delayMicroseconds(_stall);  // Delay to not violate read rate

    int16_t _dataOut = (_msbData << 8) |
                       (_lsbData & 0xFF);  // Concatenate upper and lower bytes
    // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

    return (_dataOut);
#endif
  }

  /**
   * @brief Write register address and data
   *
   * @param regAddr
   * @param regData
   */
  int writeWord(uint8_t regAddr, int16_t regData)
  {
    // Toggle sign bit, and check that the address is 8 bits
    uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8);
    uint16_t lowWord = (addr | (regData & 0xFF));  // OR Register address (A)
                                                   // with data(D) (AADD)
    uint16_t highWord =
        ((addr | 0x100) | ((regData >> 8) & 0xFF));  // OR Register address with
                                                     // data and increment
                                                     // address

    // Split words into chars
    uint8_t highBytehighWord = (highWord >> 8);
    uint8_t lowBytehighWord = (highWord & 0xFF);
    uint8_t highBytelowWord = (lowWord >> 8);
    uint8_t lowBytelowWord = (lowWord & 0xFF);

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
    // Write register address to be read
    select();                               // Set CS low to enable device
    _spi_bus->write((int)highBytelowWord);  // Write high byte from low word to
                                            // SPI bus
    _spi_bus->write((int)lowBytelowWord);   // Write low byte from low word to
                                            // SPI bus requirement
    deselect();                             // Set CS high to disable device
    wait_us(_stall);                        // Delay to not violate read rate

    // Read data from requested register
    select();                                // Set CS low to enable device
    _spi_bus->write((int)highBytehighWord);  // Send (0x00), place upper byte
                                             // into variable
    _spi_bus->write((int)lowBytehighWord);   // Send (0x00), place lower byte
                                             // into variable
    deselect();                              // Set CS high to disable device
    wait_ms(_stall);                         // Delay to not violate read rate

    return 1;
#else
    // Write highWord to SPI bus
    digitalWrite(cs, LOW);                  // Set CS low to enable device
    _spi_bus->->transfer(highBytelowWord);  // Write high byte from low word to
                                            // SPI bus
    _spi_bus->->transfer(lowBytelowWord);   // Write low byte from low word to
                                            // SPI bus
    digitalWrite(cs, HIGH);                 // Set CS high to disable device

    delayMicroseconds(_stall);
    ;  // Delay to not violate read rate

    // Write lowWord to SPI bus
    digitalWrite(cs, LOW);                   // Set CS low to enable device
    _spi_bus->->transfer(highBytehighWord);  // Write high byte from high word
                                             // to SPI bus
    _spi_bus->->transfer(lowBytehighWord);   // Write low byte from high word to
                                             // SPI bus
    digitalWrite(cs, HIGH);                  // Set CS high to disable device

    delayMicroseconds(_stall);
    ;  // Delay to not violate read rate

    return (1);
#endif
  }

  virtual int16_t ping(uint8_t whoami_reg_address = 0x00, int delay_ms = 0)
  {
    return readWord(whoami_reg_address);
  }

  /** GETTERS */

  /** SETTERS */

  /**
   * @brief Set the Clock Speed object
   *
   * @param clock_speed
   */
  void setClockSpeed(uint32_t clock_speed)
  {
    _clock_speed = clock_speed;
    begin();
  }
};

#endif  // SPI_DEVICE_H