#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#include "devices/hardware.h"
#include "devices/device_manager/device_manager.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#ifndef NUCLEO
I2CBus PrimaryBus(0, p9, p10);
I2CBus SecondaryBus(1, p28, p27);
#else
// I2CBus PrimaryBus(0, PC_9, PA_8);
I2CBus PrimaryBus(0, PF_0, PF_1);
// I2CBus SecondaryBus2(2, PD_13, PD_12);
// I2CBus PrimaryBus(3, PD_15, PD_14);
#endif
#else
I2CBus PrimaryBus(0, PIN_WIRE_SDA, PIN_WIRE_SCL);
#endif

#ifndef DISABLE_ROS
ros::NodeHandle nh;
DeviceManager device_manager(nh);
#else
DeviceManager device_manager;
#endif

#ifndef DISABLE_ROS
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#ifndef NUCLEO
// StrainGauge strain_gauge(0, p15, nh, STRAIN_GAUGE_ID, "strain_gauge",
//                          "/devices/index/strain_gauge", 5);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "index",
                       "/devices/index/bend_sensor", p16, 10);
Motor motor(0, p19 /*aVSense*/, p25 /*aEnable*/, p26 /*vRef*/, p6 /*nSleep*/,
            p8 /*nFault*/, p7 /*nConfig*/, p5 /*aPhase*/, nh, MOTOR_ID, "index",
            "/devices/index/motor_measured", "/devices/index/motor_desired",
            10);
#else
// StrainGauge strain_gauge(0, p15, nh, STRAIN_GAUGE_ID, "strain_gauge",
//                          "/devices/index/strain_gauge", 5);
// BendSensor bend_sensor(0, 0x12, PrimaryBus, nh, BEND_SENSOR_ID,
// "bend_sensor",
//                        "index", "/devices/index/bend_sensor", PF_2, 50);

// Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
//         PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
//         ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
//         const char* meas_topic_name, const char* des_topic_name,
//         int refresh_RATE);

// // Index
// Motor thumb_motor(0, PA_3 /*aVSense*/, PD_8 /*aEnable*/, PC_8 /*vRef*/,
//                   PE_9 /*nSleep*/, PE_8 /*nFault*/, PE_11 /*nConfig*/,
//                   PF_13 /*aPhase*/, nh, MOTOR_ID, "thumb",
//                   "/devices/thumb/motor_measured",
//                   "/devices/thumb/motor_desired", 50);

// Index
Motor index_motor(0, PA_3 /*aVSense*/, PD_9 /*aEnable*/, PC_9 /*vRef*/,
                  PF_14 /*nSleep*/, PE_8 /*nFault*/, PE_13 /*nConfig*/,
                  PE_11 /*aPhase*/, nh, MOTOR_ID, "motor", "index", nullptr,
                  "/devices/index/motor_cmd", 20 /*Hz*/);  // 10

// // Index
// Motor middle_motor(2, PA_3 /*aVSense*/, PD_9 /*aEnable*/, PB_8 /*vRef*/,
//                    PF_15 /*nSleep*/, PE_8 /*nFault*/, PE_11 /*nConfig*/,
//                    PE_13 /*aPhase*/, nh, MOTOR_ID, "middle",
//                    "/devices/middle/motor_measured",
//                    "/devices/middle/motor_desired", 50);

// // Index
// Motor ring_motor(3, PA_3 /*aVSense*/, PD_9 /*aEnable*/, PB_9 /*vRef*/,
//                  PG_9 /*nSleep*/, PE_8 /*nFault*/, PE_11 /*nConfig*/,
//                  PG_14 /*aPhase*/, nh, MOTOR_ID, "ring",
//                  "/devices/ring/motor_measured",
//                  "/devices/ring/motor_desired", 50);

// ADIS16470(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName dr,
//             PinName rst, ros::NodeHandle& nh, int clock_speed = 100000,
//             uint8_t dev_index = 0, const char* dev_name = NULL,
//             const char* topic_name = NULL, int refresh_RATE = 1)
ADIS16470 imu(0, PA_7, PA_6, PA_5, PD_14, PD_15, PF_12, nh, 1000000, ADI_IMU_ID,
              "imu", "hand_imu", "/devices/hand_imu/imu",
              50 /*Hz*/);  // Hertz 20

#endif
#else
StrainGauge strain_gauge(0, A0, nh, STRAIN_GAUGE_ID, "strain_gauge",
                         "/devices/index/strain_gauge", 50);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "index",
                       "/devices/index/bend_sensor", 3, 10);
Motor motor(0, p18, p25, p26, p6, p8, p7, p5, nh, MOTOR_ID, "motor",
            "/devices/index/motor_measured", "/devices/index/motor_desired",
            10);

#endif
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
StrainGauge strain_gauge(0, p15, STRAIN_GAUGE_ID, 5);
// BendSensor bend_sensor(0x12, PrimaryBus, BEND_SENSOR_ID, p16, 10);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, MOTOR_ID, 10);
#else
StrainGauge strain_gauge(0, A0, STRAIN_GAUGE_ID, 50);
BendSensor bend_sensor(0x12, PrimaryBus, BEND_SENSOR_ID, 3, 10);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, MOTOR_ID,
            10);  // change arduino pins
#endif
#endif

static int MAX_REFRESH_RATE = RATE;
int devices_counter = 0;

void addDevice(Device* device)
{
  device_manager.addDevice(device, devices_counter);
  devices_counter++;
}

void addDevices()
{
  // addDevice(&bend_sensor);
  // addDevice(&strain_gauge);
  addDevice(&imu);
  // addDevice(&thumb_motor);
  addDevice(&index_motor);
  // addDevice(&middle_motor);
  // addDevice(&ring_motor);

#ifdef DISABLE_ROS
  print("Added Devices\n");
#endif
  MAX_REFRESH_RATE = device_manager.getMaxRefreshRate();
#ifdef DISABLE_ROS
  char str[50];
  sprintf(str, "Set Max Refresh Rate to %d\n", MAX_REFRESH_RATE);
  print(str);
#endif
}

// Can shift to Utils/hardware header
void halt(float time_ms)
{
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
  wait_ms(time_ms);
#else
  delay(time_ms);
#endif
}

volatile bool is_init = false;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
int main()
{
#ifndef DISABLE_ROS
  nh.getHardware()->setBaud(9600);  // 57600
  nh.initNode();

  // wait until you are actually connected
  while (!nh.connected())
  {
    nh.spinOnce();
  }
#else
  wait_ms(1000);
  print("Hello\n");
  wait_ms(1000);
#endif

  char str[100];

  addDevices();

  volatile bool is_init = false;
  while (1)
  {
    if (!is_init)
    {
      device_manager.initializeDevices();

      is_init = device_manager.getInitStatus();

#ifndef DISABLE_ROS
      for (int i = 0; i < NUM_DEVICES; i++)
      {
        Device* dev = device_manager.getDevice(i);
        if (device_manager._init_status[i])
        {
          const char* tempstr = "Successfully initialized:  - ";
          snprintf(str,
                   strlen(tempstr) + strlen(dev->getDeviceName()) +
                       sizeof(dev->getId()) + 1,
                   "Successfully initialized: %s - %d", dev->getDeviceName(),
                   dev->getId());
          nh.loginfo(str);
        }
        else
        {
          const char* tempstr = "Failed to initialize:  - ";
          snprintf(str,
                   strlen(tempstr) + strlen(dev->getDeviceName()) +
                       sizeof(dev->getId()) + 1,
                   "Failed to initialize: %s - %d", dev->getDeviceName(),
                   dev->getId());
          nh.logwarn(str);
        }
      }

      if (is_init) nh.loginfo("Initialization Successful");
        // else
        // nh.logwarn("Initialization Failed, Retrying ...");
#else
      print(str);
#endif
    }

    // Move all this to device manager
    device_manager.updateDevices();

#ifndef DISABLE_ROS
    nh.spinOnce();
#endif
    halt(RATE);
  }

  return 0;
}
#else
void setup()
{
#ifndef DISABLE_ROS
  nh.initNode();
#else
  delay(3000);
  Serial.begin(9600);
  delay(3000);
  print("Hello!\n");
#endif

  addDevices();
#ifdef DISABLE_ROS
  print("Added Devices!\n");
#endif

  is_init = device_manager.initializeDevices();
}

void loop()
{
  device_manager.updateDevices(i);

  if (i <= device_manager.getMaxRefreshRate())
  {
    i++;
  }
  else
    i = 0;
#ifndef DISABLE_ROS
  nh.spinOnce();
  halt(RATE);
#endif
}
#endif