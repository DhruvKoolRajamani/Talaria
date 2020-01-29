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
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, nh, MOTOR_ID, "index",
            "/devices/index/motor_measured", "/devices/index/motor_desired",
            10);
#else
// StrainGauge strain_gauge(0, p15, nh, STRAIN_GAUGE_ID, "strain_gauge",
//                          "/devices/index/strain_gauge", 5);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "index",
                       "/devices/index/bend_sensor", PF_2, 10);

// Motor(uint8_t id, PinName aVSense, PinName aEnable, PinName vRef,
//         PinName nSleep, PinName nFault, PinName nConfig, PinName aPhase,
//         ros::NodeHandle& nh, uint8_t dev_index, const char* dev_name,
//         const char* meas_topic_name, const char* des_topic_name,
//         int refresh_rate);

Motor motor(0, PA_3 /*aVSense*/, PB_9 /*aEnable*/, PC_9 /*vRef*/,
            PE_9 /*nSleep*/, PF_14 /*nFault*/, PE_11 /*nConfig*/,
            PF_13 /*aPhase*/, nh, MOTOR_ID, "index",
            "/devices/index/motor_measured", "/devices/index/motor_desired",
            10);
#endif
#else
StrainGauge strain_gauge(0, A0, nh, STRAIN_GAUGE_ID, "strain_gauge",
                         "/devices/index/strain_gauge", 50);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "index",
                       "/devices/index/bend_sensor", 3, 10);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, nh, MOTOR_ID, "motor",
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

static int MAX_REFRESH_RATE = 1;

void addDevices()
{
  device_manager.addDevice(&bend_sensor, BEND_SENSOR_ID);
  // device_manager.addDevice(&strain_gauge, STRAIN_GAUGE_ID);
  device_manager.addDevice(&motor, MOTOR_ID);
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

#ifndef DISABLE_ROS
float rate = 1;
#else
float rate = 1;
#endif

volatile bool is_init = false;
int i = 1;

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
#else
  wait_ms(1000);
  print("Hello\n");
  wait_ms(1000);
  char str[100];
#endif

  addDevices();

  volatile bool is_init = false;
  int i = 1;
  while (1)
  {
    if (!is_init)
    {
      is_init = device_manager.initializeDevices();
#ifdef DISABLE_ROS
      sprintf(str, "Not initialized count: %d\n", i);
      print(str);
#endif
    }

    // Move all this to device manager
    device_manager.updateDevices(i);

    if (!is_init)
    {
      rate = 1000;
    }

    if (i <= device_manager.getMaxRefreshRate())
    {
      i++;
    }
    else
      i = 0;
#ifndef DISABLE_ROS
    nh.spinOnce();
#endif
    halt(rate);
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
  halt(rate);
#endif
}
#endif