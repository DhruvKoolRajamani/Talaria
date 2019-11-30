#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
#include "mbed.h"
#else
#include "Arduino.h"
#endif

#include "devices/hardware.h"
#include "devices/device_manager/device_manager.h"

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
I2CBus PrimaryBus(0, p9, p10);
I2CBus SecondaryBus(1, p28, p27);
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
StrainGauge strain_gauge(0, p15, nh, STRAIN_GAUGE_ID, "strain_gauge",
                         "/devices/index/strain_gauge", 5);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "bend_sensor",
                       "/devices/index/bend_sensor", p16, (1 / 100) * 1000);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, nh, MOTOR_ID, "motor",
            "/devices/index/motor", (1 / 10) * 1000);

#else
StrainGauge strain_gauge(0, A0, nh, STRAIN_GAUGE_ID, "strain_gauge",
                         "/devices/index/strain_gauge", 5);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "bend_sensor",
                       "/devices/index/bend_sensor", 3, (1 / 100) * 1000);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, nh, MOTOR_ID, "motor",
            "/devices/index/motor", (1 / 10) * 1000);

#endif
std_msgs::String debug_msgs;
ros::Publisher debug_pub("/debug", &debug_msgs);
#else
#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
StrainGauge strain_gauge(0, p15, STRAIN_GAUGE_ID, 5);
BendSensor bend_sensor(0x12, PrimaryBus, BEND_SENSOR_ID, p16, (1 / 100) * 1000);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, MOTOR_ID, (1 / 10) * 1000);
#else
StrainGauge strain_gauge(0, A0, STRAIN_GAUGE_ID, 5);
BendSensor bend_sensor(0x12, PrimaryBus, BEND_SENSOR_ID, 3, (1 / 100) * 1000);
Motor motor(0, p19, p25, p26, p6, p8, p7, p5, MOTOR_ID, (1 / 10) * 1000);
#endif
#endif

void addDevices()
{
  device_manager.addDevice(&bend_sensor, BEND_SENSOR_ID);
  device_manager.addDevice(&strain_gauge, STRAIN_GAUGE_ID);
  device_manager.addDevice(&_aVSense, AD_IMU_SENSOR_ID);
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

#ifndef PIO_FRAMEWORK_ARDUINO_PRESENT
int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
#endif

  addDevices();

  volatile bool is_init = false;
  int i = 1;
  while (1)
  {
    if (!is_init)
    {
      is_init = device_manager.initializeDevices();
    }

    // Move all this to device manager
    device_manager.updateDevices(i);

    if (!is_init)
      rate = 1000;

    if (i <= device_manager.getMaxRefreshRate())
    {
      i++;
    }
    else
      i = 0;
    halt(rate);
#ifndef DISABLE_ROS
    nh.spinOnce();
#endif
  }

  return 0;
}
#else
volatile bool is_init = false;
int i = 1;
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
  halt(rate);
#ifndef DISABLE_ROS
  nh.spinOnce();
#endif
}
#endif