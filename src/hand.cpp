#include "mbed.h"
#include "devices/hardware.h"
#include "devices/device_manager/device_manager.h"

I2CBus PrimaryBus(0, p9, p10);
I2CBus SecondaryBus(1, p28, p27);

#ifndef DISABLE_ROS
ros::NodeHandle nh;
// DeviceManager device_manager(nh);
#else
DeviceManager device_manager;
#endif

#ifndef DISABLE_ROS
// // StrainGauge strain_gauge(0, p15, nh, 1, "strain_gauge",
// //                          "/devices/index/strain_gauge_1");
BendSensor bend_sensor(0x12, PrimaryBus, nh, 1, "bs", "/devices/index/bs", p15);
std_msgs::String network_msg;
#else
BendSensor bend_sensor(0, PrimaryBus, 1);
#endif

float rate = 1;
// 1000 * 1 / 50;
int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
#endif

  volatile bool is_init = false;
  int i = 1;
  while (1)
  {
    if (!is_init)
    {
      bend_sensor.softReset();
      wait_ms(1000);
      is_init = bend_sensor.initialize();
    }

    // Move all this to device manager
    if (i == (1000 * 1 / 50))
    {
      bend_sensor.update();
      i = 1;
    }

    if (!is_init)
      rate = 1000;
    wait_ms(rate);
    i++;
#ifndef DISABLE_ROS
    nh.spinOnce();
#endif
  }

  return 0;
}