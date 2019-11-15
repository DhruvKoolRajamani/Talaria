#include "mbed.h"
#include "devices/hardware.h"
#include "devices/device_manager/device_manager.h"

I2CBus PrimaryBus(0, p9, p10);
I2CBus SecondaryBus(1, p28, p27);

#ifndef DISABLE_ROS
ros::NodeHandle nh;
DeviceManager device_manager(nh);
#else
DeviceManager device_manager;
#endif

#ifndef DISABLE_ROS
StrainGauge strain_gauge(0, p16, nh, STRAIN_GAUGE_ID, "strain_gauge",
                         "/devices/index/strain_gauge", 5);
BendSensor bend_sensor(0x12, PrimaryBus, nh, BEND_SENSOR_ID, "bend_sensor",
                       "/devices/index/bend_sensor", p15, 20);

// std_msgs::String debug_string;
// ros::Publisher debug_pub("/debug_pub", &debug_string);
#else
BendSensor bend_sensor(0, PrimaryBus, BEND_SENSOR_ID);
#endif

void addDevices()
{
  device_manager.addDevice(&bend_sensor, BEND_SENSOR_ID);
  device_manager.addDevice(&strain_gauge, STRAIN_GAUGE_ID);
}

float rate = 1;
int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
  // nh.advertise(debug_pub);
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
    wait_ms(rate);
#ifndef DISABLE_ROS
    nh.spinOnce();
#endif
  }

  return 0;
}