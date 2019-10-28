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
BendSensor bend_sensor(0x12, PrimaryBus, nh, 0, "bs", "/devices/index");
std_msgs::String network_msg;
ros::Publisher network_pub("network_strings", &network_msg);
#else
BendSensor bend_sensor(0x12, PrimaryBus, 0);
#endif

char hello_msg[50] = "";

int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
  nh.advertise(network_pub);
#endif

  if (!bend_sensor.initialize())
  {
    sprintf(hello_msg, "Initialize Error");

#ifndef DISABLE_ROS
    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
#else
    printf("%s\n", hello_msg);
#endif
    wait_ms(100);
  }

  while (1)
  {
    bend_sensor.update();
    sprintf(hello_msg, "Chip Id is: %x", bend_sensor.getChipId());

#ifndef DISABLE_ROS
    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
#else
    printf("%s\n", hello_msg);
#endif
    wait_ms(100);
  }

  return 0;
}