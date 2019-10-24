#include "mbed.h"
#include "devices/hardware.h"
#include "devices/device_manager/device_manager.h"

I2CBus BodyBus(0, PC_9, PA_8);
I2CBus KneeBus(1, PF_0, PF_1);
I2CBus AnkleBus(2, PD_13, PD_12);
I2CBus FootBus(3, PB_9, PB_8);

#ifndef DISABLE_ROS
ros::NodeHandle nh;
// DeviceManager device_manager(nh);
#else
DeviceManager device_manager;
#endif

#ifndef DISABLE_ROS
BMI_160 Imu(0x68, FootBus, nh, 0, "imu", "/devices/body");
std_msgs::String network_msg;
ros::Publisher network_pub("network_strings", &network_msg);
#else
BMI_160 Imu(0x68, FootBus, 0);
#endif

char hello_msg[50] = "";

int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
  nh.advertise(network_pub);
#endif

  if (!Imu.initialize())
  {
    sprintf(hello_msg, "Initialize Error");

#ifndef DISABLE_ROS
    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
    wait_ms(1000);
#else
    printf("%s\n", hello_msg);
#endif
    wait_ms(1000);
  }

  while (1)
  {
    if (Imu.ping(BMI_160::REGISTER_ADDRESS::CHIP_ID))
    {
      sprintf(hello_msg, "Chip Id is: %x", Imu.getChipId());
    }
    else
    {
      sprintf(hello_msg, "Error");
    }

#ifndef DISABLE_ROS
    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
#else
    printf("%s\n", hello_msg);
#endif
    wait_ms(1000);
  }

  return 0;
}