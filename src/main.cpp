#include "mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include "devices/imu/bmi_160.h"

using namespace ros;

// I2CBus BodyBus(0, PA_9, PA_8);
// I2CBus KneeBus(1, PF_0, PF_1);
// I2CBus AnkleBus(2, PD_13, PD_12);
I2CBus FootBus(3, PB_9, PB_8);

const int 

bool enabled = true;

// I2CBus Busses[4] = { BodyBus, KneeBus, AnkleBus, FootBus };
I2CBus* I2CBusses = &FootBus;

BMI_160 Imu(0x68, 3, 0);

NodeHandle nh;

std_msgs::String network_msg;
Publisher network_pub("network_strings", &network_msg);

char hello_msg[50] = "";

int main()
{
  nh.initNode();
  nh.advertise(network_pub);

  while (1)
  {
    if (Imu.ping() && enabled)
    {
      sprintf(hello_msg, "Chip Id is: %x\n", Imu.getChipId());
    }
    else
    {
      sprintf(hello_msg, "Error");
    }

    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
    wait_ms(1000);
  }

  return 0;
}