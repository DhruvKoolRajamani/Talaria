#include "mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include "devices/imu/bmi_160.h"

using namespace ros;

I2CBus BodyBus(0, PC_9, PA_8);
I2CBus KneeBus(1, PF_0, PF_1);
I2CBus AnkleBus(2, PD_13, PD_12);
I2CBus FootBus(3, PB_9, PB_8);

I2CBus Busses[4] = { BodyBus, KneeBus, AnkleBus, FootBus };
I2CBus* I2CBusses = Busses;

BMI_160 Imu(0x68, 3, 0);

NodeHandle nh;

std_msgs::String network_msg;
// std_msgs::Float32MultiArray gyro_data;
Publisher network_pub("network_strings", &network_msg);
// Publisher data_pub("Data floats", &gyro_data);
char hello_msg[50] = "";
float gdata[6];

int main()
{
  nh.initNode();
  nh.advertise(network_pub);
  // nh.advertise(data_pub);

  while (1)
  {
    if (Imu.ping() && Imu.readAcc())
    {
      sprintf(hello_msg, "Chip Id is: %x, %f", 
        Imu.getChipId(), Imu.getAccX());
      
      // gdata[0] = Imu.getGyroX();
      // gdata[1] = Imu.getGyroY();
      // gdata[2] = Imu.getGyroZ();
      // gdata[3] = Imu.getAccX();
      // gdata[4] = Imu.getAccY();
      // gdata[5] = Imu.getAccZ();
    }
    else
    {
      sprintf(hello_msg, "Error");
      
    }


    network_msg.data = hello_msg;
    // gyro_data.data = gdata;
    network_pub.publish(&network_msg);
    // data_pub.publish(&gyro_data);

    nh.spinOnce();
    wait_ms(1000);
  }

  return 0;
}