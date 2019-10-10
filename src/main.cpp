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
std_msgs::Float32MultiArray gyro_data;
Publisher network_pub("network_strings", &network_msg);
Publisher data_pub("Data floats", &gyro_data);
char hello_msg[50] = "";
float gdata[6];

int main()
{
  nh.initNode();
  nh.advertise(network_pub);
  nh.advertise(data_pub);

  while (1)
  {
    if (Imu.ping() && Imu.updateIMU())
    {
      sprintf(hello_msg, "Chip Id is: %x\n GyroX: %f GyroY: %f GyroZ: %f\n AccX: %f AccY: %f AccZ: %f\n", 
        Imu.getChipId(), Imu.getGyroX(), Imu.getGyroY(), Imu.getGyroZ(), Imu.getAccX(), Imu.getAccY(), Imu.getAccZ());
      
      gdata[0] = Imu.getGyroX();
      gdata[1] = Imu.getGyroY();
      gdata[2] = Imu.getGyroZ();
      gdata[3] = Imu.getAccX();
      gdata[4] = Imu.getAccY();
      gdata[5] = Imu.getAccZ();
    }
    else
    {
      sprintf(hello_msg, "Error");
      gdata[0] = -1.0;
      gdata[1] = -1.0;
      gdata[2] = -1.0;
      gdata[3] = -1.0;
      gdata[4] = -1.0;
      gdata[5] = -1.0;
    }


    network_msg.data = hello_msg;
    gyro_data.data = gdata;
    network_pub.publish(&network_msg);
    data_pub.publish(&gyro_data);

    nh.spinOnce();
    wait_ms(1000);
  }

  return 0;
}