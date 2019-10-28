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
StrainGauge strain_gauge(0, p15, nh, 1, "strain_gauge",
                         "/devices/index/strain_gauge_1");
std_msgs::String network_msg;
ros::Publisher network_pub("network_strings", &network_msg);
#else
AnalogDevice ReadStrain(0, p15, 1);
#endif

char hello_msg[50] = "";

int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
  nh.advertise(network_pub);
#endif

  strain_gauge.calibrate();
  sprintf(hello_msg, "Calibrated value is: %f",
          strain_gauge.getUnstrainedVoltage());

  while (1)
  {
    strain_gauge.update();
    sprintf(hello_msg, "Strain value is: %f", strain_gauge.getStrain());

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