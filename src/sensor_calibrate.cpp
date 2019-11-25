/**
 * @file sensor_calibrate.cpp
 * @author Surya Murugavel Ravishankar (smurugavelravish@wpi.edu)
 * @brief
 * @version 0.1
 * @date 2019-11-10
 *
 * @copyright Copyright (c) 2019
 *
 */
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
ros::Publisher network_pub("/network", &network_msg);
#else
BendSensor bend_sensor(0, PrimaryBus, 1);
#endif
typedef enum
{
  ADS_CALIBRATE_FIRST = 0,  // First calibration point, typically 0 degrees
  ADS_CALIBRATE_SECOND,     // Second calibration point, 45-255 degrees
  ADS_CALIBRATE_CLEAR,  // Clears user calibration, restores factory calibration
  ADS_CALIBRATE_STRETCH_ZERO,    // 0mm strain calibration point
  ADS_CALIBRATE_STRETCH_SECOND,  // Second calibration point for stretch,
                                 // typically 30mm
} ADS_CALIBRATION_STEP_T;
float rate = 1;
// 1000 * 1 / 50;
int main()
{
#ifndef DISABLE_ROS
  nh.initNode();
  nh.advertise(network_pub);
#endif

  volatile bool is_init = false;
  int i = 1;

  wait_ms(5000);

  while (is_init)
  {
    is_init = bend_sensor.ping();
  }

  if (is_init)
  {
    // printf("Calibrating: \n");
    // Restore factory calibration coefficients
    wait_ms(1000);
    bend_sensor.calibrate(ADS_CALIBRATE_CLEAR, 0);
  }

  wait_ms(2000);

  while (1)
  {
    char data[50];
    sprintf(data, "calibrated!!!");
    network_msg.data = data;
    network_pub.publish(&network_msg);
    wait_ms(rate * 10);
    nh.spinOnce();
  }
  return 0;
}