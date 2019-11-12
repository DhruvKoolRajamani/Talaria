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
#endif

  volatile bool is_init = false;
  int i = 1;
  int calibrate = 1;
  while (1)
  {
    if (!is_init)
    {
      bend_sensor.softReset();
      wait_ms(1000);
      is_init = bend_sensor.initialize();
    }

    if (is_init && calibrate)
    {
      printf("Calibrating: \n");
      // Restore factory calibration coefficients
      bend_sensor.calibrate(ADS_CALIBRATE_CLEAR, 0);
      wait_ms(2000);
      printf("Take first calibration point at zero degrees\n");
      wait_ms(2000);
      bend_sensor.calibrate(ADS_CALIBRATE_FIRST, 0);
      printf("Done\n");
      wait_ms(2000);
      printf("Take second calibration point at ninety degrees\n");
      wait_ms(2000);
      bend_sensor.calibrate(ADS_CALIBRATE_SECOND, 90);
      printf("Done\n");
      wait_ms(2000);
      printf("Calibrate the zero millimeter linear displacement\n");
      wait_ms(2000);
      bend_sensor.calibrate(ADS_CALIBRATE_STRETCH_ZERO, 0);
      printf("Done\n");
      wait_ms(2000);
      printf("Calibrate the 30 millimeter linear displacement make sure bend "
             "is 0\n");
      wait_ms(2000);
      bend_sensor.calibrate(ADS_CALIBRATE_STRETCH_SECOND, 30);
      printf("Done\n");
      wait_ms(2000);
      calibrate = 0;
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