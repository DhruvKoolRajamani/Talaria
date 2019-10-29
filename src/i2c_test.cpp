#include "mbed.h"
#include <ros.h>
#include <std_msgs/String.h>

using namespace ros;

I2C i2c(PB_9, PB_8);


NodeHandle nh;

std_msgs::String network_msg;
Publisher network_pub("network_strings", &network_msg);

char hello_msg[50] = "";
uint8_t chip_reg = 0x00;
uint8_t chip_id = 0xD1;
uint8_t chip_addr = 0x68;
uint8_t buffer;
int main()
{
  nh.initNode();
  nh.advertise(network_pub);
  char packet[] = {static_cast<char>(chip_reg)};
  while (1)
  {

    if(i2c.write(chip_addr, packet, 1) == 0){
        i2c.read(chip_addr, reinterpret_cast<char *>(buffer), 1);
        if (chip_id == (uint8_t) buffer) {
            sprintf(hello_msg, "Chip Id is: %x", chip_id);
        }
        else{
          sprintf(hello_msg, "%x", (uint8_t) buffer);
        }
    }else {
        sprintf(hello_msg, "%x", (uint8_t) buffer);
    }
    
    
    

    network_msg.data = hello_msg;
    network_pub.publish(&network_msg);

    nh.spinOnce();
    wait_ms(1000);
  }

  return 0;
}