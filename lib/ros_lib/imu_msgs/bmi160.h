#ifndef _ROS_imu_msgs_bmi160_h
#define _ROS_imu_msgs_bmi160_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"

namespace imu_msgs
{

  class bmi160 : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _chip_id_type;
      _chip_id_type chip_id;
      std_msgs::Float32 gyro[3];
      std_msgs::Float32 acc[3];

    bmi160():
      header(),
      chip_id(0),
      gyro(),
      acc()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->chip_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->chip_id);
      for( uint32_t i = 0; i < 3; i++){
      offset += this->gyro[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->acc[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->chip_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->chip_id);
      for( uint32_t i = 0; i < 3; i++){
      offset += this->gyro[i].deserialize(inbuffer + offset);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += this->acc[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "imu_msgs/bmi160"; };
    const char * getMD5(){ return "fb7a34db91cd9c52bd9bc31ae6e1f9e2"; };

  };

}
#endif
