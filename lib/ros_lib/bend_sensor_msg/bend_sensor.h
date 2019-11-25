#ifndef _ROS_bend_sensor_msg_bend_sensor_h
#define _ROS_bend_sensor_msg_bend_sensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

namespace bend_sensor_msg
{

  class bend_sensor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::UInt8 _chip_id_type;
      _chip_id_type chip_id;
      typedef std_msgs::String _debug_type;
      _debug_type debug;
      typedef std_msgs::Float32 _bend_type;
      _bend_type bend;
      typedef std_msgs::Float32 _stretch_type;
      _stretch_type stretch;

    bend_sensor():
      header(),
      chip_id(),
      debug(),
      bend(),
      stretch()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->chip_id.serialize(outbuffer + offset);
      offset += this->debug.serialize(outbuffer + offset);
      offset += this->bend.serialize(outbuffer + offset);
      offset += this->stretch.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->chip_id.deserialize(inbuffer + offset);
      offset += this->debug.deserialize(inbuffer + offset);
      offset += this->bend.deserialize(inbuffer + offset);
      offset += this->stretch.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "bend_sensor_msg/bend_sensor"; };
    const char * getMD5(){ return "12f9e693274fd6a1897924a19b68498d"; };

  };

}
#endif
