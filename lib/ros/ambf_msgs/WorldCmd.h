#ifndef _ROS_ambf_msgs_WorldCmd_h
#define _ROS_ambf_msgs_WorldCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ambf_msgs
{

  class WorldCmd : public ros::Msg
  {
    public:
      typedef bool _enable_step_throttling_type;
      _enable_step_throttling_type enable_step_throttling;
      typedef bool _step_clock_type;
      _step_clock_type step_clock;
      typedef uint8_t _n_skip_steps_type;
      _n_skip_steps_type n_skip_steps;

    WorldCmd():
      enable_step_throttling(0),
      step_clock(0),
      n_skip_steps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable_step_throttling;
      u_enable_step_throttling.real = this->enable_step_throttling;
      *(outbuffer + offset + 0) = (u_enable_step_throttling.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_step_throttling);
      union {
        bool real;
        uint8_t base;
      } u_step_clock;
      u_step_clock.real = this->step_clock;
      *(outbuffer + offset + 0) = (u_step_clock.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->step_clock);
      *(outbuffer + offset + 0) = (this->n_skip_steps >> (8 * 0)) & 0xFF;
      offset += sizeof(this->n_skip_steps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_enable_step_throttling;
      u_enable_step_throttling.base = 0;
      u_enable_step_throttling.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_step_throttling = u_enable_step_throttling.real;
      offset += sizeof(this->enable_step_throttling);
      union {
        bool real;
        uint8_t base;
      } u_step_clock;
      u_step_clock.base = 0;
      u_step_clock.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->step_clock = u_step_clock.real;
      offset += sizeof(this->step_clock);
      this->n_skip_steps =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->n_skip_steps);
     return offset;
    }

    const char * getType(){ return "ambf_msgs/WorldCmd"; };
    const char * getMD5(){ return "6941ddbc8f8196cff4beb0278a6ad79d"; };

  };

}
#endif
