#ifndef _ROS_ambf_msgs_WorldState_h
#define _ROS_ambf_msgs_WorldState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ambf_msgs
{

  class WorldState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint64_t _dynamic_loop_freq_type;
      _dynamic_loop_freq_type dynamic_loop_freq;
      typedef uint8_t _n_devices_type;
      _n_devices_type n_devices;
      typedef uint32_t _sim_step_type;
      _sim_step_type sim_step;
      typedef float _wall_time_type;
      _wall_time_type wall_time;
      typedef float _sim_time_type;
      _sim_time_type sim_time;

    WorldState():
      header(),
      dynamic_loop_freq(0),
      n_devices(0),
      sim_step(0),
      wall_time(0),
      sim_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        uint64_t real;
        uint32_t base;
      } u_dynamic_loop_freq;
      u_dynamic_loop_freq.real = this->dynamic_loop_freq;
      *(outbuffer + offset + 0) = (u_dynamic_loop_freq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dynamic_loop_freq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dynamic_loop_freq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dynamic_loop_freq.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dynamic_loop_freq);
      *(outbuffer + offset + 0) = (this->n_devices >> (8 * 0)) & 0xFF;
      offset += sizeof(this->n_devices);
      *(outbuffer + offset + 0) = (this->sim_step >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sim_step >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sim_step >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sim_step >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sim_step);
      union {
        float real;
        uint32_t base;
      } u_wall_time;
      u_wall_time.real = this->wall_time;
      *(outbuffer + offset + 0) = (u_wall_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wall_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wall_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wall_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wall_time);
      union {
        float real;
        uint32_t base;
      } u_sim_time;
      u_sim_time.real = this->sim_time;
      *(outbuffer + offset + 0) = (u_sim_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sim_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sim_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sim_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sim_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        uint64_t real;
        uint32_t base;
      } u_dynamic_loop_freq;
      u_dynamic_loop_freq.base = 0;
      u_dynamic_loop_freq.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dynamic_loop_freq.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dynamic_loop_freq.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dynamic_loop_freq.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dynamic_loop_freq = u_dynamic_loop_freq.real;
      offset += sizeof(this->dynamic_loop_freq);
      this->n_devices =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->n_devices);
      this->sim_step =  ((uint32_t) (*(inbuffer + offset)));
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sim_step);
      union {
        float real;
        uint32_t base;
      } u_wall_time;
      u_wall_time.base = 0;
      u_wall_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wall_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wall_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wall_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wall_time = u_wall_time.real;
      offset += sizeof(this->wall_time);
      union {
        float real;
        uint32_t base;
      } u_sim_time;
      u_sim_time.base = 0;
      u_sim_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sim_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sim_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sim_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sim_time = u_sim_time.real;
      offset += sizeof(this->sim_time);
     return offset;
    }

    const char * getType(){ return "ambf_msgs/WorldState"; };
    const char * getMD5(){ return "19e71cae5899074bfba211b483c66a1f"; };

  };

}
#endif
