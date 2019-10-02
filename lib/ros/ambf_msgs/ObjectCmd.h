#ifndef _ROS_ambf_msgs_ObjectCmd_h
#define _ROS_ambf_msgs_ObjectCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"

namespace ambf_msgs
{

  class ObjectCmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _enable_position_controller_type;
      _enable_position_controller_type enable_position_controller;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Wrench _wrench_type;
      _wrench_type wrench;
      uint32_t joint_cmds_length;
      typedef float _joint_cmds_type;
      _joint_cmds_type st_joint_cmds;
      _joint_cmds_type * joint_cmds;
      uint32_t position_controller_mask_length;
      typedef bool _position_controller_mask_type;
      _position_controller_mask_type st_position_controller_mask;
      _position_controller_mask_type * position_controller_mask;
      typedef bool _publish_children_names_type;
      _publish_children_names_type publish_children_names;
      typedef bool _publish_joint_names_type;
      _publish_joint_names_type publish_joint_names;
      typedef bool _publish_joint_positions_type;
      _publish_joint_positions_type publish_joint_positions;

    ObjectCmd():
      header(),
      enable_position_controller(0),
      pose(),
      wrench(),
      joint_cmds_length(0), joint_cmds(NULL),
      position_controller_mask_length(0), position_controller_mask(NULL),
      publish_children_names(0),
      publish_joint_names(0),
      publish_joint_positions(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_enable_position_controller;
      u_enable_position_controller.real = this->enable_position_controller;
      *(outbuffer + offset + 0) = (u_enable_position_controller.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_position_controller);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->wrench.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_cmds_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_cmds_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_cmds_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_cmds_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_cmds_length);
      for( uint32_t i = 0; i < joint_cmds_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_cmdsi;
      u_joint_cmdsi.real = this->joint_cmds[i];
      *(outbuffer + offset + 0) = (u_joint_cmdsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_cmdsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_cmdsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_cmdsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_cmds[i]);
      }
      *(outbuffer + offset + 0) = (this->position_controller_mask_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_controller_mask_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_controller_mask_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_controller_mask_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_controller_mask_length);
      for( uint32_t i = 0; i < position_controller_mask_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_position_controller_maski;
      u_position_controller_maski.real = this->position_controller_mask[i];
      *(outbuffer + offset + 0) = (u_position_controller_maski.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_controller_mask[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_publish_children_names;
      u_publish_children_names.real = this->publish_children_names;
      *(outbuffer + offset + 0) = (u_publish_children_names.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->publish_children_names);
      union {
        bool real;
        uint8_t base;
      } u_publish_joint_names;
      u_publish_joint_names.real = this->publish_joint_names;
      *(outbuffer + offset + 0) = (u_publish_joint_names.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->publish_joint_names);
      union {
        bool real;
        uint8_t base;
      } u_publish_joint_positions;
      u_publish_joint_positions.real = this->publish_joint_positions;
      *(outbuffer + offset + 0) = (u_publish_joint_positions.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->publish_joint_positions);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_enable_position_controller;
      u_enable_position_controller.base = 0;
      u_enable_position_controller.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_position_controller = u_enable_position_controller.real;
      offset += sizeof(this->enable_position_controller);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->wrench.deserialize(inbuffer + offset);
      uint32_t joint_cmds_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_cmds_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_cmds_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_cmds_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_cmds_length);
      if(joint_cmds_lengthT > joint_cmds_length)
        this->joint_cmds = (float*)realloc(this->joint_cmds, joint_cmds_lengthT * sizeof(float));
      joint_cmds_length = joint_cmds_lengthT;
      for( uint32_t i = 0; i < joint_cmds_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_cmds;
      u_st_joint_cmds.base = 0;
      u_st_joint_cmds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_cmds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_cmds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_cmds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_cmds = u_st_joint_cmds.real;
      offset += sizeof(this->st_joint_cmds);
        memcpy( &(this->joint_cmds[i]), &(this->st_joint_cmds), sizeof(float));
      }
      uint32_t position_controller_mask_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_controller_mask_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_controller_mask_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_controller_mask_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_controller_mask_length);
      if(position_controller_mask_lengthT > position_controller_mask_length)
        this->position_controller_mask = (bool*)realloc(this->position_controller_mask, position_controller_mask_lengthT * sizeof(bool));
      position_controller_mask_length = position_controller_mask_lengthT;
      for( uint32_t i = 0; i < position_controller_mask_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_position_controller_mask;
      u_st_position_controller_mask.base = 0;
      u_st_position_controller_mask.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_position_controller_mask = u_st_position_controller_mask.real;
      offset += sizeof(this->st_position_controller_mask);
        memcpy( &(this->position_controller_mask[i]), &(this->st_position_controller_mask), sizeof(bool));
      }
      union {
        bool real;
        uint8_t base;
      } u_publish_children_names;
      u_publish_children_names.base = 0;
      u_publish_children_names.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->publish_children_names = u_publish_children_names.real;
      offset += sizeof(this->publish_children_names);
      union {
        bool real;
        uint8_t base;
      } u_publish_joint_names;
      u_publish_joint_names.base = 0;
      u_publish_joint_names.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->publish_joint_names = u_publish_joint_names.real;
      offset += sizeof(this->publish_joint_names);
      union {
        bool real;
        uint8_t base;
      } u_publish_joint_positions;
      u_publish_joint_positions.base = 0;
      u_publish_joint_positions.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->publish_joint_positions = u_publish_joint_positions.real;
      offset += sizeof(this->publish_joint_positions);
     return offset;
    }

    const char * getType(){ return "ambf_msgs/ObjectCmd"; };
    const char * getMD5(){ return "091c255bd5039de21056e952538cf83c"; };

  };

}
#endif
