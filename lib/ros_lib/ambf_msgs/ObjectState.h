#ifndef _ROS_ambf_msgs_ObjectState_h
#define _ROS_ambf_msgs_ObjectState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"

namespace ambf_msgs
{

  class ObjectState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _sim_step_type;
      _sim_step_type sim_step;
      typedef std_msgs::String _name_type;
      _name_type name;
      typedef float _wall_time_type;
      _wall_time_type wall_time;
      typedef float _sim_time_type;
      _sim_time_type sim_time;
      typedef float _mass_type;
      _mass_type mass;
      typedef geometry_msgs::Point _pInertia_type;
      _pInertia_type pInertia;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Wrench _wrench_type;
      _wrench_type wrench;
      typedef const char* _userdata_description_type;
      _userdata_description_type userdata_description;
      uint32_t children_names_length;
      typedef char* _children_names_type;
      _children_names_type st_children_names;
      _children_names_type * children_names;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t userdata_length;
      typedef float _userdata_type;
      _userdata_type st_userdata;
      _userdata_type * userdata;
      uint32_t joint_positions_length;
      typedef float _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;

    ObjectState():
      header(),
      sim_step(0),
      name(),
      wall_time(0),
      sim_time(0),
      mass(0),
      pInertia(),
      pose(),
      wrench(),
      userdata_description(""),
      children_names_length(0), children_names(NULL),
      joint_names_length(0), joint_names(NULL),
      userdata_length(0), userdata(NULL),
      joint_positions_length(0), joint_positions(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sim_step >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sim_step >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sim_step >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sim_step >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sim_step);
      offset += this->name.serialize(outbuffer + offset);
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
      union {
        float real;
        uint32_t base;
      } u_mass;
      u_mass.real = this->mass;
      *(outbuffer + offset + 0) = (u_mass.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mass.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mass.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mass.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mass);
      offset += this->pInertia.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->wrench.serialize(outbuffer + offset);
      uint32_t length_userdata_description = strlen(this->userdata_description);
      varToArr(outbuffer + offset, length_userdata_description);
      offset += 4;
      memcpy(outbuffer + offset, this->userdata_description, length_userdata_description);
      offset += length_userdata_description;
      *(outbuffer + offset + 0) = (this->children_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->children_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->children_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->children_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->children_names_length);
      for( uint32_t i = 0; i < children_names_length; i++){
      uint32_t length_children_namesi = strlen(this->children_names[i]);
      varToArr(outbuffer + offset, length_children_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->children_names[i], length_children_namesi);
      offset += length_children_namesi;
      }
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->userdata_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->userdata_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->userdata_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->userdata_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->userdata_length);
      for( uint32_t i = 0; i < userdata_length; i++){
      union {
        float real;
        uint32_t base;
      } u_userdatai;
      u_userdatai.real = this->userdata[i];
      *(outbuffer + offset + 0) = (u_userdatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_userdatai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_userdatai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_userdatai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->userdata[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions_length);
      for( uint32_t i = 0; i < joint_positions_length; i++){
      union {
        float real;
        uint32_t base;
      } u_joint_positionsi;
      u_joint_positionsi.real = this->joint_positions[i];
      *(outbuffer + offset + 0) = (u_joint_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_positionsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->sim_step =  ((uint32_t) (*(inbuffer + offset)));
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sim_step |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sim_step);
      offset += this->name.deserialize(inbuffer + offset);
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
      union {
        float real;
        uint32_t base;
      } u_mass;
      u_mass.base = 0;
      u_mass.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mass.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mass.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mass.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mass = u_mass.real;
      offset += sizeof(this->mass);
      offset += this->pInertia.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->wrench.deserialize(inbuffer + offset);
      uint32_t length_userdata_description;
      arrToVar(length_userdata_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_userdata_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_userdata_description-1]=0;
      this->userdata_description = (char *)(inbuffer + offset-1);
      offset += length_userdata_description;
      uint32_t children_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      children_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      children_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      children_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->children_names_length);
      if(children_names_lengthT > children_names_length)
        this->children_names = (char**)realloc(this->children_names, children_names_lengthT * sizeof(char*));
      children_names_length = children_names_lengthT;
      for( uint32_t i = 0; i < children_names_length; i++){
      uint32_t length_st_children_names;
      arrToVar(length_st_children_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_children_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_children_names-1]=0;
      this->st_children_names = (char *)(inbuffer + offset-1);
      offset += length_st_children_names;
        memcpy( &(this->children_names[i]), &(this->st_children_names), sizeof(char*));
      }
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t userdata_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      userdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      userdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      userdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->userdata_length);
      if(userdata_lengthT > userdata_length)
        this->userdata = (float*)realloc(this->userdata, userdata_lengthT * sizeof(float));
      userdata_length = userdata_lengthT;
      for( uint32_t i = 0; i < userdata_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_userdata;
      u_st_userdata.base = 0;
      u_st_userdata.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_userdata.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_userdata.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_userdata.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_userdata = u_st_userdata.real;
      offset += sizeof(this->st_userdata);
        memcpy( &(this->userdata[i]), &(this->st_userdata), sizeof(float));
      }
      uint32_t joint_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_positions_length);
      if(joint_positions_lengthT > joint_positions_length)
        this->joint_positions = (float*)realloc(this->joint_positions, joint_positions_lengthT * sizeof(float));
      joint_positions_length = joint_positions_lengthT;
      for( uint32_t i = 0; i < joint_positions_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_joint_positions;
      u_st_joint_positions.base = 0;
      u_st_joint_positions.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_positions.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_positions.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_positions.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_joint_positions = u_st_joint_positions.real;
      offset += sizeof(this->st_joint_positions);
        memcpy( &(this->joint_positions[i]), &(this->st_joint_positions), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "ambf_msgs/ObjectState"; };
    const char * getMD5(){ return "5c41957ade5befe08be9e8f0ca23c5c4"; };

  };

}
#endif
