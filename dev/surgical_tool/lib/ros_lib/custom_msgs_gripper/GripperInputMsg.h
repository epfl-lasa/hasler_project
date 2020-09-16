#ifndef _ROS_custom_msgs_gripper_GripperInputMsg_h
#define _ROS_custom_msgs_gripper_GripperInputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs_gripper
{

  class GripperInputMsg : public ros::Msg
  {
    public:
      typedef float _ros_dPosition_type;
      _ros_dPosition_type ros_dPosition;
      typedef float _ros_dSpeed_type;
      _ros_dSpeed_type ros_dSpeed;

    GripperInputMsg():
      ros_dPosition(0),
      ros_dSpeed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ros_dPosition;
      u_ros_dPosition.real = this->ros_dPosition;
      *(outbuffer + offset + 0) = (u_ros_dPosition.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_dPosition.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_dPosition.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_dPosition.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_dPosition);
      union {
        float real;
        uint32_t base;
      } u_ros_dSpeed;
      u_ros_dSpeed.real = this->ros_dSpeed;
      *(outbuffer + offset + 0) = (u_ros_dSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_dSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_dSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_dSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_dSpeed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ros_dPosition;
      u_ros_dPosition.base = 0;
      u_ros_dPosition.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_dPosition.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_dPosition.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_dPosition.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_dPosition = u_ros_dPosition.real;
      offset += sizeof(this->ros_dPosition);
      union {
        float real;
        uint32_t base;
      } u_ros_dSpeed;
      u_ros_dSpeed.base = 0;
      u_ros_dSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_dSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_dSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_dSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_dSpeed = u_ros_dSpeed.real;
      offset += sizeof(this->ros_dSpeed);
     return offset;
    }

    const char * getType(){ return "custom_msgs_gripper/GripperInputMsg"; };
    const char * getMD5(){ return "ea6c7e01e15e5c8889e942e6a4d23751"; };

  };

}
#endif