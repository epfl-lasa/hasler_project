#ifndef _ROS_SERVICE_gripperSetStateSrv_h
#define _ROS_SERVICE_gripperSetStateSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs_gripper
{

static const char GRIPPERSETSTATESRV[] = "custom_msgs_gripper/gripperSetStateSrv";

  class gripperSetStateSrvRequest : public ros::Msg
  {
    public:
      typedef int8_t _ros_gripperMachineState_type;
      _ros_gripperMachineState_type ros_gripperMachineState;

    gripperSetStateSrvRequest():
      ros_gripperMachineState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_gripperMachineState;
      u_ros_gripperMachineState.real = this->ros_gripperMachineState;
      *(outbuffer + offset + 0) = (u_ros_gripperMachineState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_gripperMachineState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_gripperMachineState;
      u_ros_gripperMachineState.base = 0;
      u_ros_gripperMachineState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_gripperMachineState = u_ros_gripperMachineState.real;
      offset += sizeof(this->ros_gripperMachineState);
     return offset;
    }

    const char * getType(){ return GRIPPERSETSTATESRV; };
    const char * getMD5(){ return "07cbdb76b8fbc25b360ae89ace9f97f6"; };

  };

  class gripperSetStateSrvResponse : public ros::Msg
  {
    public:
      typedef bool _gripper_newState_type;
      _gripper_newState_type gripper_newState;

    gripperSetStateSrvResponse():
      gripper_newState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gripper_newState;
      u_gripper_newState.real = this->gripper_newState;
      *(outbuffer + offset + 0) = (u_gripper_newState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_newState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gripper_newState;
      u_gripper_newState.base = 0;
      u_gripper_newState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_newState = u_gripper_newState.real;
      offset += sizeof(this->gripper_newState);
     return offset;
    }

    const char * getType(){ return GRIPPERSETSTATESRV; };
    const char * getMD5(){ return "0f0d6a6ff05f3dd6f14969e84835312d"; };

  };

  class gripperSetStateSrv {
    public:
    typedef gripperSetStateSrvRequest Request;
    typedef gripperSetStateSrvResponse Response;
  };

}
#endif
