#ifndef _ROS_SERVICE_setStateSrv_v2_h
#define _ROS_SERVICE_setStateSrv_v2_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

static const char SETSTATESRV_V2[] = "custom_msgs/setStateSrv_v2";

  class setStateSrv_v2Request : public ros::Msg
  {
    public:
      typedef int8_t _ros_machineState_type;
      _ros_machineState_type ros_machineState;
      uint8_t ros_effortComp[5];

    setStateSrv_v2Request():
      ros_machineState(0),
      ros_effortComp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_machineState;
      u_ros_machineState.real = this->ros_machineState;
      *(outbuffer + offset + 0) = (u_ros_machineState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_machineState);
      for( uint32_t i = 0; i < 5; i++){
      *(outbuffer + offset + 0) = (this->ros_effortComp[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_effortComp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ros_machineState;
      u_ros_machineState.base = 0;
      u_ros_machineState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_machineState = u_ros_machineState.real;
      offset += sizeof(this->ros_machineState);
      for( uint32_t i = 0; i < 5; i++){
      this->ros_effortComp[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ros_effortComp[i]);
      }
     return offset;
    }

    const char * getType(){ return SETSTATESRV_V2; };
    const char * getMD5(){ return "92a66b19916ae01450b25761044d9bf0"; };

  };

  class setStateSrv_v2Response : public ros::Msg
  {
    public:
      typedef bool _platform_newState_type;
      _platform_newState_type platform_newState;

    setStateSrv_v2Response():
      platform_newState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_newState;
      u_platform_newState.real = this->platform_newState;
      *(outbuffer + offset + 0) = (u_platform_newState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_newState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_newState;
      u_platform_newState.base = 0;
      u_platform_newState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_newState = u_platform_newState.real;
      offset += sizeof(this->platform_newState);
     return offset;
    }

    const char * getType(){ return SETSTATESRV_V2; };
    const char * getMD5(){ return "11d99a622803a32975b5ee3ee7b6fdad"; };

  };

  class setStateSrv_v2 {
    public:
    typedef setStateSrv_v2Request Request;
    typedef setStateSrv_v2Response Response;
  };

}
#endif
