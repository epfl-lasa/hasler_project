#ifndef _ROS_SERVICE_gripperSetControllerSrv_h
#define _ROS_SERVICE_gripperSetControllerSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs_gripper
{

static const char GRIPPERSETCONTROLLERSRV[] = "custom_msgs_gripper/gripperSetControllerSrv";

  class gripperSetControllerSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _ros_controllerType_type;
      _ros_controllerType_type ros_controllerType;
      typedef bool _ros_defaultControl_type;
      _ros_defaultControl_type ros_defaultControl;
      typedef float _ros_posP_type;
      _ros_posP_type ros_posP;
      typedef float _ros_posI_type;
      _ros_posI_type ros_posI;
      typedef float _ros_posD_type;
      _ros_posD_type ros_posD;
      typedef float _ros_speedP_type;
      _ros_speedP_type ros_speedP;
      typedef float _ros_speedI_type;
      _ros_speedI_type ros_speedI;
      typedef float _ros_speedD_type;
      _ros_speedD_type ros_speedD;

    gripperSetControllerSrvRequest():
      ros_controllerType(0),
      ros_defaultControl(0),
      ros_posP(0),
      ros_posI(0),
      ros_posD(0),
      ros_speedP(0),
      ros_speedI(0),
      ros_speedD(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ros_controllerType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_controllerType);
      union {
        bool real;
        uint8_t base;
      } u_ros_defaultControl;
      u_ros_defaultControl.real = this->ros_defaultControl;
      *(outbuffer + offset + 0) = (u_ros_defaultControl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_defaultControl);
      union {
        float real;
        uint32_t base;
      } u_ros_posP;
      u_ros_posP.real = this->ros_posP;
      *(outbuffer + offset + 0) = (u_ros_posP.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posP.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posP.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posP.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posP);
      union {
        float real;
        uint32_t base;
      } u_ros_posI;
      u_ros_posI.real = this->ros_posI;
      *(outbuffer + offset + 0) = (u_ros_posI.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posI.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posI.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posI.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posI);
      union {
        float real;
        uint32_t base;
      } u_ros_posD;
      u_ros_posD.real = this->ros_posD;
      *(outbuffer + offset + 0) = (u_ros_posD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posD);
      union {
        float real;
        uint32_t base;
      } u_ros_speedP;
      u_ros_speedP.real = this->ros_speedP;
      *(outbuffer + offset + 0) = (u_ros_speedP.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedP.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedP.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedP.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedP);
      union {
        float real;
        uint32_t base;
      } u_ros_speedI;
      u_ros_speedI.real = this->ros_speedI;
      *(outbuffer + offset + 0) = (u_ros_speedI.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedI.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedI.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedI.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedI);
      union {
        float real;
        uint32_t base;
      } u_ros_speedD;
      u_ros_speedD.real = this->ros_speedD;
      *(outbuffer + offset + 0) = (u_ros_speedD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedD);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->ros_controllerType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ros_controllerType);
      union {
        bool real;
        uint8_t base;
      } u_ros_defaultControl;
      u_ros_defaultControl.base = 0;
      u_ros_defaultControl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_defaultControl = u_ros_defaultControl.real;
      offset += sizeof(this->ros_defaultControl);
      union {
        float real;
        uint32_t base;
      } u_ros_posP;
      u_ros_posP.base = 0;
      u_ros_posP.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posP.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posP.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posP.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posP = u_ros_posP.real;
      offset += sizeof(this->ros_posP);
      union {
        float real;
        uint32_t base;
      } u_ros_posI;
      u_ros_posI.base = 0;
      u_ros_posI.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posI.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posI.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posI.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posI = u_ros_posI.real;
      offset += sizeof(this->ros_posI);
      union {
        float real;
        uint32_t base;
      } u_ros_posD;
      u_ros_posD.base = 0;
      u_ros_posD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posD = u_ros_posD.real;
      offset += sizeof(this->ros_posD);
      union {
        float real;
        uint32_t base;
      } u_ros_speedP;
      u_ros_speedP.base = 0;
      u_ros_speedP.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedP.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedP.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedP.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedP = u_ros_speedP.real;
      offset += sizeof(this->ros_speedP);
      union {
        float real;
        uint32_t base;
      } u_ros_speedI;
      u_ros_speedI.base = 0;
      u_ros_speedI.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedI.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedI.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedI.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedI = u_ros_speedI.real;
      offset += sizeof(this->ros_speedI);
      union {
        float real;
        uint32_t base;
      } u_ros_speedD;
      u_ros_speedD.base = 0;
      u_ros_speedD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedD = u_ros_speedD.real;
      offset += sizeof(this->ros_speedD);
     return offset;
    }

    const char * getType(){ return GRIPPERSETCONTROLLERSRV; };
    const char * getMD5(){ return "d9691a3c86680c4d734e7add80c31da4"; };

  };

  class gripperSetControllerSrvResponse : public ros::Msg
  {
    public:
      typedef bool _gripper_controlOk_type;
      _gripper_controlOk_type gripper_controlOk;

    gripperSetControllerSrvResponse():
      gripper_controlOk(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gripper_controlOk;
      u_gripper_controlOk.real = this->gripper_controlOk;
      *(outbuffer + offset + 0) = (u_gripper_controlOk.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_controlOk);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_gripper_controlOk;
      u_gripper_controlOk.base = 0;
      u_gripper_controlOk.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_controlOk = u_gripper_controlOk.real;
      offset += sizeof(this->gripper_controlOk);
     return offset;
    }

    const char * getType(){ return GRIPPERSETCONTROLLERSRV; };
    const char * getMD5(){ return "1e06f8b2e994f2ca616bec70da3deea1"; };

  };

  class gripperSetControllerSrv {
    public:
    typedef gripperSetControllerSrvRequest Request;
    typedef gripperSetControllerSrvResponse Response;
  };

}
#endif
