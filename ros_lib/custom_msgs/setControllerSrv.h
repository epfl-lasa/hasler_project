#ifndef _ROS_SERVICE_setControllerSrv_h
#define _ROS_SERVICE_setControllerSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

static const char SETCONTROLLERSRV[] = "custom_msgs/setControllerSrv";

  class setControllerSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _ros_controllerType_type;
      _ros_controllerType_type ros_controllerType;
      typedef bool _ros_defaultControl_type;
      _ros_defaultControl_type ros_defaultControl;
      typedef int8_t _ros_controlledAxis_type;
      _ros_controlledAxis_type ros_controlledAxis;
      float ros_posP[5];
      float ros_posI[5];
      float ros_posD[5];
      float ros_speedP[5];
      float ros_speedI[5];
      float ros_speedD[5];

    setControllerSrvRequest():
      ros_controllerType(0),
      ros_defaultControl(0),
      ros_controlledAxis(0),
      ros_posP(),
      ros_posI(),
      ros_posD(),
      ros_speedP(),
      ros_speedI(),
      ros_speedD()
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
        int8_t real;
        uint8_t base;
      } u_ros_controlledAxis;
      u_ros_controlledAxis.real = this->ros_controlledAxis;
      *(outbuffer + offset + 0) = (u_ros_controlledAxis.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_controlledAxis);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posPi;
      u_ros_posPi.real = this->ros_posP[i];
      *(outbuffer + offset + 0) = (u_ros_posPi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posPi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posPi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posPi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posP[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posIi;
      u_ros_posIi.real = this->ros_posI[i];
      *(outbuffer + offset + 0) = (u_ros_posIi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posIi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posIi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posIi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posI[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posDi;
      u_ros_posDi.real = this->ros_posD[i];
      *(outbuffer + offset + 0) = (u_ros_posDi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_posDi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_posDi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_posDi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_posD[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedPi;
      u_ros_speedPi.real = this->ros_speedP[i];
      *(outbuffer + offset + 0) = (u_ros_speedPi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedPi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedPi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedPi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedP[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedIi;
      u_ros_speedIi.real = this->ros_speedI[i];
      *(outbuffer + offset + 0) = (u_ros_speedIi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedIi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedIi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedIi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedI[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedDi;
      u_ros_speedDi.real = this->ros_speedD[i];
      *(outbuffer + offset + 0) = (u_ros_speedDi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_speedDi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_speedDi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_speedDi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ros_speedD[i]);
      }
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
        int8_t real;
        uint8_t base;
      } u_ros_controlledAxis;
      u_ros_controlledAxis.base = 0;
      u_ros_controlledAxis.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_controlledAxis = u_ros_controlledAxis.real;
      offset += sizeof(this->ros_controlledAxis);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posPi;
      u_ros_posPi.base = 0;
      u_ros_posPi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posPi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posPi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posPi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posP[i] = u_ros_posPi.real;
      offset += sizeof(this->ros_posP[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posIi;
      u_ros_posIi.base = 0;
      u_ros_posIi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posIi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posIi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posIi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posI[i] = u_ros_posIi.real;
      offset += sizeof(this->ros_posI[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_posDi;
      u_ros_posDi.base = 0;
      u_ros_posDi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_posDi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_posDi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_posDi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_posD[i] = u_ros_posDi.real;
      offset += sizeof(this->ros_posD[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedPi;
      u_ros_speedPi.base = 0;
      u_ros_speedPi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedPi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedPi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedPi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedP[i] = u_ros_speedPi.real;
      offset += sizeof(this->ros_speedP[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedIi;
      u_ros_speedIi.base = 0;
      u_ros_speedIi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedIi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedIi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedIi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedI[i] = u_ros_speedIi.real;
      offset += sizeof(this->ros_speedI[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_ros_speedDi;
      u_ros_speedDi.base = 0;
      u_ros_speedDi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_speedDi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_speedDi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_speedDi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ros_speedD[i] = u_ros_speedDi.real;
      offset += sizeof(this->ros_speedD[i]);
      }
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "f8d7ccb8e953156cb228c974e2ac2cfc"; };

  };

  class setControllerSrvResponse : public ros::Msg
  {
    public:
      typedef bool _platform_controlOk_type;
      _platform_controlOk_type platform_controlOk;

    setControllerSrvResponse():
      platform_controlOk(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_controlOk;
      u_platform_controlOk.real = this->platform_controlOk;
      *(outbuffer + offset + 0) = (u_platform_controlOk.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_controlOk);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_platform_controlOk;
      u_platform_controlOk.base = 0;
      u_platform_controlOk.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_controlOk = u_platform_controlOk.real;
      offset += sizeof(this->platform_controlOk);
     return offset;
    }

    const char * getType(){ return SETCONTROLLERSRV; };
    const char * getMD5(){ return "b3690307401e27c573abefe10dae9da3"; };

  };

  class setControllerSrv {
    public:
    typedef setControllerSrvRequest Request;
    typedef setControllerSrvResponse Response;
  };

}
#endif
