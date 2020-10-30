#ifndef _ROS_custom_msgs_FootOutputMsg_v3_h
#define _ROS_custom_msgs_FootOutputMsg_v3_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs
{

  class FootOutputMsg_v3 : public ros::Msg
  {
    public:
      typedef ros::Time _platform_stamp_type;
      _platform_stamp_type platform_stamp;
      typedef int8_t _platform_id_type;
      _platform_id_type platform_id;
      float platform_position[5];
      float platform_speed[5];
      float platform_effortRef[5];
      float platform_effortD[5];
      float platform_effortM[5];
      typedef int8_t _platform_machineState_type;
      _platform_machineState_type platform_machineState;
      typedef int8_t _platform_controllerType_type;
      _platform_controllerType_type platform_controllerType;

    FootOutputMsg_v3():
      platform_stamp(),
      platform_id(0),
      platform_position(),
      platform_speed(),
      platform_effortRef(),
      platform_effortD(),
      platform_effortM(),
      platform_machineState(0),
      platform_controllerType(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->platform_stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->platform_stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->platform_stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->platform_stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_stamp.sec);
      *(outbuffer + offset + 0) = (this->platform_stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->platform_stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->platform_stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->platform_stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_platform_id;
      u_platform_id.real = this->platform_id;
      *(outbuffer + offset + 0) = (u_platform_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_positioni;
      u_platform_positioni.real = this->platform_position[i];
      *(outbuffer + offset + 0) = (u_platform_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_platform_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_platform_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_platform_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_speedi;
      u_platform_speedi.real = this->platform_speed[i];
      *(outbuffer + offset + 0) = (u_platform_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_platform_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_platform_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_platform_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortRefi;
      u_platform_effortRefi.real = this->platform_effortRef[i];
      *(outbuffer + offset + 0) = (u_platform_effortRefi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_platform_effortRefi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_platform_effortRefi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_platform_effortRefi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_effortRef[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortDi;
      u_platform_effortDi.real = this->platform_effortD[i];
      *(outbuffer + offset + 0) = (u_platform_effortDi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_platform_effortDi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_platform_effortDi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_platform_effortDi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_effortD[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortMi;
      u_platform_effortMi.real = this->platform_effortM[i];
      *(outbuffer + offset + 0) = (u_platform_effortMi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_platform_effortMi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_platform_effortMi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_platform_effortMi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->platform_effortM[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_platform_machineState;
      u_platform_machineState.real = this->platform_machineState;
      *(outbuffer + offset + 0) = (u_platform_machineState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_machineState);
      union {
        int8_t real;
        uint8_t base;
      } u_platform_controllerType;
      u_platform_controllerType.real = this->platform_controllerType;
      *(outbuffer + offset + 0) = (u_platform_controllerType.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->platform_controllerType);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->platform_stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->platform_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->platform_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->platform_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->platform_stamp.sec);
      this->platform_stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->platform_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->platform_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->platform_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->platform_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_platform_id;
      u_platform_id.base = 0;
      u_platform_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_id = u_platform_id.real;
      offset += sizeof(this->platform_id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_positioni;
      u_platform_positioni.base = 0;
      u_platform_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_platform_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_platform_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_platform_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->platform_position[i] = u_platform_positioni.real;
      offset += sizeof(this->platform_position[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_speedi;
      u_platform_speedi.base = 0;
      u_platform_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_platform_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_platform_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_platform_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->platform_speed[i] = u_platform_speedi.real;
      offset += sizeof(this->platform_speed[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortRefi;
      u_platform_effortRefi.base = 0;
      u_platform_effortRefi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_platform_effortRefi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_platform_effortRefi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_platform_effortRefi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->platform_effortRef[i] = u_platform_effortRefi.real;
      offset += sizeof(this->platform_effortRef[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortDi;
      u_platform_effortDi.base = 0;
      u_platform_effortDi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_platform_effortDi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_platform_effortDi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_platform_effortDi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->platform_effortD[i] = u_platform_effortDi.real;
      offset += sizeof(this->platform_effortD[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_platform_effortMi;
      u_platform_effortMi.base = 0;
      u_platform_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_platform_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_platform_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_platform_effortMi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->platform_effortM[i] = u_platform_effortMi.real;
      offset += sizeof(this->platform_effortM[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_platform_machineState;
      u_platform_machineState.base = 0;
      u_platform_machineState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_machineState = u_platform_machineState.real;
      offset += sizeof(this->platform_machineState);
      union {
        int8_t real;
        uint8_t base;
      } u_platform_controllerType;
      u_platform_controllerType.base = 0;
      u_platform_controllerType.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->platform_controllerType = u_platform_controllerType.real;
      offset += sizeof(this->platform_controllerType);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootOutputMsg_v3"; };
    const char * getMD5(){ return "2008763e24ad7127d2bae04ecfb00678"; };

  };

}
#endif