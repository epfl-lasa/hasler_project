#ifndef _ROS_custom_msgs_FootInputMsg_h
#define _ROS_custom_msgs_FootInputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class FootInputMsg : public ros::Msg
  {
    public:
      typedef float _FxDes_type;
      _FxDes_type FxDes;
      typedef float _FyDes_type;
      _FyDes_type FyDes;
      typedef float _TphiDes_type;
      _TphiDes_type TphiDes;
      typedef float _TthetaDes_type;
      _TthetaDes_type TthetaDes;
      typedef float _TpsiDes_type;
      _TpsiDes_type TpsiDes;
      typedef int16_t _stateDes_type;
      _stateDes_type stateDes;

    FootInputMsg():
      FxDes(0),
      FyDes(0),
      TphiDes(0),
      TthetaDes(0),
      TpsiDes(0),
      stateDes(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_FxDes;
      u_FxDes.real = this->FxDes;
      *(outbuffer + offset + 0) = (u_FxDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FxDes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FxDes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FxDes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->FxDes);
      union {
        float real;
        uint32_t base;
      } u_FyDes;
      u_FyDes.real = this->FyDes;
      *(outbuffer + offset + 0) = (u_FyDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FyDes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FyDes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FyDes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->FyDes);
      union {
        float real;
        uint32_t base;
      } u_TphiDes;
      u_TphiDes.real = this->TphiDes;
      *(outbuffer + offset + 0) = (u_TphiDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_TphiDes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_TphiDes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_TphiDes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->TphiDes);
      union {
        float real;
        uint32_t base;
      } u_TthetaDes;
      u_TthetaDes.real = this->TthetaDes;
      *(outbuffer + offset + 0) = (u_TthetaDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_TthetaDes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_TthetaDes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_TthetaDes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->TthetaDes);
      union {
        float real;
        uint32_t base;
      } u_TpsiDes;
      u_TpsiDes.real = this->TpsiDes;
      *(outbuffer + offset + 0) = (u_TpsiDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_TpsiDes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_TpsiDes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_TpsiDes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->TpsiDes);
      union {
        int16_t real;
        uint16_t base;
      } u_stateDes;
      u_stateDes.real = this->stateDes;
      *(outbuffer + offset + 0) = (u_stateDes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stateDes.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->stateDes);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_FxDes;
      u_FxDes.base = 0;
      u_FxDes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FxDes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FxDes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FxDes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->FxDes = u_FxDes.real;
      offset += sizeof(this->FxDes);
      union {
        float real;
        uint32_t base;
      } u_FyDes;
      u_FyDes.base = 0;
      u_FyDes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FyDes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FyDes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FyDes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->FyDes = u_FyDes.real;
      offset += sizeof(this->FyDes);
      union {
        float real;
        uint32_t base;
      } u_TphiDes;
      u_TphiDes.base = 0;
      u_TphiDes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_TphiDes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_TphiDes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_TphiDes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->TphiDes = u_TphiDes.real;
      offset += sizeof(this->TphiDes);
      union {
        float real;
        uint32_t base;
      } u_TthetaDes;
      u_TthetaDes.base = 0;
      u_TthetaDes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_TthetaDes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_TthetaDes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_TthetaDes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->TthetaDes = u_TthetaDes.real;
      offset += sizeof(this->TthetaDes);
      union {
        float real;
        uint32_t base;
      } u_TpsiDes;
      u_TpsiDes.base = 0;
      u_TpsiDes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_TpsiDes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_TpsiDes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_TpsiDes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->TpsiDes = u_TpsiDes.real;
      offset += sizeof(this->TpsiDes);
      union {
        int16_t real;
        uint16_t base;
      } u_stateDes;
      u_stateDes.base = 0;
      u_stateDes.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stateDes.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stateDes = u_stateDes.real;
      offset += sizeof(this->stateDes);
     return offset;
    }

    const char * getType(){ return "custom_msgs/FootInputMsg"; };
    const char * getMD5(){ return "a39283cd0a6414e571a661f8b3a644bc"; };

  };

}
#endif