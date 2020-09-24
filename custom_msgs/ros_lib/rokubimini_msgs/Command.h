#ifndef _ROS_rokubimini_msgs_Command_h
#define _ROS_rokubimini_msgs_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/WrenchStamped.h"

namespace rokubimini_msgs
{

  class Command : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _setAsZeroTare_type;
      _setAsZeroTare_type setAsZeroTare;
      typedef bool _resetTareLoad_type;
      _resetTareLoad_type resetTareLoad;
      typedef geometry_msgs::WrenchStamped _zeroTare_type;
      _zeroTare_type zeroTare;

    Command():
      header(),
      setAsZeroTare(0),
      resetTareLoad(0),
      zeroTare()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_setAsZeroTare;
      u_setAsZeroTare.real = this->setAsZeroTare;
      *(outbuffer + offset + 0) = (u_setAsZeroTare.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->setAsZeroTare);
      union {
        bool real;
        uint8_t base;
      } u_resetTareLoad;
      u_resetTareLoad.real = this->resetTareLoad;
      *(outbuffer + offset + 0) = (u_resetTareLoad.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->resetTareLoad);
      offset += this->zeroTare.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_setAsZeroTare;
      u_setAsZeroTare.base = 0;
      u_setAsZeroTare.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->setAsZeroTare = u_setAsZeroTare.real;
      offset += sizeof(this->setAsZeroTare);
      union {
        bool real;
        uint8_t base;
      } u_resetTareLoad;
      u_resetTareLoad.base = 0;
      u_resetTareLoad.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->resetTareLoad = u_resetTareLoad.real;
      offset += sizeof(this->resetTareLoad);
      offset += this->zeroTare.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rokubimini_msgs/Command"; };
    const char * getMD5(){ return "b4f166db90d9b4b36e2f798075fdf022"; };

  };

}
#endif