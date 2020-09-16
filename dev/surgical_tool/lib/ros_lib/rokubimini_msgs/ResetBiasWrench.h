#ifndef _ROS_SERVICE_ResetBiasWrench_h
#define _ROS_SERVICE_ResetBiasWrench_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rokubimini_msgs
{

static const char RESETBIASWRENCH[] = "rokubimini_msgs/ResetBiasWrench";

  class ResetBiasWrenchRequest : public ros::Msg
  {
    public:
      typedef int32_t _devid_type;
      _devid_type devid;

    ResetBiasWrenchRequest():
      devid(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_devid;
      u_devid.real = this->devid;
      *(outbuffer + offset + 0) = (u_devid.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_devid.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_devid.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_devid.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->devid);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_devid;
      u_devid.base = 0;
      u_devid.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_devid.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_devid.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_devid.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->devid = u_devid.real;
      offset += sizeof(this->devid);
     return offset;
    }

    const char * getType(){ return RESETBIASWRENCH; };
    const char * getMD5(){ return "899fc876fe79c419f24f892812c17e47"; };

  };

  class ResetBiasWrenchResponse : public ros::Msg
  {
    public:

    ResetBiasWrenchResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return RESETBIASWRENCH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetBiasWrench {
    public:
    typedef ResetBiasWrenchRequest Request;
    typedef ResetBiasWrenchResponse Response;
  };

}
#endif
