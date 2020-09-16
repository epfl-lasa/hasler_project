#ifndef _ROS_signal_logger_msgs_PairIntDouble_h
#define _ROS_signal_logger_msgs_PairIntDouble_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

  class PairIntDouble : public ros::Msg
  {
    public:
      typedef int32_t _first_type;
      _first_type first;
      typedef double _second_type;
      _second_type second;

    PairIntDouble():
      first(0),
      second(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_first;
      u_first.real = this->first;
      *(outbuffer + offset + 0) = (u_first.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_first.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_first.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_first.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->first);
      union {
        double real;
        uint64_t base;
      } u_second;
      u_second.real = this->second;
      *(outbuffer + offset + 0) = (u_second.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_second.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_second.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_second.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_second.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_second.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_second.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_second.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->second);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_first;
      u_first.base = 0;
      u_first.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_first.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_first.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_first.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->first = u_first.real;
      offset += sizeof(this->first);
      union {
        double real;
        uint64_t base;
      } u_second;
      u_second.base = 0;
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_second.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->second = u_second.real;
      offset += sizeof(this->second);
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/PairIntDouble"; };
    const char * getMD5(){ return "b738ef892416c0794b1ea21e23f3e348"; };

  };

}
#endif