#ifndef _ROS_signal_logger_msgs_PairStringDouble_h
#define _ROS_signal_logger_msgs_PairStringDouble_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

  class PairStringDouble : public ros::Msg
  {
    public:
      typedef const char* _first_type;
      _first_type first;
      typedef double _second_type;
      _second_type second;

    PairStringDouble():
      first(""),
      second(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_first = strlen(this->first);
      varToArr(outbuffer + offset, length_first);
      offset += 4;
      memcpy(outbuffer + offset, this->first, length_first);
      offset += length_first;
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
      uint32_t length_first;
      arrToVar(length_first, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_first; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_first-1]=0;
      this->first = (char *)(inbuffer + offset-1);
      offset += length_first;
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

    const char * getType(){ return "signal_logger_msgs/PairStringDouble"; };
    const char * getMD5(){ return "61479750a8a2da2e629a5d4755c0076b"; };

  };

}
#endif