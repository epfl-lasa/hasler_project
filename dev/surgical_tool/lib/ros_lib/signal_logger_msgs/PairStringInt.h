#ifndef _ROS_signal_logger_msgs_PairStringInt_h
#define _ROS_signal_logger_msgs_PairStringInt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

  class PairStringInt : public ros::Msg
  {
    public:
      typedef const char* _first_type;
      _first_type first;
      typedef int32_t _second_type;
      _second_type second;

    PairStringInt():
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
        int32_t real;
        uint32_t base;
      } u_second;
      u_second.real = this->second;
      *(outbuffer + offset + 0) = (u_second.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_second.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_second.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_second.base >> (8 * 3)) & 0xFF;
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
        int32_t real;
        uint32_t base;
      } u_second;
      u_second.base = 0;
      u_second.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_second.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_second.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_second.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->second = u_second.real;
      offset += sizeof(this->second);
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/PairStringInt"; };
    const char * getMD5(){ return "8a23cde2ef2ca21bdd63f96b3ed8cf87"; };

  };

}
#endif