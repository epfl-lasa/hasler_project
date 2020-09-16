#ifndef _ROS_signal_logger_msgs_PairStringIntStamped_h
#define _ROS_signal_logger_msgs_PairStringIntStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace signal_logger_msgs
{

  class PairStringIntStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _first_type;
      _first_type first;
      typedef int32_t _second_type;
      _second_type second;

    PairStringIntStamped():
      header(),
      first(""),
      second(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
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

    const char * getType(){ return "signal_logger_msgs/PairStringIntStamped"; };
    const char * getMD5(){ return "354b96523598f459aaadac055895eea6"; };

  };

}
#endif