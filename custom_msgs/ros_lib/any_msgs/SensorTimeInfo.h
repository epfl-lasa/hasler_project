#ifndef _ROS_any_msgs_SensorTimeInfo_h
#define _ROS_any_msgs_SensorTimeInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace any_msgs
{

  class SensorTimeInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint64_t _counter_type;
      _counter_type counter;
      typedef uint64_t _duration_type;
      _duration_type duration;

    SensorTimeInfo():
      header(),
      counter(0),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        uint64_t real;
        uint32_t base;
      } u_counter;
      u_counter.real = this->counter;
      *(outbuffer + offset + 0) = (u_counter.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_counter.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_counter.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_counter.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->counter);
      union {
        uint64_t real;
        uint32_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        uint64_t real;
        uint32_t base;
      } u_counter;
      u_counter.base = 0;
      u_counter.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_counter.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_counter.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_counter.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->counter = u_counter.real;
      offset += sizeof(this->counter);
      union {
        uint64_t real;
        uint32_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return "any_msgs/SensorTimeInfo"; };
    const char * getMD5(){ return "5e7899e9fdd164cac2eb207cb4fe8d86"; };

  };

}
#endif