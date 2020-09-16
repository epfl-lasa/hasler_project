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
      *(outbuffer + offset + 0) = (this->counter >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->counter >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->counter >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->counter >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->counter >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->counter >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->counter >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->counter >> (8 * 7)) & 0xFF;
      offset += sizeof(this->counter);
      *(outbuffer + offset + 0) = (this->duration >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->duration >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->duration >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->duration >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->duration >> (8 * 7)) & 0xFF;
      offset += sizeof(this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->counter =  ((uint64_t) (*(inbuffer + offset)));
      this->counter |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->counter |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->counter);
      this->duration =  ((uint64_t) (*(inbuffer + offset)));
      this->duration |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->duration |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return "any_msgs/SensorTimeInfo"; };
    const char * getMD5(){ return "5e7899e9fdd164cac2eb207cb4fe8d86"; };

  };

}
#endif