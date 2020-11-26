#ifndef _ROS_endoscope_feedback_SurgicalTaskStateMsg_h
#define _ROS_endoscope_feedback_SurgicalTaskStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace endoscope_feedback
{

  class SurgicalTaskStateMsg : public ros::Msg
  {
    public:
      uint8_t robotMode[2];

    SurgicalTaskStateMsg():
      robotMode()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->robotMode[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robotMode[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      this->robotMode[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robotMode[i]);
      }
     return offset;
    }

    const char * getType(){ return "endoscope_feedback/SurgicalTaskStateMsg"; };
    const char * getMD5(){ return "db94e7a91c11a7c4e667c93f446b7aa2"; };

  };

}
#endif