#ifndef _ROS_signal_logger_msgs_TimeStamped_h
#define _ROS_signal_logger_msgs_TimeStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"

namespace signal_logger_msgs
{

  class TimeStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Time _value_type;
      _value_type value;

    TimeStamped():
      header(),
      value()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->value.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->value.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/TimeStamped"; };
    const char * getMD5(){ return "fe554d022094846c9be27c052b932901"; };

  };

}
#endif