#ifndef _ROS_signal_logger_msgs_Int8MultiArrayStamped_h
#define _ROS_signal_logger_msgs_Int8MultiArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "signal_logger_msgs/Int8MultiArray.h"

namespace signal_logger_msgs
{

  class Int8MultiArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef signal_logger_msgs::Int8MultiArray _matrix_type;
      _matrix_type matrix;

    Int8MultiArrayStamped():
      header(),
      matrix()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->matrix.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->matrix.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/Int8MultiArrayStamped"; };
    const char * getMD5(){ return "f9f5db2c598aedd028887259efdc0491"; };

  };

}
#endif