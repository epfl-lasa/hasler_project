#ifndef _ROS_signal_logger_msgs_Int64MultiArrayStamped_h
#define _ROS_signal_logger_msgs_Int64MultiArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int64MultiArray.h"

namespace signal_logger_msgs
{

  class Int64MultiArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Int64MultiArray _matrix_type;
      _matrix_type matrix;

    Int64MultiArrayStamped():
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

    const char * getType(){ return "signal_logger_msgs/Int64MultiArrayStamped"; };
    const char * getMD5(){ return "6cbeadc8180fe9f60ac85faa1b30d791"; };

  };

}
#endif