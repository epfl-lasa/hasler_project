#ifndef _ROS_signal_logger_msgs_Float64MultiArrayStamped_h
#define _ROS_signal_logger_msgs_Float64MultiArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64MultiArray.h"

namespace signal_logger_msgs
{

  class Float64MultiArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Float64MultiArray _matrix_type;
      _matrix_type matrix;

    Float64MultiArrayStamped():
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

    const char * getType(){ return "signal_logger_msgs/Float64MultiArrayStamped"; };
    const char * getMD5(){ return "1bb5840a1e4eb0636e1e9c8f48693e1f"; };

  };

}
#endif