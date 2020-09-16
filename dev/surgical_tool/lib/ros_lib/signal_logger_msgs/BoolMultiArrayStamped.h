#ifndef _ROS_signal_logger_msgs_BoolMultiArrayStamped_h
#define _ROS_signal_logger_msgs_BoolMultiArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "signal_logger_msgs/BoolMultiArray.h"

namespace signal_logger_msgs
{

  class BoolMultiArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef signal_logger_msgs::BoolMultiArray _matrix_type;
      _matrix_type matrix;

    BoolMultiArrayStamped():
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

    const char * getType(){ return "signal_logger_msgs/BoolMultiArrayStamped"; };
    const char * getMD5(){ return "2d9fa6a74cba1d58bc182be7fdf2b9ff"; };

  };

}
#endif