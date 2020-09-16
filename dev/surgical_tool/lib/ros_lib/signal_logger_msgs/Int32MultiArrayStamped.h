#ifndef _ROS_signal_logger_msgs_Int32MultiArrayStamped_h
#define _ROS_signal_logger_msgs_Int32MultiArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Int32MultiArray.h"

namespace signal_logger_msgs
{

  class Int32MultiArrayStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Int32MultiArray _matrix_type;
      _matrix_type matrix;

    Int32MultiArrayStamped():
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

    const char * getType(){ return "signal_logger_msgs/Int32MultiArrayStamped"; };
    const char * getMD5(){ return "c64738467fbd91e7a95ad790743cceb8"; };

  };

}
#endif