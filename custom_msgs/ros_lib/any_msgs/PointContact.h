#ifndef _ROS_any_msgs_PointContact_h
#define _ROS_any_msgs_PointContact_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

namespace any_msgs
{

  class PointContact : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Wrench _wrench_type;
      _wrench_type wrench;
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef geometry_msgs::Vector3 _normal_type;
      _normal_type normal;
      typedef uint8_t _state_type;
      _state_type state;
      enum { STATE_OPEN = 0 };
      enum { STATE_CLOSED = 1 };
      enum { STATE_SLIPPING = 2 };

    PointContact():
      header(),
      wrench(),
      position(),
      twist(),
      normal(),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->wrench.serialize(outbuffer + offset);
      offset += this->position.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      offset += this->normal.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->wrench.deserialize(inbuffer + offset);
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
      offset += this->normal.deserialize(inbuffer + offset);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "any_msgs/PointContact"; };
    const char * getMD5(){ return "c6b0ca2e6ced57468c5f3d70b9b31dda"; };

  };

}
#endif