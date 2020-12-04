#ifndef _ROS_any_msgs_ImuWithTrigger_h
#define _ROS_any_msgs_ImuWithTrigger_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Imu.h"

namespace any_msgs
{

  class ImuWithTrigger : public ros::Msg
  {
    public:
      typedef sensor_msgs::Imu _imu_type;
      _imu_type imu;
      typedef bool _triggerIndicator_type;
      _triggerIndicator_type triggerIndicator;

    ImuWithTrigger():
      imu(),
      triggerIndicator(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->imu.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_triggerIndicator;
      u_triggerIndicator.real = this->triggerIndicator;
      *(outbuffer + offset + 0) = (u_triggerIndicator.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->triggerIndicator);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->imu.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_triggerIndicator;
      u_triggerIndicator.base = 0;
      u_triggerIndicator.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->triggerIndicator = u_triggerIndicator.real;
      offset += sizeof(this->triggerIndicator);
     return offset;
    }

    const char * getType(){ return "any_msgs/ImuWithTrigger"; };
    const char * getMD5(){ return "be412b7f053c3ac481ce4c68479b5267"; };

  };

}
#endif