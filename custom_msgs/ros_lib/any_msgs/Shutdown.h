#ifndef _ROS_SERVICE_Shutdown_h
#define _ROS_SERVICE_Shutdown_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace any_msgs
{

static const char SHUTDOWN[] = "any_msgs/Shutdown";

  class ShutdownRequest : public ros::Msg
  {
    public:
      typedef uint16_t _type_type;
      _type_type type;
      enum { FULL_SHUTDOWN =  1                   };
      enum { MOTORS_POWER_SHUTDOWN =  2           };
      enum { SMART_FULL_SHUTDOWN =  3             };
      enum { SMART_MOTORS_POWER_SHUTDOWN =  4     };
      enum { REST_MANEUVER =  5                   };
      enum { STANDUP_MANEUVER =  6                };
      enum { MOTORS_POWER_STARTUP =  7	           };
      enum { SMART_MOTORS_POWER_STARTUP =  8	   };

    ShutdownRequest():
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->type >> (8 * 1)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->type =  ((uint16_t) (*(inbuffer + offset)));
      this->type |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->type);
     return offset;
    }

    const char * getType(){ return SHUTDOWN; };
    const char * getMD5(){ return "9baf7e346edb59804fd1c23dacff1943"; };

  };

  class ShutdownResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    ShutdownResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SHUTDOWN; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class Shutdown {
    public:
    typedef ShutdownRequest Request;
    typedef ShutdownResponse Response;
  };

}
#endif
