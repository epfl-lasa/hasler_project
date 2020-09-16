#ifndef _ROS_SERVICE_SetLoggerElement_h
#define _ROS_SERVICE_SetLoggerElement_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "signal_logger_msgs/LogElement.h"

namespace signal_logger_msgs
{

static const char SETLOGGERELEMENT[] = "signal_logger_msgs/SetLoggerElement";

  class SetLoggerElementRequest : public ros::Msg
  {
    public:
      typedef signal_logger_msgs::LogElement _log_element_type;
      _log_element_type log_element;

    SetLoggerElementRequest():
      log_element()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->log_element.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->log_element.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETLOGGERELEMENT; };
    const char * getMD5(){ return "9131e3a1b950ffb93b5dd91562a083ce"; };

  };

  class SetLoggerElementResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetLoggerElementResponse():
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

    const char * getType(){ return SETLOGGERELEMENT; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetLoggerElement {
    public:
    typedef SetLoggerElementRequest Request;
    typedef SetLoggerElementResponse Response;
  };

}
#endif
