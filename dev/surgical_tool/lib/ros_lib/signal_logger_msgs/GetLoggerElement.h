#ifndef _ROS_SERVICE_GetLoggerElement_h
#define _ROS_SERVICE_GetLoggerElement_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "signal_logger_msgs/LogElement.h"

namespace signal_logger_msgs
{

static const char GETLOGGERELEMENT[] = "signal_logger_msgs/GetLoggerElement";

  class GetLoggerElementRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;

    GetLoggerElementRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return GETLOGGERELEMENT; };
    const char * getMD5(){ return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

  class GetLoggerElementResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef signal_logger_msgs::LogElement _log_element_type;
      _log_element_type log_element;

    GetLoggerElementResponse():
      success(0),
      log_element()
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
      offset += this->log_element.serialize(outbuffer + offset);
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
      offset += this->log_element.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETLOGGERELEMENT; };
    const char * getMD5(){ return "7cd293aa13a069b6ecedbd227c209263"; };

  };

  class GetLoggerElement {
    public:
    typedef GetLoggerElementRequest Request;
    typedef GetLoggerElementResponse Response;
  };

}
#endif
