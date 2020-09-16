#ifndef _ROS_SERVICE_EditLoggerScript_h
#define _ROS_SERVICE_EditLoggerScript_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

static const char EDITLOGGERSCRIPT[] = "signal_logger_msgs/EditLoggerScript";

  class EditLoggerScriptRequest : public ros::Msg
  {
    public:
      typedef const char* _filepath_type;
      _filepath_type filepath;

    EditLoggerScriptRequest():
      filepath("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_filepath = strlen(this->filepath);
      varToArr(outbuffer + offset, length_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->filepath, length_filepath);
      offset += length_filepath;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_filepath;
      arrToVar(length_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filepath-1]=0;
      this->filepath = (char *)(inbuffer + offset-1);
      offset += length_filepath;
     return offset;
    }

    const char * getType(){ return EDITLOGGERSCRIPT; };
    const char * getMD5(){ return "5ef967a25f780d4a216c15b3834dca97"; };

  };

  class EditLoggerScriptResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    EditLoggerScriptResponse():
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

    const char * getType(){ return EDITLOGGERSCRIPT; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class EditLoggerScript {
    public:
    typedef EditLoggerScriptRequest Request;
    typedef EditLoggerScriptResponse Response;
  };

}
#endif
