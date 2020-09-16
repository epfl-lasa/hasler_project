#ifndef _ROS_SERVICE_String_cmd_h
#define _ROS_SERVICE_String_cmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace netft_rdt_driver
{

static const char STRING_CMD[] = "netft_rdt_driver/String_cmd";

  class String_cmdRequest : public ros::Msg
  {
    public:
      typedef const char* _cmd_type;
      _cmd_type cmd;

    String_cmdRequest():
      cmd("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_cmd = strlen(this->cmd);
      varToArr(outbuffer + offset, length_cmd);
      offset += 4;
      memcpy(outbuffer + offset, this->cmd, length_cmd);
      offset += length_cmd;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_cmd;
      arrToVar(length_cmd, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cmd; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cmd-1]=0;
      this->cmd = (char *)(inbuffer + offset-1);
      offset += length_cmd;
     return offset;
    }

    const char * getType(){ return STRING_CMD; };
    const char * getMD5(){ return "43a54fa49066cddcf148717d9d4a6353"; };

  };

  class String_cmdResponse : public ros::Msg
  {
    public:
      typedef const char* _res_type;
      _res_type res;

    String_cmdResponse():
      res("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_res = strlen(this->res);
      varToArr(outbuffer + offset, length_res);
      offset += 4;
      memcpy(outbuffer + offset, this->res, length_res);
      offset += length_res;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_res;
      arrToVar(length_res, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_res; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_res-1]=0;
      this->res = (char *)(inbuffer + offset-1);
      offset += length_res;
     return offset;
    }

    const char * getType(){ return STRING_CMD; };
    const char * getMD5(){ return "53af918a2a4a2a182c184142fff49b0c"; };

  };

  class String_cmd {
    public:
    typedef String_cmdRequest Request;
    typedef String_cmdResponse Response;
  };

}
#endif
