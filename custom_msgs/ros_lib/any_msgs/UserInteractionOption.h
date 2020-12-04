#ifndef _ROS_any_msgs_UserInteractionOption_h
#define _ROS_any_msgs_UserInteractionOption_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace any_msgs
{

  class UserInteractionOption : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _description_type;
      _description_type description;
      typedef bool _selectable_type;
      _selectable_type selectable;

    UserInteractionOption():
      name(""),
      description(""),
      selectable(0)
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
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      union {
        bool real;
        uint8_t base;
      } u_selectable;
      u_selectable.real = this->selectable;
      *(outbuffer + offset + 0) = (u_selectable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->selectable);
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
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      union {
        bool real;
        uint8_t base;
      } u_selectable;
      u_selectable.base = 0;
      u_selectable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->selectable = u_selectable.real;
      offset += sizeof(this->selectable);
     return offset;
    }

    const char * getType(){ return "any_msgs/UserInteractionOption"; };
    const char * getMD5(){ return "51df2b429a59d3e18be350172b4022d0"; };

  };

}
#endif