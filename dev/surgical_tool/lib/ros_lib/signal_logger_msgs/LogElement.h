#ifndef _ROS_signal_logger_msgs_LogElement_h
#define _ROS_signal_logger_msgs_LogElement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

  class LogElement : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef bool _is_logged_type;
      _is_logged_type is_logged;
      typedef uint32_t _divider_type;
      _divider_type divider;
      typedef uint8_t _action_type;
      _action_type action;
      typedef uint8_t _buffer_type_type;
      _buffer_type_type buffer_type;
      typedef uint32_t _buffer_size_type;
      _buffer_size_type buffer_size;
      typedef uint32_t _no_items_in_buffer_type;
      _no_items_in_buffer_type no_items_in_buffer;
      typedef uint32_t _no_unread_items_in_buffer_type;
      _no_unread_items_in_buffer_type no_unread_items_in_buffer;
      enum { ACTION_SAVE_AND_PUBLISH =   0 };
      enum { ACTION_SAVE =   1 };
      enum { ACTION_PUBLISH =   2 };
      enum { BUFFERTYPE_FIXED_SIZE =   0 };
      enum { BUFFERTYPE_LOOPING =   1 };
      enum { BUFFERTYPE_EXPONENTIALLY_GROWING =   2 };

    LogElement():
      name(""),
      is_logged(0),
      divider(0),
      action(0),
      buffer_type(0),
      buffer_size(0),
      no_items_in_buffer(0),
      no_unread_items_in_buffer(0)
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
      union {
        bool real;
        uint8_t base;
      } u_is_logged;
      u_is_logged.real = this->is_logged;
      *(outbuffer + offset + 0) = (u_is_logged.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_logged);
      *(outbuffer + offset + 0) = (this->divider >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->divider >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->divider >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->divider >> (8 * 3)) & 0xFF;
      offset += sizeof(this->divider);
      *(outbuffer + offset + 0) = (this->action >> (8 * 0)) & 0xFF;
      offset += sizeof(this->action);
      *(outbuffer + offset + 0) = (this->buffer_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buffer_type);
      *(outbuffer + offset + 0) = (this->buffer_size >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->buffer_size >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->buffer_size >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->buffer_size >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buffer_size);
      *(outbuffer + offset + 0) = (this->no_items_in_buffer >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->no_items_in_buffer >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->no_items_in_buffer >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->no_items_in_buffer >> (8 * 3)) & 0xFF;
      offset += sizeof(this->no_items_in_buffer);
      *(outbuffer + offset + 0) = (this->no_unread_items_in_buffer >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->no_unread_items_in_buffer >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->no_unread_items_in_buffer >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->no_unread_items_in_buffer >> (8 * 3)) & 0xFF;
      offset += sizeof(this->no_unread_items_in_buffer);
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
      union {
        bool real;
        uint8_t base;
      } u_is_logged;
      u_is_logged.base = 0;
      u_is_logged.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_logged = u_is_logged.real;
      offset += sizeof(this->is_logged);
      this->divider =  ((uint32_t) (*(inbuffer + offset)));
      this->divider |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->divider |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->divider |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->divider);
      this->action =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->action);
      this->buffer_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->buffer_type);
      this->buffer_size =  ((uint32_t) (*(inbuffer + offset)));
      this->buffer_size |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->buffer_size |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->buffer_size |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->buffer_size);
      this->no_items_in_buffer =  ((uint32_t) (*(inbuffer + offset)));
      this->no_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->no_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->no_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->no_items_in_buffer);
      this->no_unread_items_in_buffer =  ((uint32_t) (*(inbuffer + offset)));
      this->no_unread_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->no_unread_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->no_unread_items_in_buffer |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->no_unread_items_in_buffer);
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/LogElement"; };
    const char * getMD5(){ return "9dea7ae34ab5f949b6a61683e65bb248"; };

  };

}
#endif