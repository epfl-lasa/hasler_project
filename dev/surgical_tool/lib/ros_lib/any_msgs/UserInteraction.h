#ifndef _ROS_SERVICE_UserInteraction_h
#define _ROS_SERVICE_UserInteraction_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "any_msgs/UserInteractionOption.h"

namespace any_msgs
{

static const char USERINTERACTION[] = "any_msgs/UserInteraction";

  class UserInteractionRequest : public ros::Msg
  {
    public:
      typedef const char* _query_type;
      _query_type query;
      typedef any_msgs::UserInteractionOption _default_option_type;
      _default_option_type default_option;
      uint32_t options_length;
      typedef any_msgs::UserInteractionOption _options_type;
      _options_type st_options;
      _options_type * options;

    UserInteractionRequest():
      query(""),
      default_option(),
      options_length(0), options(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_query = strlen(this->query);
      varToArr(outbuffer + offset, length_query);
      offset += 4;
      memcpy(outbuffer + offset, this->query, length_query);
      offset += length_query;
      offset += this->default_option.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->options_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->options_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->options_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->options_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->options_length);
      for( uint32_t i = 0; i < options_length; i++){
      offset += this->options[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_query;
      arrToVar(length_query, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_query; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_query-1]=0;
      this->query = (char *)(inbuffer + offset-1);
      offset += length_query;
      offset += this->default_option.deserialize(inbuffer + offset);
      uint32_t options_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->options_length);
      if(options_lengthT > options_length)
        this->options = (any_msgs::UserInteractionOption*)realloc(this->options, options_lengthT * sizeof(any_msgs::UserInteractionOption));
      options_length = options_lengthT;
      for( uint32_t i = 0; i < options_length; i++){
      offset += this->st_options.deserialize(inbuffer + offset);
        memcpy( &(this->options[i]), &(this->st_options), sizeof(any_msgs::UserInteractionOption));
      }
     return offset;
    }

    const char * getType(){ return USERINTERACTION; };
    const char * getMD5(){ return "dd8cd75d9a62fe06db1189918163c02e"; };

  };

  class UserInteractionResponse : public ros::Msg
  {
    public:
      typedef any_msgs::UserInteractionOption _selected_option_type;
      _selected_option_type selected_option;

    UserInteractionResponse():
      selected_option()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->selected_option.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->selected_option.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return USERINTERACTION; };
    const char * getMD5(){ return "930fe615090363f75356e0b5b4624682"; };

  };

  class UserInteraction {
    public:
    typedef UserInteractionRequest Request;
    typedef UserInteractionResponse Response;
  };

}
#endif
