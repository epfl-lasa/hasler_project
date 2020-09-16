#ifndef _ROS_notification_msgs_Notification_h
#define _ROS_notification_msgs_Notification_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace notification_msgs
{

  class Notification : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _level_type;
      _level_type level;
      typedef uint32_t _code_type;
      _code_type code;
      typedef const char* _id_type;
      _id_type id;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _description_type;
      _description_type description;
      uint32_t output_devices_length;
      typedef char* _output_devices_type;
      _output_devices_type st_output_devices;
      _output_devices_type * output_devices;
      enum { LEVEL_DEBUG =  0 };
      enum { LEVEL_INFO =  1 };
      enum { LEVEL_WARN =  2 };
      enum { LEVEL_ERROR =  3 };
      enum { LEVEL_FATAL =  4 };

    Notification():
      header(),
      level(0),
      code(0),
      id(""),
      name(""),
      description(""),
      output_devices_length(0), output_devices(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->code >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->code >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->code >> (8 * 3)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
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
      *(outbuffer + offset + 0) = (this->output_devices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_devices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->output_devices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->output_devices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_devices_length);
      for( uint32_t i = 0; i < output_devices_length; i++){
      uint32_t length_output_devicesi = strlen(this->output_devices[i]);
      varToArr(outbuffer + offset, length_output_devicesi);
      offset += 4;
      memcpy(outbuffer + offset, this->output_devices[i], length_output_devicesi);
      offset += length_output_devicesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->level =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level);
      this->code =  ((uint32_t) (*(inbuffer + offset)));
      this->code |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->code |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->code |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->code);
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
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
      uint32_t output_devices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      output_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      output_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      output_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->output_devices_length);
      if(output_devices_lengthT > output_devices_length)
        this->output_devices = (char**)realloc(this->output_devices, output_devices_lengthT * sizeof(char*));
      output_devices_length = output_devices_lengthT;
      for( uint32_t i = 0; i < output_devices_length; i++){
      uint32_t length_st_output_devices;
      arrToVar(length_st_output_devices, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_output_devices; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_output_devices-1]=0;
      this->st_output_devices = (char *)(inbuffer + offset-1);
      offset += length_st_output_devices;
        memcpy( &(this->output_devices[i]), &(this->st_output_devices), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "notification_msgs/Notification"; };
    const char * getMD5(){ return "b9a3b6f708dc8da2ca8a0362a99462b4"; };

  };

}
#endif