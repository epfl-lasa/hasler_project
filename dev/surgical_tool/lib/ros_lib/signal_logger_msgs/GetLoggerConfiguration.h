#ifndef _ROS_SERVICE_GetLoggerConfiguration_h
#define _ROS_SERVICE_GetLoggerConfiguration_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

static const char GETLOGGERCONFIGURATION[] = "signal_logger_msgs/GetLoggerConfiguration";

  class GetLoggerConfigurationRequest : public ros::Msg
  {
    public:

    GetLoggerConfigurationRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETLOGGERCONFIGURATION; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetLoggerConfigurationResponse : public ros::Msg
  {
    public:
      typedef const char* _logger_namespace_type;
      _logger_namespace_type logger_namespace;
      typedef const char* _script_filepath_type;
      _script_filepath_type script_filepath;
      uint32_t log_element_names_length;
      typedef char* _log_element_names_type;
      _log_element_names_type st_log_element_names;
      _log_element_names_type * log_element_names;
      typedef double _collect_frequency_type;
      _collect_frequency_type collect_frequency;

    GetLoggerConfigurationResponse():
      logger_namespace(""),
      script_filepath(""),
      log_element_names_length(0), log_element_names(NULL),
      collect_frequency(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_logger_namespace = strlen(this->logger_namespace);
      varToArr(outbuffer + offset, length_logger_namespace);
      offset += 4;
      memcpy(outbuffer + offset, this->logger_namespace, length_logger_namespace);
      offset += length_logger_namespace;
      uint32_t length_script_filepath = strlen(this->script_filepath);
      varToArr(outbuffer + offset, length_script_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->script_filepath, length_script_filepath);
      offset += length_script_filepath;
      *(outbuffer + offset + 0) = (this->log_element_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->log_element_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->log_element_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->log_element_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->log_element_names_length);
      for( uint32_t i = 0; i < log_element_names_length; i++){
      uint32_t length_log_element_namesi = strlen(this->log_element_names[i]);
      varToArr(outbuffer + offset, length_log_element_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->log_element_names[i], length_log_element_namesi);
      offset += length_log_element_namesi;
      }
      union {
        double real;
        uint64_t base;
      } u_collect_frequency;
      u_collect_frequency.real = this->collect_frequency;
      *(outbuffer + offset + 0) = (u_collect_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_collect_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_collect_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_collect_frequency.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_collect_frequency.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_collect_frequency.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_collect_frequency.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_collect_frequency.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->collect_frequency);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_logger_namespace;
      arrToVar(length_logger_namespace, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_logger_namespace; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_logger_namespace-1]=0;
      this->logger_namespace = (char *)(inbuffer + offset-1);
      offset += length_logger_namespace;
      uint32_t length_script_filepath;
      arrToVar(length_script_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_script_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_script_filepath-1]=0;
      this->script_filepath = (char *)(inbuffer + offset-1);
      offset += length_script_filepath;
      uint32_t log_element_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      log_element_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      log_element_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      log_element_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->log_element_names_length);
      if(log_element_names_lengthT > log_element_names_length)
        this->log_element_names = (char**)realloc(this->log_element_names, log_element_names_lengthT * sizeof(char*));
      log_element_names_length = log_element_names_lengthT;
      for( uint32_t i = 0; i < log_element_names_length; i++){
      uint32_t length_st_log_element_names;
      arrToVar(length_st_log_element_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_log_element_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_log_element_names-1]=0;
      this->st_log_element_names = (char *)(inbuffer + offset-1);
      offset += length_st_log_element_names;
        memcpy( &(this->log_element_names[i]), &(this->st_log_element_names), sizeof(char*));
      }
      union {
        double real;
        uint64_t base;
      } u_collect_frequency;
      u_collect_frequency.base = 0;
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_collect_frequency.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->collect_frequency = u_collect_frequency.real;
      offset += sizeof(this->collect_frequency);
     return offset;
    }

    const char * getType(){ return GETLOGGERCONFIGURATION; };
    const char * getMD5(){ return "8555c5e4369456fd46e254f90164712b"; };

  };

  class GetLoggerConfiguration {
    public:
    typedef GetLoggerConfigurationRequest Request;
    typedef GetLoggerConfigurationResponse Response;
  };

}
#endif
