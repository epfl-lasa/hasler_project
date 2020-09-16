#ifndef _ROS_SERVICE_SaveLoggerData_h
#define _ROS_SERVICE_SaveLoggerData_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace signal_logger_msgs
{

static const char SAVELOGGERDATA[] = "signal_logger_msgs/SaveLoggerData";

  class SaveLoggerDataRequest : public ros::Msg
  {
    public:
      uint32_t logfileTypes_length;
      typedef int8_t _logfileTypes_type;
      _logfileTypes_type st_logfileTypes;
      _logfileTypes_type * logfileTypes;
      enum { LOGFILE_TYPE_BINARY = 0 };
      enum { LOGFILE_TYPE_CSV = 1 };
      enum { LOGFILE_TYPE_BAG = 2 };

    SaveLoggerDataRequest():
      logfileTypes_length(0), logfileTypes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->logfileTypes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->logfileTypes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->logfileTypes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->logfileTypes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->logfileTypes_length);
      for( uint32_t i = 0; i < logfileTypes_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_logfileTypesi;
      u_logfileTypesi.real = this->logfileTypes[i];
      *(outbuffer + offset + 0) = (u_logfileTypesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->logfileTypes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t logfileTypes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      logfileTypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      logfileTypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      logfileTypes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->logfileTypes_length);
      if(logfileTypes_lengthT > logfileTypes_length)
        this->logfileTypes = (int8_t*)realloc(this->logfileTypes, logfileTypes_lengthT * sizeof(int8_t));
      logfileTypes_length = logfileTypes_lengthT;
      for( uint32_t i = 0; i < logfileTypes_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_logfileTypes;
      u_st_logfileTypes.base = 0;
      u_st_logfileTypes.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_logfileTypes = u_st_logfileTypes.real;
      offset += sizeof(this->st_logfileTypes);
        memcpy( &(this->logfileTypes[i]), &(this->st_logfileTypes), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return SAVELOGGERDATA; };
    const char * getMD5(){ return "de377bf5a2077047677da0383cd971b8"; };

  };

  class SaveLoggerDataResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SaveLoggerDataResponse():
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

    const char * getType(){ return SAVELOGGERDATA; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SaveLoggerData {
    public:
    typedef SaveLoggerDataRequest Request;
    typedef SaveLoggerDataResponse Response;
  };

}
#endif
