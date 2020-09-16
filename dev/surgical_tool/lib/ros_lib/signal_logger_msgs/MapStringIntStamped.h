#ifndef _ROS_signal_logger_msgs_MapStringIntStamped_h
#define _ROS_signal_logger_msgs_MapStringIntStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "signal_logger_msgs/PairStringInt.h"

namespace signal_logger_msgs
{

  class MapStringIntStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t pairs_length;
      typedef signal_logger_msgs::PairStringInt _pairs_type;
      _pairs_type st_pairs;
      _pairs_type * pairs;

    MapStringIntStamped():
      header(),
      pairs_length(0), pairs(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->pairs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pairs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pairs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pairs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pairs_length);
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->pairs[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t pairs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pairs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pairs_length);
      if(pairs_lengthT > pairs_length)
        this->pairs = (signal_logger_msgs::PairStringInt*)realloc(this->pairs, pairs_lengthT * sizeof(signal_logger_msgs::PairStringInt));
      pairs_length = pairs_lengthT;
      for( uint32_t i = 0; i < pairs_length; i++){
      offset += this->st_pairs.deserialize(inbuffer + offset);
        memcpy( &(this->pairs[i]), &(this->st_pairs), sizeof(signal_logger_msgs::PairStringInt));
      }
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/MapStringIntStamped"; };
    const char * getMD5(){ return "d2b19b9dafeb23d52baab89b8c371ab0"; };

  };

}
#endif