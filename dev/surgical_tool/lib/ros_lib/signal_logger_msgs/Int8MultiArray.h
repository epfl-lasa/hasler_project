#ifndef _ROS_signal_logger_msgs_Int8MultiArray_h
#define _ROS_signal_logger_msgs_Int8MultiArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

namespace signal_logger_msgs
{

  class Int8MultiArray : public ros::Msg
  {
    public:
      typedef std_msgs::MultiArrayLayout _layout_type;
      _layout_type layout;
      uint32_t dim_length;
      typedef std_msgs::MultiArrayDimension _dim_type;
      _dim_type st_dim;
      _dim_type * dim;
      typedef const char* _label_type;
      _label_type label;
      typedef uint32_t _size_type;
      _size_type size;
      typedef uint32_t _stride_type;
      _stride_type stride;
      typedef uint32_t _data_offset_type;
      _data_offset_type data_offset;
      uint32_t data_length;
      typedef int8_t _data_type;
      _data_type st_data;
      _data_type * data;

    Int8MultiArray():
      layout(),
      dim_length(0), dim(NULL),
      label(""),
      size(0),
      stride(0),
      data_offset(0),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->layout.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->dim_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dim_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dim_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dim_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dim_length);
      for( uint32_t i = 0; i < dim_length; i++){
      offset += this->dim[i].serialize(outbuffer + offset);
      }
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      *(outbuffer + offset + 0) = (this->size >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->size >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->size >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->size >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size);
      *(outbuffer + offset + 0) = (this->stride >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stride >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stride >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stride >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stride);
      *(outbuffer + offset + 0) = (this->data_offset >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_offset >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_offset >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_offset >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_offset);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->layout.deserialize(inbuffer + offset);
      uint32_t dim_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dim_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dim_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dim_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dim_length);
      if(dim_lengthT > dim_length)
        this->dim = (std_msgs::MultiArrayDimension*)realloc(this->dim, dim_lengthT * sizeof(std_msgs::MultiArrayDimension));
      dim_length = dim_lengthT;
      for( uint32_t i = 0; i < dim_length; i++){
      offset += this->st_dim.deserialize(inbuffer + offset);
        memcpy( &(this->dim[i]), &(this->st_dim), sizeof(std_msgs::MultiArrayDimension));
      }
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      this->size =  ((uint32_t) (*(inbuffer + offset)));
      this->size |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->size |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->size |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->size);
      this->stride =  ((uint32_t) (*(inbuffer + offset)));
      this->stride |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stride |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stride |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stride);
      this->data_offset =  ((uint32_t) (*(inbuffer + offset)));
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->data_offset |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->data_offset);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (int8_t*)realloc(this->data, data_lengthT * sizeof(int8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return "signal_logger_msgs/Int8MultiArray"; };
    const char * getMD5(){ return "c829977282f98c9bca7ef92b525e0ffb"; };

  };

}
#endif