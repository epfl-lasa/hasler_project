#ifndef _ROS_custom_msgs_gripper_SharedGrasping_h
#define _ROS_custom_msgs_gripper_SharedGrasping_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs_gripper
{

  class SharedGrasping : public ros::Msg
  {
    public:
      typedef ros::Time _sGrasp_stamp_type;
      _sGrasp_stamp_type sGrasp_stamp;
      typedef int8_t _sGrasp_id_type;
      _sGrasp_id_type sGrasp_id;
      float sGrasp_hapticTorques[5];
      float sGrasp_hFilters[5];
      typedef float _sGrasp_threshold_type;
      _sGrasp_threshold_type sGrasp_threshold;
      typedef int8_t _sGrasp_aState_type;
      _sGrasp_aState_type sGrasp_aState;

    SharedGrasping():
      sGrasp_stamp(),
      sGrasp_id(0),
      sGrasp_hapticTorques(),
      sGrasp_hFilters(),
      sGrasp_threshold(0),
      sGrasp_aState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sGrasp_stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sGrasp_stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sGrasp_stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sGrasp_stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sGrasp_stamp.sec);
      *(outbuffer + offset + 0) = (this->sGrasp_stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sGrasp_stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sGrasp_stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sGrasp_stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sGrasp_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_sGrasp_id;
      u_sGrasp_id.real = this->sGrasp_id;
      *(outbuffer + offset + 0) = (u_sGrasp_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sGrasp_id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_sGrasp_hapticTorquesi;
      u_sGrasp_hapticTorquesi.real = this->sGrasp_hapticTorques[i];
      *(outbuffer + offset + 0) = (u_sGrasp_hapticTorquesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sGrasp_hapticTorquesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sGrasp_hapticTorquesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sGrasp_hapticTorquesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sGrasp_hapticTorques[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_sGrasp_hFiltersi;
      u_sGrasp_hFiltersi.real = this->sGrasp_hFilters[i];
      *(outbuffer + offset + 0) = (u_sGrasp_hFiltersi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sGrasp_hFiltersi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sGrasp_hFiltersi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sGrasp_hFiltersi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sGrasp_hFilters[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_sGrasp_threshold;
      u_sGrasp_threshold.real = this->sGrasp_threshold;
      *(outbuffer + offset + 0) = (u_sGrasp_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sGrasp_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sGrasp_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sGrasp_threshold.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sGrasp_threshold);
      union {
        int8_t real;
        uint8_t base;
      } u_sGrasp_aState;
      u_sGrasp_aState.real = this->sGrasp_aState;
      *(outbuffer + offset + 0) = (u_sGrasp_aState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sGrasp_aState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sGrasp_stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->sGrasp_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sGrasp_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sGrasp_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sGrasp_stamp.sec);
      this->sGrasp_stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->sGrasp_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sGrasp_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sGrasp_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sGrasp_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_sGrasp_id;
      u_sGrasp_id.base = 0;
      u_sGrasp_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sGrasp_id = u_sGrasp_id.real;
      offset += sizeof(this->sGrasp_id);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_sGrasp_hapticTorquesi;
      u_sGrasp_hapticTorquesi.base = 0;
      u_sGrasp_hapticTorquesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sGrasp_hapticTorquesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sGrasp_hapticTorquesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sGrasp_hapticTorquesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sGrasp_hapticTorques[i] = u_sGrasp_hapticTorquesi.real;
      offset += sizeof(this->sGrasp_hapticTorques[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_sGrasp_hFiltersi;
      u_sGrasp_hFiltersi.base = 0;
      u_sGrasp_hFiltersi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sGrasp_hFiltersi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sGrasp_hFiltersi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sGrasp_hFiltersi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sGrasp_hFilters[i] = u_sGrasp_hFiltersi.real;
      offset += sizeof(this->sGrasp_hFilters[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_sGrasp_threshold;
      u_sGrasp_threshold.base = 0;
      u_sGrasp_threshold.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sGrasp_threshold.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sGrasp_threshold.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sGrasp_threshold.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sGrasp_threshold = u_sGrasp_threshold.real;
      offset += sizeof(this->sGrasp_threshold);
      union {
        int8_t real;
        uint8_t base;
      } u_sGrasp_aState;
      u_sGrasp_aState.base = 0;
      u_sGrasp_aState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sGrasp_aState = u_sGrasp_aState.real;
      offset += sizeof(this->sGrasp_aState);
     return offset;
    }

    const char * getType(){ return "custom_msgs_gripper/SharedGrasping"; };
    const char * getMD5(){ return "4afbdc5e920e43fbe4c39aa7aa6b83cd"; };

  };

}
#endif