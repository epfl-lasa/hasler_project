#ifndef _ROS_custom_msgs_gripper_GripperOutputMsg_h
#define _ROS_custom_msgs_gripper_GripperOutputMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace custom_msgs_gripper
{

  class GripperOutputMsg : public ros::Msg
  {
    public:
      typedef ros::Time _gripper_stamp_type;
      _gripper_stamp_type gripper_stamp;
      typedef int8_t _gripper_id_type;
      _gripper_id_type gripper_id;
      typedef float _gripper_position_type;
      _gripper_position_type gripper_position;
      typedef float _gripper_speed_type;
      _gripper_speed_type gripper_speed;
      typedef int8_t _gripper_controllerType_type;
      _gripper_controllerType_type gripper_controllerType;
      typedef int8_t _gripper_machineState_type;
      _gripper_machineState_type gripper_machineState;
      typedef uint8_t _gripper_isOpen_type;
      _gripper_isOpen_type gripper_isOpen;

    GripperOutputMsg():
      gripper_stamp(),
      gripper_id(0),
      gripper_position(0),
      gripper_speed(0),
      gripper_controllerType(0),
      gripper_machineState(0),
      gripper_isOpen(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gripper_stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gripper_stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gripper_stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gripper_stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_stamp.sec);
      *(outbuffer + offset + 0) = (this->gripper_stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gripper_stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gripper_stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gripper_stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_id;
      u_gripper_id.real = this->gripper_id;
      *(outbuffer + offset + 0) = (u_gripper_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_id);
      union {
        float real;
        uint32_t base;
      } u_gripper_position;
      u_gripper_position.real = this->gripper_position;
      *(outbuffer + offset + 0) = (u_gripper_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_position);
      union {
        float real;
        uint32_t base;
      } u_gripper_speed;
      u_gripper_speed.real = this->gripper_speed;
      *(outbuffer + offset + 0) = (u_gripper_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gripper_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gripper_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gripper_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gripper_speed);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_controllerType;
      u_gripper_controllerType.real = this->gripper_controllerType;
      *(outbuffer + offset + 0) = (u_gripper_controllerType.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_controllerType);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_machineState;
      u_gripper_machineState.real = this->gripper_machineState;
      *(outbuffer + offset + 0) = (u_gripper_machineState.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_machineState);
      *(outbuffer + offset + 0) = (this->gripper_isOpen >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_isOpen);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->gripper_stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->gripper_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gripper_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gripper_stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gripper_stamp.sec);
      this->gripper_stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->gripper_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gripper_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gripper_stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gripper_stamp.nsec);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_id;
      u_gripper_id.base = 0;
      u_gripper_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_id = u_gripper_id.real;
      offset += sizeof(this->gripper_id);
      union {
        float real;
        uint32_t base;
      } u_gripper_position;
      u_gripper_position.base = 0;
      u_gripper_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper_position = u_gripper_position.real;
      offset += sizeof(this->gripper_position);
      union {
        float real;
        uint32_t base;
      } u_gripper_speed;
      u_gripper_speed.base = 0;
      u_gripper_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gripper_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gripper_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gripper_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gripper_speed = u_gripper_speed.real;
      offset += sizeof(this->gripper_speed);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_controllerType;
      u_gripper_controllerType.base = 0;
      u_gripper_controllerType.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_controllerType = u_gripper_controllerType.real;
      offset += sizeof(this->gripper_controllerType);
      union {
        int8_t real;
        uint8_t base;
      } u_gripper_machineState;
      u_gripper_machineState.base = 0;
      u_gripper_machineState.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_machineState = u_gripper_machineState.real;
      offset += sizeof(this->gripper_machineState);
      this->gripper_isOpen =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gripper_isOpen);
     return offset;
    }

    const char * getType(){ return "custom_msgs_gripper/GripperOutputMsg"; };
    const char * getMD5(){ return "dc35d110d083b9284f4ae337ea0ab82d"; };

  };

}
#endif