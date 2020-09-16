#ifndef DEFINITIONS_ROS_H
#define DEFINITIONS_ROS_H

#include "definitions_main.h"

enum GripperInput_Category { MSG_POSITION, MSG_SPEED, NB_GI_CATEGORY };

enum Param_Category {PID_POS_C, PID_VEL_C, ALL};

//******************************NAMES*****************************
    #define GRIPPER_SUBSCRIBER_NAME_LEFT "/Gripper_Input/Left"
    #define GRIPPER_PUBLISHER_NAME_LEFT "/Gripper_Output/Left"
    #define SERVICE_CHANGE_STATE_NAME_LEFT "/update_gripper_state/Left"
    #define SERVICE_CHANGE_CTRL_NAME_LEFT "/update_gripper_controller/Left"
        
    #define GRIPPER_SUBSCRIBER_NAME_RIGHT "/Gripper_Input/Right"
    #define GRIPPER_PUBLISHER_NAME_RIGHT "/Gripper_Output/Right"
    #define SERVICE_CHANGE_STATE_NAME_RIGHT "/update_gripper_state/Right"
    #define SERVICE_CHANGE_CTRL_NAME_RIGHT "/update_gripper_controller/Left"

    

#endif // DEFINITIONS_ROS_H