#include "Gripper.h"

void Gripper::communicateToRos()
{
  // Publish foot output
  _gripperMutex.lock();
 //  pubGripperOutput();
   _nh.spinOnce(); //Publishes and Retrieves Messages
   // For Retrieving and Publishing to ROS. We can put it separate in the main in  case we want to put it in an interruption
   _gripperMutex.unlock();
}


//*****************ROS-MESSAGE-SUBSCRIBER- CALLBACK**********
//! 1
void Gripper::updateGripperInput(const custom_msgs_gripper::GripperInputMsg &msg)
{
  me->_gripperMutex.lock();
  for (uint c = 0 ; c < NB_GI_CATEGORY; c++) {
    me->_flagInputReceived[c] = true; //! To be used specially for the telemanipulation state
  }
    me->_ros_position=msg.ros_dPosition;
    me->_ros_speed=msg.ros_dSpeed;

  me->_gripperMutex.unlock();
}

//*****************ROS-SERVICES-CALLBACKS*************************/

//! 2
void Gripper::updateState(const custom_msgs_gripper::gripperSetStateSrv::Request &req, custom_msgs_gripper::gripperSetStateSrvResponse &resp )
{
  me->_gripperMutex.lock();
  State newState = (State) req.ros_gripperMachineState;
  //! Update the dimensions of the motor commands -> reflected force (normal) + compensation , etc 
  
  if (!(newState==me->_gripper_state)) // If I want to go to a new state
  { 
    resp.gripper_newState=true;
    me->_flagClearLastState=true;
    me->_ros_state = newState;
  } 
  else{ resp.gripper_newState=false; } //! You are already in the desired state
  me->_gripperMutex.unlock();
}

//! 3
void Gripper::updateController(const custom_msgs_gripper::gripperSetControllerSrv::Request &req,custom_msgs_gripper::gripperSetControllerSrv::Response &resp )
{
  me->_gripperMutex.lock();
  if ((me->_gripper_state==GRIPPER_STATE_CONTROL))
  {
    Controller newController = (Controller) req.ros_controllerType;
    if (me->_gripper_controllerType != newController)
    {
      me->_flagControllerTypeChanged = true;
    }
      me->_ros_controllerType = newController;
   me->_ros_flagDefaultControl = req.ros_defaultControl;
   if (!me->_ros_flagDefaultControl){
   
   
    float scale=SCALE_GAINS_ANGULAR_POSITION;

       me->_ros_kpPosition=req.ros_posP * scale;
       me->_ros_kiPosition=req.ros_posI * scale;
       me->_ros_kdPosition=req.ros_posD * scale; 

       scale=SCALE_GAINS_ANGULAR_SPEED;

       me->_ros_kpSpeed=req.ros_speedP * scale;
       me->_ros_kiSpeed=req.ros_speedI * scale;
       me->_ros_kdSpeed=req.ros_speedD * scale; 
    
    me->_flagCtrlGainsNew=true;
   }
    resp.gripper_controlOk=true;
  }
  else
  {
    resp.gripper_controlOk=false; 
  }
  me->_gripperMutex.unlock();
}


//! 4
void Gripper::pubGripperOutput()
{
  _msgGripperOutput.gripper_stamp = _nh.now();
  _msgGripperOutput.gripper_id = 0;
  
  _msgGripperOutput.gripper_position = _position * RAD_TO_DEG ;
  _msgGripperOutput.gripper_speed= _speed * RAD_TO_DEG ;

  _msgGripperOutput.gripper_controllerType = (uint8_t)_gripper_controllerType;
  _msgGripperOutput.gripper_machineState = (uint8_t)_gripper_state;
  _pubGripperOutput->publish(&_msgGripperOutput);
}

void Gripper::updateGripperFromRos() {

  if (_flagClearLastState) {
    clearLastState();
    _gripper_state = _ros_state;
    _flagClearLastState = false;
  }

  switch (_gripper_state)

  {
    case GRIPPER_STATE_CONTROL:
    {

        if (_flagControllerTypeChanged) {
          resetControllers(_gripper_controllerType);
          _flagControllerTypeChanged = false;
        }
        _gripper_controllerType = _ros_controllerType;

        if (_ros_flagDefaultControl && !_gripper_flagDefaultControl) {
          _flagDefaultCtrlNew = true;
        }
        _gripper_flagDefaultControl = _ros_flagDefaultControl;


      if (_flagCtrlGainsNew) {
          loadROSPIDGains();
          _flagCtrlGainsNew = false;
        }
    
    }
  }
   
}

bool Gripper::waitUntilRosConnect()
{
  if (!_nh.connected()) {
      //_gripperMutex.lock();
      _nh.spinOnce();
      //_gripperMutex.unlock();
      return false;
  }
  else
  {
    return true;
  }  
}