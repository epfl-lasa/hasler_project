#include "footVarSynchronizer.h"



void footVarSynchronizer::readFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg)
{
	_msgFootOutput = *msg;
	_flagOutputMessageReceived=true;
	_platform_id = msg->platform_id; 
	for (int k=0; k<NB_AXIS; k++)
		{
			_platform_position[k]=msg->platform_position[rosAxis[k]];
			_platform_speed[k]=msg->platform_speed[rosAxis[k]];
			_platform_effortD[k]=msg->platform_effortD[rosAxis[k]];
			_platform_effortM[k]=msg->platform_effortM[rosAxis[k]];
		}	
	_platform_machineState= (State) msg->platform_machineState;
	_platform_controllerType= (Controller) msg->platform_controllerType;
	if(!_flagPlatformOutCommStarted)
	{_flagPlatformOutCommStarted=true;}
} 

void footVarSynchronizer::readForceModified(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  if (!_flagForceModifiedConnected) {
    ROS_INFO_ONCE("Unbiased force read!");
  	_flagForceModifiedConnected = true;
  }
  _ros_forceModified(0) = msg->wrench.force.x;
  _ros_forceModified(1) = msg->wrench.force.y;
  _ros_forceModified(2) = msg->wrench.force.z;
  _ros_forceModified(3) = msg->wrench.torque.x;
  _ros_forceModified(4) = msg->wrench.torque.y;
  _ros_forceModified(5) = msg->wrench.torque.z;
}

void footVarSynchronizer::readTorquesModified(const custom_msgs::FootOutputMsg::ConstPtr &msg) {
  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
	_ros_effortM(i)=msg->platform_effortM[rosAxis[i]];
  }
}

void footVarSynchronizer::readLegGravityCompWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  _legWrenchGravityComp(0) = msg->wrench.force.x;
  _legWrenchGravityComp(1) = msg->wrench.force.y;
  _legWrenchGravityComp(2) = msg->wrench.force.z;
  _legWrenchGravityComp(3) = msg->wrench.torque.x;
  _legWrenchGravityComp(4) = msg->wrench.torque.y;
  _legWrenchGravityComp(5) = msg->wrench.torque.z;
  if (!_flagLegCompWrenchRead) {
   // ROS_INFO("[%s footVarSync]: Receiving Leg Compensation Torques",Platform_Names[_platform_name]);
  }
  _flagLegCompWrenchRead = true;
}

void footVarSynchronizer::readLegGravCompFI(const custom_msgs::FootInputMsg::ConstPtr &msg) {
    
	for (size_t i = 0; i<NB_AXIS; i++)
	{
		_leg_grav_comp_effort(i) = msg->ros_effort[rosAxis[i]];
	}

  if (!_flagLegCompTorquesRead) {
    ROS_INFO("[%s footVarSync]: Receiving Leg Compensation Torques",Platform_Names[_platform_name]);
  }
  _flagLegCompTorquesRead = true;
}

void footVarSynchronizer::readInertiaCoriolisCompFI(const custom_msgs::FootInputMsg::ConstPtr &msg) {
    
	for (size_t i = 0; i<NB_AXIS; i++)
	{
		_inertial_coriolis_comp_effort(i) = msg->ros_effort[rosAxis[i]];
	}

  if (!_flagInertiaCoriolisRead) {
    ROS_INFO("[%s footVarSync]: Receiving Inertia and Coriolis of Platform Compensation Torques",Platform_Names[_platform_name]);
  }
  _flagInertiaCoriolisRead = true;
}

void footVarSynchronizer::readDesiredFootInputs(const custom_msgs::FootInputMsg::ConstPtr &msg,unsigned int n_)
{
	
	_msgDesiredFootInput[n_].ros_effort = msg->ros_effort;
	_msgDesiredFootInput[n_].ros_kp = msg->ros_kp;
	_msgDesiredFootInput[n_].ros_ki = msg->ros_ki;
	_msgDesiredFootInput[n_].ros_kd = msg->ros_kd;
	_msgDesiredFootInput[n_].ros_effortM = msg->ros_effortM;
	//_msgDesiredFootInput[n_].ros_forceSensor = msg->ros_forceSensor;
	_msgDesiredFootInput[n_].ros_position = msg->ros_position;
	_msgDesiredFootInput[n_].ros_speed = msg->ros_speed;
	_msgDesiredFootInput[n_].ros_filterAxisForce = msg->ros_filterAxisForce;
  	if (!_flagDesiredFootInputsRead[n_]) {
  	  ROS_INFO_ONCE("Receiving messages of foot input. Pub #%i", n_);
  	}
  _flagDesiredFootInputsRead[n_] = true;

}

void footVarSynchronizer::readTwoFeetOneToolMsg(const custom_msgs::TwoFeetOneToolMsg::ConstPtr &msg)
{
	me->_msgTwoFeetOneTool = *msg;
	_flagTwoFeetOneToolRead=true;
}

