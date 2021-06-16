#include "footVarSynchronizer.h"



void footVarSynchronizer::updateInternalVariables()
{
	_ros_newState = (uint8_t)_config.machine_state;

	_ros_controlledAxis = (int8_t) _config.controlled_axis;
	_ros_controllerType = (uint8_t) _config.controller_type;
	_ros_defaultControl = (bool) _config.use_default_gains;
	_flagSendPIDGains = (bool) _config.send_pid_gains;
	if ((bool) _config.load_param_pid_gains && !_flagLoadPIDGains)
	{
		getPIDParams();
	}
	_flagLoadPIDGains = (bool) _config.load_param_pid_gains;
    _flagControlThisPosition = (bool) _config.send_this_position;
    _flagControlZeroEffort = (bool)_config.send_zero_effort;
    _flagCapturePlatformPosition = (bool)_config.capture_platform_position;

    if (_flagControlThisPosition) {
      _ros_position << _config.desired_Position_Y,
          _config.desired_Position_X, _config.desired_Position_PITCH,
          _config.desired_Position_ROLL, _config.desired_Position_YAW;
      ROS_DEBUG("Updating_POSITION");
	}


    _ros_effortComp[NORMAL]=(uint8_t) _config.effortComp_normal;
	_ros_effortComp[SOFT_LIMITS]=(uint8_t) _config.effortComp_softLimits;
	_ros_effortComp[CUSTOM_IMPEDANCE]=(uint8_t) _config.effortComp_customImpedance;
	_ros_effortComp[COMPENSATION]=(uint8_t) _config.effortComp_compensation;
	_ros_effortComp[FEEDFORWARD]=(uint8_t) _config.effortComp_feedforward;

		if (_config.compensate_leg)
		{
			if (!_flagCompensateLeg)
			{
				ROS_INFO("[%s footVarSync]: Compensating the Leg",Platform_Names[_platform_name]);
				_flagCompensateLeg=true;
			}
		}
		else
		{
			if (_flagCompensateLeg)
			{
				ROS_INFO("[%s footVarSync]: Not Compensating the Leg Anymore",Platform_Names[_platform_name]);
				_flagCompensateLeg=false;
			}
		}
		if (_flagSendPIDGains && (_platform_machineState==TELEOPERATION || _platform_machineState==ROBOT_STATE_CONTROL) ){
			if (_platform_controllerType==POSITION_CTRL)
			{
				_ros_posP[X]=_config.kp_X; 
				_ros_posP[Y]=_config.kp_Y;
				_ros_posP[PITCH]=_config.kp_PITCH;
				_ros_posP[ROLL]=_config.kp_ROLL;
				_ros_posP[YAW]=_config.kp_YAW;

				_ros_posI[X]=_config.ki_X; 
				_ros_posI[Y]=_config.ki_Y;
				_ros_posI[PITCH]=_config.ki_PITCH;
				_ros_posI[ROLL]=_config.ki_ROLL;
				_ros_posI[YAW]=_config.ki_YAW;

				_ros_posD[X]=_config.kd_X; 
				_ros_posD[Y]=_config.kd_Y;
				_ros_posD[PITCH]=_config.kd_PITCH;
				_ros_posD[ROLL]=_config.kd_ROLL;
				_ros_posD[YAW]=_config.kd_YAW;
			} else if (_platform_controllerType==SPEED_CTRL)
			{
				_ros_speedP[X]=_config.kp_X; 
				_ros_speedP[Y]=_config.kp_Y;
				_ros_speedP[PITCH]=_config.kp_PITCH;
				_ros_speedP[ROLL]=_config.kp_ROLL;
				_ros_speedP[YAW]=_config.kp_YAW;

				_ros_speedI[X]=_config.ki_X; 
				_ros_speedI[Y]=_config.ki_Y;
				_ros_speedI[PITCH]=_config.ki_PITCH;
				_ros_speedI[ROLL]=_config.ki_ROLL;
				_ros_speedI[YAW]=_config.ki_YAW;

				_ros_speedD[X]=_config.kd_X; 
				_ros_speedD[Y]=_config.kd_Y;
				_ros_speedD[PITCH]=_config.kd_PITCH;
				_ros_speedD[ROLL]=_config.kd_ROLL;
				_ros_speedD[YAW]=_config.kd_YAW;
			}
		}else{
			// TO DO ADD THE GAINS DEPENDING ON THE TOOL
		}
		

}

void footVarSynchronizer::dynamicReconfigureCallback(foot_variables_sync::machineStateParamsConfig &config, uint32_t level)
{
  ROS_INFO("[%s footVarSync]: Reconfigure request",Platform_Names[_platform_name]);
	//cout<<_platform_machineState<<endl;
  	_flagWasDynReconfCalled=true;
	_config = config;
}
