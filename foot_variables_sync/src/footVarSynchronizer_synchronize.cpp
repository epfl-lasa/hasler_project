#include "footVarSynchronizer.h"

void footVarSynchronizer::changeParamCheck()
{
	//! Start with assumptions
	for (int i=0; i<NB_PARAMS_CATEGORIES; i++ )
	{ _flagIsParamStillSame[i]=true;}

	//! Verify
		
	// Check State

	if (_config.machine_state==_configPrev.machine_state)
	{
		_flagIsParamStillSame[Params_Category::M_STATE] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::M_STATE] = false;
	}

	// Check Effort Components

	if ( (_config.effortComp_softLimits == _configPrev.effortComp_softLimits) &&
		 (_config.effortComp_compensation == _configPrev.effortComp_compensation) &&
		 (_config.effortComp_customImpedance == _configPrev.effortComp_customImpedance) &&
		 (_config.effortComp_feedforward == _configPrev.effortComp_feedforward) &&
		 (_config.effortComp_normal == _configPrev.effortComp_normal)  )
	{
		_flagIsParamStillSame[Params_Category::EFF_COMP] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::EFF_COMP] = false;
	}

	// Check Controlled Axis

	if (_config.controlled_axis==_configPrev.controlled_axis)
	{
		_flagIsParamStillSame[Params_Category::C_AXIS] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::C_AXIS] = false;
	}

	// Check Controller Type

	if (_config.controller_type == _configPrev.controller_type)
	{
		_flagIsParamStillSame[Params_Category::C_TYPE] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::C_TYPE] = false;
	}

	
	// Check Flag Send Position
	if (_config.send_this_position == _configPrev.send_this_position)
	{
		_flagIsParamStillSame[Params_Category::FLAG_SENDPOS] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::FLAG_SENDPOS] = false;
	}

	// Check Flag Capture Platform Position
	if (_config.capture_platform_position == _configPrev.capture_platform_position)
	{
		_flagIsParamStillSame[Params_Category::FLAG_CAPTUREPOS] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::FLAG_CAPTUREPOS] = false;
	}

	// Check Desired Position
	if (fabs(_config.desired_Position_X - _configPrev.desired_Position_X) +
		fabs(_config.desired_Position_Y - _configPrev.desired_Position_Y) +
		fabs(_config.desired_Position_PITCH - _configPrev.desired_Position_PITCH) +
		fabs(_config.desired_Position_ROLL - _configPrev.desired_Position_ROLL) +
		fabs(_config.desired_Position_YAW - _configPrev.desired_Position_YAW) <= FLT_EPSILON)
		{ _flagIsParamStillSame[Params_Category::DES_POS] = true; }
	
	else { _flagIsParamStillSame[Params_Category::DES_POS] = false;	}

	// Check Flag Use Default Gains
	if (_config.use_default_gains == _configPrev.use_default_gains)
	{
		_flagIsParamStillSame[Params_Category::FLAG_GAINS] = true;
	}
	else
	{
		_flagIsParamStillSame[Params_Category::FLAG_GAINS] = false;
	}
	// Check PID Gains Position

	if (fabs(_config.kp_X - _configPrev.kp_X) +
		fabs(_config.kp_Y - _configPrev.kp_Y) +
		fabs(_config.kp_PITCH - _configPrev.kp_PITCH) +
		fabs(_config.kp_ROLL - _configPrev.kp_ROLL) +
		fabs(_config.kp_YAW - _configPrev.kp_YAW) +
		fabs(_config.kp_Y - _configPrev.kp_Y) +
		fabs(_config.ki_X - _configPrev.ki_X) +
		fabs(_config.ki_Y - _configPrev.ki_Y) +
		fabs(_config.ki_PITCH - _configPrev.ki_PITCH) +
		fabs(_config.ki_ROLL - _configPrev.ki_ROLL) +
		fabs(_config.ki_YAW - _configPrev.ki_YAW) +
		fabs(_config.kd_X - _configPrev.kd_X) +
		fabs(_config.kd_Y - _configPrev.kd_Y) +
		fabs(_config.kd_PITCH - _configPrev.kd_PITCH) +
		fabs(_config.kd_ROLL - _configPrev.kd_ROLL) +
		fabs(_config.kd_YAW - _configPrev.kd_YAW) <= FLT_EPSILON)

	{	
		if(_platform_controllerType==POSITION_CTRL)
		{
			_flagIsParamStillSame[Params_Category::PID_POS] = true;
		}else if(_platform_controllerType==SPEED_CTRL)
		{
			_flagIsParamStillSame[Params_Category::PID_SPEED] = true;
		}
	}

	else
	{
		// std::cout<<"boo"<<std::endl;
		if(_platform_controllerType==POSITION_CTRL)
		{
			_flagIsParamStillSame[Params_Category::PID_POS] = false;
		}else if(_platform_controllerType==SPEED_CTRL)
		{
			_flagIsParamStillSame[Params_Category::PID_SPEED] = false;
		}
	}
}

void footVarSynchronizer::changedPlatformCheck()
{
	//! Start with assumptions
	for (int i = 0; i < NB_FO_CATEGORIES; i++)
	{
		_flagIsPlatformStillSame[i] = true;
	}

	// //! Verify

	// // Check Position

	// if (fabs(_msgFootOutput.platform_position[0] - _msgFootOutputPrev.platform_position[0]) +
	// 	fabs(_msgFootOutput.platform_position[1] - _msgFootOutputPrev.platform_position[1]) +
	// 	fabs(_msgFootOutput.platform_position[2] - _msgFootOutputPrev.platform_position[2]) + 
	// 	fabs(_msgFootOutput.platform_position[3] - _msgFootOutputPrev.platform_position[3]) +
	// 	fabs(_msgFootOutput.platform_position[4] - _msgFootOutputPrev.platform_position[4]) <= FLT_EPSILON)
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = true;
	// }
	// else
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = false;
	// }

	// // Check Speed

	// if (fabs(_msgFootOutput.platform_speed[0] - _msgFootOutputPrev.platform_speed[0]) +
	// 	fabs(_msgFootOutput.platform_speed[1] - _msgFootOutputPrev.platform_speed[1]) +
	// 	fabs(_msgFootOutput.platform_speed[2] - _msgFootOutputPrev.platform_speed[2]) + 
	// 	fabs(_msgFootOutput.platform_speed[3] - _msgFootOutputPrev.platform_speed[3]) +
	// 	fabs(_msgFootOutput.platform_speed[4] - _msgFootOutputPrev.platform_speed[4]) <= FLT_EPSILON)
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = true;
	// }
	// else
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = false;
	// }

	// // Check EffortD

	// if (fabs(_msgFootOutput.platform_effortD[0] - _msgFootOutputPrev.platform_effortD[0]) +
	// 	fabs(_msgFootOutput.platform_effortD[1] - _msgFootOutputPrev.platform_effortD[1]) +
	// 	fabs(_msgFootOutput.platform_effortD[2] - _msgFootOutputPrev.platform_effortD[2]) + 
	// 	fabs(_msgFootOutput.platform_effortD[3] - _msgFootOutputPrev.platform_effortD[3]) +
	// 	fabs(_msgFootOutput.platform_effortD[4] - _msgFootOutputPrev.platform_effortD[4]) <= FLT_EPSILON)
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = true;
	// }
	// else
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = false;
	// }

	// // Check EffortM

	// if (fabs(_msgFootOutput.platform_effortM[0] - _msgFootOutputPrev.platform_effortM[0]) +
	// 	fabs(_msgFootOutput.platform_effortM[1] - _msgFootOutputPrev.platform_effortM[1]) +
	// 	fabs(_msgFootOutput.platform_effortM[2] - _msgFootOutputPrev.platform_effortM[2]) + 
	// 	fabs(_msgFootOutput.platform_effortM[3] - _msgFootOutputPrev.platform_effortM[3]) +
	// 	fabs(_msgFootOutput.platform_effortM[4] - _msgFootOutputPrev.platform_effortM[4]) <= FLT_EPSILON)
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTM] = true;
	// }
	// else
	// {
	// 	_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTM] = false;
	// }

	// Check State

	if (_msgFootOutput.platform_machineState == _msgFootOutputPrev.platform_machineState || 
		_config.machine_state == _msgFootOutput.platform_machineState )
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_M_STATE] = true;
	}
	else
	{
		ROS_INFO("[%s footVarSync]: The Platform changed its state!",Platform_Names[_platform_name]);
		_flagIsPlatformStillSame[FootOutput_Category::FO_M_STATE] = false;
	}

	// Check Controller Type

	if (_msgFootOutput.platform_controllerType == _msgFootOutputPrev.platform_controllerType)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE] = false;
		ROS_INFO("[%s footVarSync]: The Platform changed its controller type!",Platform_Names[_platform_name]);
	}

}

void footVarSynchronizer::requestDoActionsParams()
	{

		if (_config.machine_state == (int) TELEOPERATION &&
			_flagCapturePlatformPosition)
		{
			_config.desired_Position_X = C_WS_LIMIT_X;
			_config.desired_Position_Y = C_WS_LIMIT_Y;
			_config.desired_Position_PITCH = C_WS_LIMIT_PITCH;
			_config.desired_Position_ROLL = C_WS_LIMIT_ROLL;
			_config.desired_Position_YAW = C_WS_LIMIT_YAW;
			_ros_position << C_WS_LIMIT_Y, C_WS_LIMIT_X, C_WS_LIMIT_PITCH, C_WS_LIMIT_ROLL, C_WS_LIMIT_YAW;
			_flagParamsActionsTaken = true;
		}

		//! Send Variables of the Set State Service
		if ( (!_flagIsParamStillSame[Params_Category::M_STATE]) ||
			 (!_flagIsParamStillSame[Params_Category::EFF_COMP]) )
		{
			_flagSetStateRequested=false;
			_flagResponseSetState=false;
			requestSetState();
			_flagParamsActionsTaken=true;
			if (_flagResponseSetState){ //! Verify the service went through
				ROS_INFO("[%s footVarSync]: A new state has been sent to the platform ",Platform_Names[_platform_name]);
  				ROS_INFO("[%s footVarSync]: Resetting GAINS ",Platform_Names[_platform_name]);
				return;
			}
			else{
				ROS_INFO("[%s footVarSync]: The state machine of the platform is already up to date ",Platform_Names[_platform_name]);
			}

		}
		//! Send variables of the controller
		if ((!_flagIsParamStillSame[Params_Category::C_AXIS]) ||
			(!_flagIsParamStillSame[Params_Category::C_TYPE]) ||
			(!_flagIsParamStillSame[Params_Category::FLAG_GAINS]) ||
			(!_flagIsParamStillSame[Params_Category::PID_POS]) ||
			(!_flagIsParamStillSame[Params_Category::PID_SPEED]))

			{
				_flagSetControllerRequested = false;
				_flagResponseSetController = false;
				requestSetController();
				_flagParamsActionsTaken= true;
				if (_flagResponseSetController) //! Verify the service went through
				{
					if (!_flagIsParamStillSame[Params_Category::C_AXIS]){
						if (_config.controlled_axis==-1)
						{ ROS_INFO("[%s footVarSync]: The controlled axis has changed to: ALL" ,Platform_Names[_platform_name]);}
						else {
						  ROS_INFO("[%s footVarSync]: The controlled axis has changed to: %s",Platform_Names[_platform_name], Axis_names[_config.controlled_axis]);
						}
					}
				}
				else
				{
					ROS_WARN("You updated the control but you are not in Teleoperation or Robot State Control!");
				}
			}

        
	if(_flagControlThisPosition && !_flagCapturePlatformPosition)
	{
		_flagPositionOnlyPublished = false;
		requestSetController();
        ROS_INFO("[%s footVarSync]: A new desired position has been published (no acknowledgement)!",Platform_Names[_platform_name]);
		_flagParamsActionsTaken= true;
	}
	else if (_flagControlThisPosition && _flagCapturePlatformPosition)
	{
		ROS_WARN("You cannot sent a position while capturing the platform position");
	}

	if (_flagControlZeroEffort) {
		_flagEffortOnlyPublished = false;
		requestSetController();
		publishFootInput(&_flagEffortOnlyPublished);
		ROS_INFO("[%s footVarSync]: Zero torque send (no, acknowledgement)!",Platform_Names[_platform_name]);
		_flagParamsActionsTaken = true;
	} 
}

void footVarSynchronizer::requestDoActionsPlatform()
{
	if ((!_flagIsPlatformStillSame[FootOutput_Category::FO_M_STATE]) 
	//	 && (_flagIsParamStillSame[Params_Category::M_STATE])
	)
	{
		_config.machine_state=_platform_machineState;
		_ros_newState=_config.machine_state;
		_flagPlatformActionsTaken= true;
	}

	if ((!_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE]) 
	//	 && (_flagIsParamStillSame[Params_Category::C_TYPE]) 
		)
	{
		_config.controller_type = _platform_controllerType;
		_ros_controllerType=_config.controller_type;
		_flagPlatformActionsTaken= true;
	}

	 if (
		(!_flagIsPlatformStillSame[FootOutput_Category::FO_POS]
		 && _flagCapturePlatformPosition) 
	//	 && (_flagIsParamStillSame[Params_Category::DES_POS])
		)
	{
		if (_ros_newState!=TELEOPERATION){
			_config.desired_Position_X = _platform_position[X];
			_config.desired_Position_Y = _platform_position[Y];
			_config.desired_Position_PITCH = _platform_position[PITCH];
			_config.desired_Position_ROLL = _platform_position[ROLL];
			_config.desired_Position_YAW = _platform_position[YAW];
		}
			_flagPlatformActionsTaken= true;
	}

}

