#include "footVarSynchronizer.h"


void footVarSynchronizer::checkWhichPIDGainsToUse()
{	if (_controlTools)
	{
		if (_mixedPlatformOn)
		{	
			if(_platform_machineState==TELEOPERATION)
			{
				if ((Platform_Name) _mainPlatform == _platform_id)
				{
					_myPIDCategory = _msgTwoFeetOneTool.currentControlMode == (uint8_t) TOOL_POSITION_CTRL ? TOOL_POS_PID : TOOL_SPEED_PID;
				}else
				{
					_myPIDCategory=MP_TOOL_MIXED_PID;
				}
			}
			else if(_platform_machineState==ROBOT_STATE_CONTROL)
			{
				_myPIDCategory=S_ROBOT_CTRL_PID;
			}
		}else
		{
			_myPIDCategory = (_myToolControl ==  TOOL_POSITION_CTRL) ? TOOL_POS_PID : TOOL_SPEED_PID;
		}
	}else 
		{	
			if(_platform_machineState==TELEOPERATION)
			{
				_myPIDCategory=S_TELEOP_PID;
			}
			else if(_platform_machineState==ROBOT_STATE_CONTROL)
			{
				_myPIDCategory=S_ROBOT_CTRL_PID;
			}
		}
}

void footVarSynchronizer::controlGainsDefault(int axis_)
{
	if (axis_ == -1)
	{
		for (int k = 0; k < NB_AXIS; k++)
		{
			controlGainsDefault(k);
		}
	}

	else
	{
		if (_platform_controllerType=POSITION_CTRL)
		{
			switch (axis_)
			{
				case (X):
				{	
					if (_platform_name == RIGHT_PLATFORM_ID)
					{
						_config.kp_X = Utils_math<double>::map(Utils_math<double>::smoothFall(_platform_position(X),-X_RANGE*0.05,X_RANGE*0.05),0.0,1.0,_ros_paramP_A[_myPIDCategory][X],_ros_paramP_B[_myPIDCategory][X]);
					}
					else
					{
						_config.kp_X = Utils_math<double>::map(Utils_math<double>::smoothRise(_platform_position(X),-X_RANGE*0.05,X_RANGE*0.05),0.0,1.0,_ros_paramP_A[_myPIDCategory][X],_ros_paramP_B[_myPIDCategory][X]);
					}
					_config.ki_X = _ros_paramI[_myPIDCategory][X];
					_config.kd_X = _ros_paramD[_myPIDCategory][X];
					_ros_posP[X] = Utils_math<double>::bound(_config.kp_X,0,5000.0);
					_ros_posI[X] = Utils_math<double>::bound(_config.ki_X,0,5000.0);
					_ros_posD[X] = Utils_math<double>::bound(_config.kd_X,0,30.0);
					break;
				}
				case (Y):
				{
					_config.kp_Y = Utils_math<double>::map(Utils_math<double>::smoothRise(_platform_position(Y),-Y_RANGE*0.05,Y_RANGE*0.05),0.0,1.0,_ros_paramP_A[_myPIDCategory][Y],_ros_paramP_B[_myPIDCategory][Y]);
					_config.ki_Y = _ros_paramI[_myPIDCategory][Y];
					_config.kd_Y = _ros_paramD[_myPIDCategory][Y];
					_ros_posP[Y] = Utils_math<double>::bound(_config.kp_Y,0,5000.0);
					_ros_posI[Y] = Utils_math<double>::bound(_config.ki_Y,0,5000.0);
					_ros_posD[Y] = Utils_math<double>::bound(_config.kd_Y,0,30.0);
					break;
				}
				case (PITCH):
				{
					_config.kp_PITCH = Utils_math<double>::map(Utils_math<double>::smoothFall(_platform_position(PITCH),-PITCH_RANGE*0.1,PITCH_RANGE*0.1),0.0,1.0,_ros_paramP_A[_myPIDCategory][PITCH],_ros_paramP_B[_myPIDCategory][PITCH]);
					_config.ki_PITCH = _ros_paramI[_myPIDCategory][PITCH];
					_config.kd_PITCH = _ros_paramD[_myPIDCategory][PITCH];
					_ros_posP[PITCH] = Utils_math<double>::bound(_config.kp_PITCH,0,10000.0);
					_ros_posI[PITCH] = Utils_math<double>::bound(_config.ki_PITCH,0,10000.0);
					_ros_posD[PITCH] = Utils_math<double>::bound(_config.kd_PITCH,0,200.0);
					break;
				}
				case (ROLL):
				{
					if (_platform_name == RIGHT_PLATFORM_ID)
					{
						_config.kp_ROLL = Utils_math<double>::map(Utils_math<double>::smoothFall(_platform_position(ROLL),-ROLL_RANGE*0.1,ROLL_RANGE*0.1),0.0,1.0,_ros_paramP_A[_myPIDCategory][ROLL],_ros_paramP_B[_myPIDCategory][ROLL]);
					}
					else
					{
						_config.kp_ROLL = Utils_math<double>::map(Utils_math<double>::smoothRise(_platform_position(ROLL),-ROLL_RANGE*0.1,ROLL_RANGE*0.1),0.0,1.0,_ros_paramP_A[_myPIDCategory][ROLL],_ros_paramP_B[_myPIDCategory][ROLL]);
					}
					_config.ki_ROLL = _ros_paramI[_myPIDCategory][ROLL];
					_config.kd_ROLL = _ros_paramD[_myPIDCategory][ROLL];
					_ros_posP[ROLL] = Utils_math<double>::bound(_config.kp_ROLL,0.0,10000.0);
					_ros_posI[ROLL] = Utils_math<double>::bound(_config.ki_ROLL,0.0,10000.0);
					_ros_posD[ROLL] = Utils_math<double>::bound(_config.kd_ROLL,0.0,200.0);
					break;
				}
				case (YAW):
				{
					if (_platform_name == RIGHT_PLATFORM_ID)
					{
						_config.kp_YAW = Utils_math<double>::map(Utils_math<double>::smoothFall(_platform_position(YAW),-YAW_RANGE*0.1,YAW_RANGE*0.1),0.0,1.0,_ros_paramP_A[_myPIDCategory][YAW],_ros_paramP_B[_myPIDCategory][YAW]);
					}
					else
					{
						_config.kp_YAW = Utils_math<double>::map(Utils_math<double>::smoothRise(_platform_position(YAW),-YAW_RANGE*0.1,YAW_RANGE*0.1),0.0,1.0,_ros_paramP_A[_myPIDCategory][YAW],_ros_paramP_B[_myPIDCategory][YAW]);
					}

					_config.ki_YAW = _ros_paramI[_myPIDCategory][YAW];
					_config.kd_YAW = _ros_paramD[_myPIDCategory][YAW];
					_ros_posP[YAW] = Utils_math<double>::bound(_config.kp_YAW,0.0,10000.0);
					_ros_posI[YAW] = Utils_math<double>::bound(_config.ki_YAW,0.0,10000.0);
					_ros_posD[YAW] = Utils_math<double>::bound(_config.kd_YAW,0.0,200.0);
					break;
				}
			}
		}
	}
		
}