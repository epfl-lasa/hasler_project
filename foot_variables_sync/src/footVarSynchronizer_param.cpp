#include "footVarSynchronizer.h"

void footVarSynchronizer::getPIDParams(){
	if (_controlTools)
	{
		std::string controlMode; //! Position | Speed (Joystick)
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"_tool/toolControl", controlMode))
		{ 
		ROS_ERROR(" [%s footVarSync]: No /right_tool/toolControl  param",Platform_Names[_platform_name]); 
		}  
		_myToolControl =  controlMode.compare("position")==0 ? TOOL_POSITION_CTRL : TOOL_SPEED_CTRL;
    
		

		std::vector<double> pAToolPosControl_;
		std::vector<double> pBToolPosControl_;
		std::vector<double> dToolPosControl_;

		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolPos/p_a", pAToolPosControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolPos/p_a",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP_A[TOOL_POS_PID][i] = pAToolPosControl_[i];
			}
		}
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolPos/p_b", pBToolPosControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolPos/p_b",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP_B[TOOL_POS_PID][i] = pBToolPosControl_[i];
			}
		}

		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolPos/d", dToolPosControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolPos/d",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramD[TOOL_POS_PID][i] = dToolPosControl_[i];
			}
		}

		std::vector<double> pAToolSpeedControl_;
		std::vector<double> pBToolSpeedControl_;
		std::vector<double> dToolSpeedControl_;

		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolSpeed/p_a", pAToolSpeedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolSpeed/p_a",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP_A[TOOL_SPEED_PID][i] = pAToolSpeedControl_[i];
			}
		}

		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolSpeed/p_b", pBToolSpeedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolSpeed/p_b",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP_B[TOOL_SPEED_PID][i] = pBToolSpeedControl_[i];
			}
		}
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolSpeed/d", dToolSpeedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolSpeed/d",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramD[TOOL_SPEED_PID][i] = dToolSpeedControl_[i];
			}
		}

		if (_mixedPlatformOn)
		{
			std::string mainPlatform_="right";
			if (!_n.getParam("/mixed_platform/mainPlatform", mainPlatform_))
			{ 
			ROS_ERROR(" [%s footVarSync]: No /mixed_platform/mainPlatform  param",Platform_Names[_platform_name]); 
			} 

			_mainPlatform = mainPlatform_.compare("right")==0 ? RIGHT_PLATFORM_ID : LEFT_PLATFORM_ID;

			std::vector<double> pAToolMixedControl_;
			std::vector<double> pBToolMixedControl_;
			std::vector<double> dToolMixedControl_;

			if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolMixed/p_a", pAToolMixedControl_))
			{ 
				ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolMixed/p_a",Platform_Names[_platform_name]); 
			}
			else
			{
				for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
				{
					_ros_paramP_A[MP_TOOL_MIXED_PID][i] = pAToolMixedControl_[i];
				}
			}

			if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolMixed/p_b", pBToolMixedControl_))
			{ 
				ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolMixed/p_b",Platform_Names[_platform_name]); 
			}
			else
			{
				for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
				{
					_ros_paramP_B[MP_TOOL_MIXED_PID][i] = pBToolMixedControl_[i];
				}
			}

			if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/toolMixed/d", dToolMixedControl_))
			{ 
				ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/toolMixed/d",Platform_Names[_platform_name]); 
			}
			else
			{
				for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
				{
					_ros_paramD[MP_TOOL_MIXED_PID][i] = dToolMixedControl_[i];
				}
			}

				std::string toolControlTopic_ = "/mixedPlatform/platformState";
				if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_gains_toolControl/topic", toolControlTopic_))
				{ 
				
					ROS_ERROR("[%s footVarSync]: Missing /fi_gains_toolControl/topic",Platform_Names[_platform_name]); 
				}
				else
				{
					_subPlatformControlFromTool = _n.subscribe<custom_msgs::TwoFeetOneToolMsg>(toolControlTopic_, 1, boost::bind(&footVarSynchronizer::readTwoFeetOneToolMsg, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
				}

		}
	}

	std::vector<double> pTeloperation_;
	std::vector<double> iTeloperation_;
	std::vector<double> dTeloperation_;
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/teleoperation/p", pTeloperation_))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing state/teleoperation/p",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
		
			_ros_paramP_A[S_TELEOP_PID][i] = pTeloperation_[i];
			_ros_paramP_B[S_TELEOP_PID][i] = pTeloperation_[i];
		}
	}
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/teleoperation/i", iTeloperation_))
	{ 
	
		ROS_ERROR("[%s footVarSync]: Missing state/teleoperation/i",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
			_ros_paramI[S_TELEOP_PID][i] = iTeloperation_[i];
		}
	}
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/teleoperation/d", dTeloperation_))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing state/teleoperation/d",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
			_ros_paramD[S_TELEOP_PID][i] = dTeloperation_[i];
		}
	}

	std::vector<double> pRobotStControl_;
	std::vector<double> iRobotStControl_;
	std::vector<double> dRobotStControl_;
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/robot_control/p", pRobotStControl_))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing state/robot_control/p",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
			_ros_paramP_A[S_ROBOT_CTRL_PID][i] = pRobotStControl_[i];
			_ros_paramP_B[S_ROBOT_CTRL_PID][i] = pRobotStControl_[i];
		}
	}
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/robot_control/i", iRobotStControl_))
	{ 
	
		ROS_ERROR("[%s footVarSync]: Missing state/robot_control/i",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
			_ros_paramI[S_ROBOT_CTRL_PID][i] = iRobotStControl_[i];
		}
	}
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/state/robot_control/d", dRobotStControl_))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing state/robot_control/d",Platform_Names[_platform_name]); 
	}
	else
	{
		for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
		{
			_ros_paramD[S_ROBOT_CTRL_PID][i] = dRobotStControl_[i];
		}
	}

}