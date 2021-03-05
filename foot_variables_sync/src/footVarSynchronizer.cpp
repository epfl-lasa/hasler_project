#include "footVarSynchronizer.h"


#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

char const *Platform_Names[]{"none", "right", "left"};

footVarSynchronizer *footVarSynchronizer::me = NULL;

footVarSynchronizer::footVarSynchronizer(ros::NodeHandle &n_1, float frequency,footVarSynchronizer::Platform_Name platform_id): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_dt(1.0f/frequency)

{
	_flagPIDGainsByInput=false;
	me=this;
	_stop = false;
	_flagControlThisPosition=false;
	
	_flagSendPIDGains=false;
	_flagLoadPIDGains=false;
	_flagControlZeroEffort=false;

	_flagCapturePlatformPosition = false;
	_flagWasDynReconfCalled= false;
	_flagOutputMessageReceived=false;
	_flagInitialConfig = false;
	_flagParamsActionsTaken= false;
	_flagPlatformActionsTaken= false;

	_flagPlatformInCommStarted=false;
	_flagPositionOnlyPublished=false;
	_flagEffortOnlyPublished=false;

	_flagForceModifiedConnected=false;
	_flagLegCompTorquesRead=false;
	_flagLegCompWrenchRead = false;

	_flagTwoFeetOneToolRead = false;

	for (int i=0; i<NB_PARAMS_CATEGORIES; i++){
	_flagIsParamStillSame[i]=true;
	}

	for (int v = 0; v < NB_FO_CATEGORIES; v++)
	{
		_flagIsPlatformStillSame[v] = true;
	}

	_flagUpdateConfig=false;

    _flagSetControllerRequested=false;
    _flagSetStateRequested=false;

	_flagResponseSetController=false;
    _flagResponseSetState=false;

	_flagHumanOnPlatform=false;
	_flagCompensateLeg=false;

    _myPIDCategory = S_TELEOP_PID;

	for (int k=0; k<NB_AXIS; k++)
	{       
		//Platform

		_platform_position[k]=0.0f;
		_platform_speed[k]=0.0f;
		_platform_effortD[k]=0.0f;
		_platform_effortM[k]=0.0f;
		
	}	
		_platform_controllerType=TORQUE_CTRL;
		_platform_machineState=EMERGENCY;
		_platform_id=0;
		//ROS
		for (int k=0; k<NB_AXIS; k++)
	{

		_ros_position[k]=0.0f;
		_ros_speed[k]=0.0f;
		_ros_effort[k]=0.0f;
		_ros_filterAxisFS[k]=1.0f;

		_ros_posP[k]=0.0f;
		_ros_posI[k]=0.0f;
		_ros_posD[k]=0.0f;
		_ros_speedP[k]=0.0f;
		_ros_speedI[k]=0.0f;
		_ros_speedD[k]=0.0f;

		for (size_t c = 0; c < NB_POS_PID_C; c++)
		{
			
			_ros_paramP[c][k]=0.0f;
			_ros_paramI[c][k]=0.0f;
			_ros_paramD[c][k]=0.0f;
		}
		

	}
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++){
		_ros_effortComp[j];
	}
		
    _ros_forceSensor_controlled.setZero();
	_ros_forceModified.setZero();
	
    _ros_newState = (uint8_t)_platform_machineState;
    _ros_controllerType = (uint8_t)_platform_controllerType;
    _ros_defaultControl = true;

    _ros_controlledAxis = (int8_t)-1;

    _leg_grav_comp_effort.setZero();
    _legWrenchGravityComp.setZero();

}
footVarSynchronizer::~footVarSynchronizer()
{
	me->_n.shutdown();
}

bool footVarSynchronizer::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{	

	_mixedPlatformOn=false;
    if (!_n.getParam("mixedPlatformMode", _mixedPlatformOn))
    { 
      ROS_ERROR(" [%s footVarSync]: No mixedPlatformMode  param",Platform_Names[_platform_name]); 
    } 

	getPIDParams();

	
	std::vector<std::string> _fiPublishers;			
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_input/topics", _fiPublishers))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing fi_input/topics param",Platform_Names[_platform_name]); 
	}

	int _nbDesiredFootInputPublishers = _fiPublishers.size();
	
	if (_nbDesiredFootInputPublishers!=0)
	{
		_subDesiredFootInput.resize(_nbDesiredFootInputPublishers);
		_msgDesiredFootInput.resize(_nbDesiredFootInputPublishers);
		_flagDesiredFootInputsRead.resize(_nbDesiredFootInputPublishers);
		ROS_INFO("[%s footVarSync]: The list of publishers for the platform input are: ",Platform_Names[_platform_name]);
		for (unsigned int i  = 0; i<_nbDesiredFootInputPublishers; i++)
		{	
			_flagDesiredFootInputsRead[i]=false;
			ROS_INFO("[%s footVarSync]: Topic %i: %s",Platform_Names[_platform_name],i,_fiPublishers[i].c_str());
			_subDesiredFootInput[i] = _n.subscribe<custom_msgs::FootInputMsg>("/"+std::string(Platform_Names[_platform_name])+"/"+_fiPublishers[i], 1, boost::bind(&footVarSynchronizer::readDesiredFootInputs, this, _1,i), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		}
	}
	


	if (_platform_name==LEFT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg>(PLATFORM_PUBLISHER_NAME_LEFT, 1, boost::bind(&footVarSynchronizer::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		 
		 
		_subForceModified = _n.subscribe<geometry_msgs::WrenchStamped>("/left_platform/force_sensor_modifier/force_modified", 1,boost::bind(&footVarSynchronizer::readForceModified, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subLegGravCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&footVarSynchronizer::readLegGravCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());							
    	_subLegGravCompWrench = _n.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footVarSynchronizer::readLegGravityCompWrench, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_clientSetState=_n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_LEFT);
		_clientSetController=_n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_LEFT);
	}
	if (_platform_name==RIGHT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg>(PLATFORM_SUBSCRIBER_NAME_RIGHT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&footVarSynchronizer::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		
		_subForceModified = _n.subscribe<geometry_msgs::WrenchStamped>("/right_platform/force_sensor_modifier/force_modified", 1,boost::bind(&footVarSynchronizer::readForceModified, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subLegGravCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&footVarSynchronizer::readLegGravCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());	

		_subLegGravCompWrench = _n.subscribe<geometry_msgs::WrenchStamped>("/right_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footVarSynchronizer::readLegGravityCompWrench, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		
		_clientSetState=_n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_RIGHT);
		_clientSetController=_n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_RIGHT);
	}

  	_dynRecCallback = boost::bind(&footVarSynchronizer::dynamicReconfigureCallback, this, _1, _2);
  	_dynRecServer.setCallback(_dynRecCallback);

	//Subscriber definitions	
	signal(SIGINT,footVarSynchronizer::stopNode);
	
	if (_n.ok()) 
	{   
		ros::spinOnce();
		ROS_INFO("[%s footVarSync]: The supervisory variables synchronization of the foot interface is about to start ",Platform_Names[_platform_name]);
		//! Load the default gains for position control

		return true;
	}
	else 
	{
		ROS_ERROR("[%s footVarSync]: The ros node has a problem.",Platform_Names[_platform_name]);
		return false;
	}
}


void footVarSynchronizer::stopNode(int sig)
{
    me->_stop= true;
	me->publishFootInput(NULL);
}

void footVarSynchronizer::run()
{
  while (!_stop) 
  {	
	if (_flagPlatformOutCommStarted && _subFootOutput.getNumPublishers()!=0)
	{
		if ((_platform_id!=(uint8_t) _platform_name)&&(_platform_id!=UNKNOWN))
		{
			ROS_ERROR("[%s footVarSync]: This node for variables synchronization is acting on the wrong platform",Platform_Names[_platform_name]);
			break;
		}
		else
		{
			if (!_flagInitialConfig)
			{
				controlGainsDefault(-1);
				_config.machine_state = (uint8_t)_platform_machineState;
				_ros_newState = _config.machine_state;
				_config.controller_type = (uint8_t)_platform_controllerType;
				_ros_controllerType = _config.controller_type;
				// _config.desired_Position_X=_platform_position[X];
				// _config.desired_Position_Y=_platform_position[Y];
				// _config.desired_Position_PITCH = _platform_position[PITCH];
				// _config.desired_Position_ROLL = _platform_position[ROLL];
				// _config.desired_Position_YAW = _platform_position[YAW];
				_config.use_default_gains=true;
				_config.send_pid_gains=false;
				_config.load_param_pid_gains=false;
				_ros_defaultControl=true;
				_ros_position=_platform_position;
				_config.controlled_axis=-1;
				_dynRecServer.setConfigDefault(_config);
				_dynRecServer.updateConfig(_config);
				_configPrev = _config;
				_msgFootOutputPrev=_msgFootOutput;
				_flagInitialConfig=true;
				ROS_INFO("[%s footVarSync]: Updating default parameters from the platform in the rqt_reconfig...",Platform_Names[_platform_name]);
				ros::spinOnce();
				
			} else if(_subFootOutput.getNumPublishers()!=0)
			{
				checkWhichPIDGainsToUse();
				// cout<<_myPIDCategory<<endl;
				if (!_ros_defaultControl && _flagLoadPIDGains)
				{ 
					controlGainsDefault(-1);
					_flagUpdateConfig = true;
				}
				if (_flagWasDynReconfCalled)
				{
					updateInternalVariables();
					changeParamCheck();
					_flagParamsActionsTaken = false;
					requestDoActionsParams();
					if (_flagParamsActionsTaken)
						{ updateConfigAfterParamsChanged();}
					_flagWasDynReconfCalled = false;
				}
				
				if (_flagOutputMessageReceived)
				{
					changedPlatformCheck();
					_flagPlatformActionsTaken= false;
					requestDoActionsPlatform();
					if (_flagPlatformActionsTaken) {updateConfigAfterPlatformChanged();}
					_flagOutputMessageReceived = false;
				}
				

				if (_flagParamsActionsTaken)
				{
					_configPrev = _config;
					_flagParamsActionsTaken= false;
				}
				if (_flagPlatformActionsTaken)
				{
					_msgFootOutputPrev = _msgFootOutput;
					_flagPlatformActionsTaken= false;
				}


				if (_flagForceModifiedConnected)
				{ 
					correctForceForLegCompensation();

					if (abs(_ros_forceModified.segment(0,3).norm()) > HUMAN_ON_PLATFORM_THRESHOLD) {
						if(!_flagHumanOnPlatform)
						{ 
							ROS_INFO("[%s footVarSync]: Probably there is a human on the platform",Platform_Names[_platform_name]); 
							_flagHumanOnPlatform=true;
						}
					}
					else
					{
						if(_flagHumanOnPlatform)
						{ 
							ROS_INFO("[%s footVarSync]: Probably there is NOT a human on the platform",Platform_Names[_platform_name]); 
							_flagHumanOnPlatform=false;
						}
					}
					}
					
					if (_flagHumanOnPlatform && _flagCompensateLeg && _subLegGravCompTorques.getNumPublishers()!=0)
					{	
						_ros_effort = _leg_grav_comp_effort;
						//cout<<_ros_effort.transpose()<<endl;
					}
					else
					{
						_ros_effort.setZero();
					}
				
			}
			

		}
	}
	else
	{
		_flagInitialConfig=false;
	}
          publishFootInput(&_flagPositionOnlyPublished);
		  
        ros::spinOnce();
	_loopRate.sleep();
  }
  ROS_INFO("[%s footVarSync]: Parameters setting and variables synchronization stoped",Platform_Names[_platform_name]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}



void footVarSynchronizer::checkWhichPIDGainsToUse()
{	
	if (_mixedPlatformOn)
	{	
		if(_platform_machineState==TELEOPERATION)
		{
			if ((Platform_Name) _mainPlatform == _platform_id)
			{
				_myPIDCategory = _msgTwoFeetOneTool.currentControlMode == (uint8_t) TOOL_POSITION_CTRL ? MP_TOOL_POS_PID : MP_TOOL_SPEED_PID;
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

	if (_flagControlZeroEffort) {
		_ros_effort.setConstant(0.0f);
        ROS_DEBUG("Sending zero torque to the platform");
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

	//! Verify

	// Check Position

	if (fabs(_msgFootOutput.platform_position[0] - _msgFootOutputPrev.platform_position[0]) +
		fabs(_msgFootOutput.platform_position[1] - _msgFootOutputPrev.platform_position[1]) +
		fabs(_msgFootOutput.platform_position[2] - _msgFootOutputPrev.platform_position[2]) + 
		fabs(_msgFootOutput.platform_position[3] - _msgFootOutputPrev.platform_position[3]) +
		fabs(_msgFootOutput.platform_position[4] - _msgFootOutputPrev.platform_position[4]) <= FLT_EPSILON)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = false;
	}

	// Check Speed

	if (fabs(_msgFootOutput.platform_speed[0] - _msgFootOutputPrev.platform_speed[0]) +
		fabs(_msgFootOutput.platform_speed[1] - _msgFootOutputPrev.platform_speed[1]) +
		fabs(_msgFootOutput.platform_speed[2] - _msgFootOutputPrev.platform_speed[2]) + 
		fabs(_msgFootOutput.platform_speed[3] - _msgFootOutputPrev.platform_speed[3]) +
		fabs(_msgFootOutput.platform_speed[4] - _msgFootOutputPrev.platform_speed[4]) <= FLT_EPSILON)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = false;
	}

	// Check EffortD

	if (fabs(_msgFootOutput.platform_effortD[0] - _msgFootOutputPrev.platform_effortD[0]) +
		fabs(_msgFootOutput.platform_effortD[1] - _msgFootOutputPrev.platform_effortD[1]) +
		fabs(_msgFootOutput.platform_effortD[2] - _msgFootOutputPrev.platform_effortD[2]) + 
		fabs(_msgFootOutput.platform_effortD[3] - _msgFootOutputPrev.platform_effortD[3]) +
		fabs(_msgFootOutput.platform_effortD[4] - _msgFootOutputPrev.platform_effortD[4]) <= FLT_EPSILON)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = false;
	}

	// Check EffortM

	if (fabs(_msgFootOutput.platform_effortM[0] - _msgFootOutputPrev.platform_effortM[0]) +
		fabs(_msgFootOutput.platform_effortM[1] - _msgFootOutputPrev.platform_effortM[1]) +
		fabs(_msgFootOutput.platform_effortM[2] - _msgFootOutputPrev.platform_effortM[2]) + 
		fabs(_msgFootOutput.platform_effortM[3] - _msgFootOutputPrev.platform_effortM[3]) +
		fabs(_msgFootOutput.platform_effortM[4] - _msgFootOutputPrev.platform_effortM[4]) <= FLT_EPSILON)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTM] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTM] = false;
	}

	// Check State

	if (_msgFootOutput.platform_machineState == _msgFootOutputPrev.platform_machineState)
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


void footVarSynchronizer::updateConfigAfterParamsChanged()
	{
		_flagUpdateConfig = false;

	

		if (_flagPositionOnlyPublished && _flagControlThisPosition)
		{
			_config.send_this_position = false;
			_flagControlThisPosition = false;
			_flagUpdateConfig = true;
		}

		if (_flagEffortOnlyPublished && _flagControlZeroEffort) {
			_config.send_zero_effort = false;
			_flagControlZeroEffort = false;
			_flagUpdateConfig = true;
		}

                if (_flagUpdateConfig)
		{
			//_mutex.lock();
			_dynRecServer.updateConfig(_config);
			//_mutex.unlock();
			_flagUpdateConfig = false;
		}

	}

void footVarSynchronizer::updateConfigAfterPlatformChanged()
	{
		_flagUpdateConfig = true;

		if (_flagUpdateConfig)
		{
			//_mutex.lock();
			_dynRecServer.updateConfig(_config);
			//_mutex.unlock();
			_flagUpdateConfig = false;
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
		ROS_INFO("[%s footVarSync]: Zero torque send (no,Platform_Names[_platform_name] "
				"acknowledgement)!");
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

void footVarSynchronizer::processAllPublishers()
{
	_msgTotalDesiredFootInput.ros_effort.fill(0.0f);
	_msgTotalDesiredFootInput.ros_speed.fill(0.0f);
	_msgTotalDesiredFootInput.ros_position.fill(0.0f);
	_msgTotalDesiredFootInput.ros_kp.fill(0.0f);
	_msgTotalDesiredFootInput.ros_ki.fill(0.0f);
	_msgTotalDesiredFootInput.ros_kd.fill(0.0f);
	_msgTotalDesiredFootInput.ros_filterAxisForce.fill(1.0f);
	
	bool inputsRead=true;

	for (size_t i = 0; i < _nbDesiredFootInputPublishers; i++)
	{
		inputsRead = inputsRead && _flagDesiredFootInputsRead[i];
	}
	
	if (inputsRead)
	{

		for (unsigned int i=0; i<_nbDesiredFootInputPublishers; i++)
		{
			if (_flagDesiredFootInputsRead[i] && _subDesiredFootInput[i].getNumPublishers()!=0)
			{
				for (unsigned int j=0; j<NB_PLATFORM_AXIS; j++)
				{
					if (_platform_controllerType==POSITION_CTRL)
					{
						_msgTotalDesiredFootInput.ros_position[j] += _msgDesiredFootInput[i].ros_position[j];
					}else if (_platform_controllerType==SPEED_CTRL)
					{
						_msgTotalDesiredFootInput.ros_speed[j] += _msgDesiredFootInput[i].ros_speed[j];
					}
					if (_platform_machineState==TELEOPERATION)
					{
						if (fabs(_msgDesiredFootInput[i].ros_kp[j])>FLT_EPSILON
							|| fabs(_msgDesiredFootInput[i].ros_ki[j])>FLT_EPSILON ||
							fabs(_msgDesiredFootInput[i].ros_kd[j])>FLT_EPSILON)
							{
								_flagPIDGainsByInput=true;
								_msgTotalDesiredFootInput.ros_kp[j] += _msgDesiredFootInput[i].ros_kp[j];			
								_msgTotalDesiredFootInput.ros_ki[j] += _msgDesiredFootInput[i].ros_ki[j];			
								_msgTotalDesiredFootInput.ros_kd[j] += _msgDesiredFootInput[i].ros_kd[j];			
							}
					}
					_msgTotalDesiredFootInput.ros_effort[j] += _msgDesiredFootInput[i].ros_effort[j];
					_msgTotalDesiredFootInput.ros_filterAxisForce[j] *= _msgDesiredFootInput[i].ros_filterAxisForce[j];
				}
				
			}	
		}
	}
}

void footVarSynchronizer::publishFootInput(bool* flagVariableOnly_) {
  //! Keep send the same valuest that the platform is broadcasting
  processAllPublishers();	
 // _mutex.lock();
 if (!_stop)
 {
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgFootInput.ros_position[rosAxis[k]] = _ros_position[k] + _msgTotalDesiredFootInput.ros_position[rosAxis[k]];
    _msgFootInput.ros_speed[rosAxis[k]] = _ros_speed[k] + _msgTotalDesiredFootInput.ros_speed[rosAxis[k]] ;
    _msgFootInput.ros_effort[rosAxis[k]] = _ros_effort[k] + _msgTotalDesiredFootInput.ros_effort[rosAxis[k]];
	_msgFootInput.ros_filterAxisForce[rosAxis[k]] = _ros_filterAxisFS[k] * _msgTotalDesiredFootInput.ros_filterAxisForce[rosAxis[k]];
	
	if (!_flagPIDGainsByInput)
	{
		if (_platform_controllerType==POSITION_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_ros_posP[k],0.0,_ros_posP_Max[k]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_ros_posI[k],0.0,_ros_posI_Max[k]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_ros_posD[k],0.0,_ros_posD_Max[k]);
		} else if (_platform_controllerType==SPEED_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_ros_speedP[k],0.0,_ros_speedP_Max[k]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_ros_speedI[k],0.0,_ros_speedI_Max[k]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_ros_speedD[k],0.0,_ros_speedD_Max[k]);
		}
	}else
	{
		if (_platform_controllerType==POSITION_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_kp[rosAxis[k]],0.0,_ros_posP_Max[k]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_ki[rosAxis[k]],0.0,_ros_posI_Max[k]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_kd[rosAxis[k]],0.0,_ros_posD_Max[k]);
		} else if (_platform_controllerType==SPEED_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_kp[rosAxis[k]],0.0,_ros_speedP_Max[k]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_ki[rosAxis[k]],0.0,_ros_speedI_Max[k]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_kd[rosAxis[k]],0.0,_ros_speedD_Max[k]);
		}
	}
	
  }
  if (_flagForceModifiedConnected && _subForceModified.getNumPublishers()!=0)
  {
	for (int k = 0; k < NB_AXIS_WRENCH; k++) {
		_msgFootInput.ros_forceSensor[k] = _ros_forceSensor_controlled[k]; // rosAxis not applied
  	}
  }else
  {
	for (int k = 0; k < NB_AXIS_WRENCH; k++) {
		_msgFootInput.ros_forceSensor[k] = 0.0; // rosAxis not applied
  	}
  }
 }
 else
 {
	 _msgFootInput.ros_effort.fill(0.0f);
	 _msgFootInput.ros_filterAxisForce.fill(0.0f);
	 _msgFootInput.ros_forceSensor.fill(0.0f);
	 _msgFootInput.ros_kp.fill(0.0f);
	 _msgFootInput.ros_ki.fill(0.0f);
	 _msgFootInput.ros_kd.fill(0.0f);
	 for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
	 {
		 _msgFootInput.ros_position[i] = _platform_position[i];
	 }
	 _msgFootInput.ros_speed.fill(0.0f);
	 _msgFootInput.ros_effort.fill(0.0f);
 }

  _pubFootInput.publish(_msgFootInput);
  if (flagVariableOnly_!=NULL)
  {
  	*flagVariableOnly_ = true;
  }
  //_mutex.unlock();
}


void footVarSynchronizer::fetchFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg)
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


void footVarSynchronizer::requestSetState(){
	//_mutex.lock();
	_flagSetStateRequested=true;
	_srvSetState.request.ros_machineState=_ros_newState;
	_srvSetState.request.ros_effortComp.resize(NB_EFFORT_COMPONENTS);
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++)
	{
		_srvSetState.request.ros_effortComp[j]=_ros_effortComp[j];
	}

	_clientSetState.call(_srvSetState);

	_flagResponseSetState = _srvSetState.response.platform_newState;
	//_mutex.unlock();
}

void footVarSynchronizer::requestSetController(){
	//_mutex.lock();
	_flagSetControllerRequested=true;
	_srvSetController.request.ros_controllerType=_ros_controllerType;
	_srvSetController.request.ros_defaultControl=_ros_defaultControl;
	_srvSetController.request.ros_controlledAxis=_ros_controlledAxis;

	// for (int k=0; k<NB_AXIS; k++)
	// {
	// 	_srvSetController.request.ros_posP[rosAxis[k]]=_ros_posP[k];
	// 	_srvSetController.request.ros_posI[rosAxis[k]]=_ros_posI[k];
	// 	_srvSetController.request.ros_posD[rosAxis[k]]=_ros_posD[k];

	// 	_srvSetController.request.ros_speedP[rosAxis[k]]=_ros_speedP[k];
	// 	_srvSetController.request.ros_speedI[rosAxis[k]]=_ros_speedI[k];
	// 	_srvSetController.request.ros_speedD[rosAxis[k]]=_ros_speedD[k];
	// }

	_clientSetController.call(_srvSetController);
	ROS_INFO("[%s footVarSync]: Set Controller Request. Updating the parameters of the machine state",Platform_Names[_platform_name]);
	_flagResponseSetController = _srvSetController.response.platform_controlOk;

	//_mutex.unlock();
}


void footVarSynchronizer::dynamicReconfigureCallback(foot_variables_sync::machineStateParamsConfig &config, uint32_t level)
{
//   ROS_INFO("[%s footVarSync]: Reconfigure request. Updating the parameters of the machine state",Platform_Names[_platform_name]);
  	_flagWasDynReconfCalled=true;
	_config = config;
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
					_config.kp_X = _ros_paramP[_myPIDCategory][X];
					_config.ki_X = _ros_paramI[_myPIDCategory][X];
					_config.kd_X = _ros_paramD[_myPIDCategory][X];
					_ros_posP[X] = Utils_math<double>::bound(_config.kp_X,0,5000.0);
					_ros_posI[X] = Utils_math<double>::bound(_config.ki_X,0,5000.0);
					_ros_posD[X] = Utils_math<double>::bound(_config.kd_X,0,30.0);
					break;
				}
				case (Y):
				{
					_config.kp_Y = _ros_paramP[_myPIDCategory][Y];
					_config.ki_Y = _ros_paramI[_myPIDCategory][Y];
					_config.kd_Y = _ros_paramD[_myPIDCategory][Y];
					_ros_posP[Y] = Utils_math<double>::bound(_config.kp_Y,0,5000.0);
					_ros_posI[Y] = Utils_math<double>::bound(_config.ki_Y,0,5000.0);
					_ros_posD[Y] = Utils_math<double>::bound(_config.kd_Y,0,30.0);
					break;
				}
				case (PITCH):
				{
					_config.kp_PITCH = _ros_paramP[_myPIDCategory][PITCH];
					_config.ki_PITCH = _ros_paramI[_myPIDCategory][PITCH];
					_config.kd_PITCH = _ros_paramD[_myPIDCategory][PITCH];
					_ros_posP[PITCH] = Utils_math<double>::bound(_config.kp_PITCH,0,10000.0);
					_ros_posI[PITCH] = Utils_math<double>::bound(_config.ki_PITCH,0,10000.0);
					_ros_posD[PITCH] = Utils_math<double>::bound(_config.kd_PITCH,0,100.0);
					break;
				}
				case (ROLL):
				{
					_config.kp_ROLL = _ros_paramP[_myPIDCategory][ROLL];
					_config.ki_ROLL = _ros_paramI[_myPIDCategory][ROLL];
					_config.kd_ROLL = _ros_paramD[_myPIDCategory][ROLL];
					_ros_posP[ROLL] = Utils_math<double>::bound(_config.kp_ROLL,0,10000);
					_ros_posI[ROLL] = Utils_math<double>::bound(_config.ki_ROLL,0,10000);
					_ros_posD[ROLL] = Utils_math<double>::bound(_config.kd_ROLL,0,100);
					break;
				}
				case (YAW):
				{
					_config.kp_YAW = _ros_paramP[_myPIDCategory][YAW];
					_config.ki_YAW = _ros_paramI[_myPIDCategory][YAW];
					_config.kd_YAW = _ros_paramD[_myPIDCategory][YAW];
					_ros_posP[YAW] = Utils_math<double>::bound(_config.kp_YAW,0,10000);
					_ros_posI[YAW] = Utils_math<double>::bound(_config.ki_YAW,0,10000);
					_ros_posD[YAW] = Utils_math<double>::bound(_config.kd_YAW,0,100);
					break;
				}
			}
		}
	}
		
}

void footVarSynchronizer::resetDesiredPositionToCurrent(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      resetDesiredPositionToCurrent(k);
    }
  }

  else{
    switch(axis_){
        case(X): {_config.desired_Position_X=_platform_position[X];}
        case(Y): {_config.desired_Position_Y=_platform_position[Y];}
        case(PITCH): {_config.desired_Position_PITCH=_platform_position[PITCH];}
        case(ROLL): {_config.desired_Position_ROLL=_platform_position[ROLL];}
        case(YAW): {_config.desired_Position_YAW=_platform_position[YAW];}
      }
  }
}

void footVarSynchronizer::correctForceForLegCompensation() {
	 
	 if (_flagHumanOnPlatform && _flagCompensateLeg && _flagLegCompWrenchRead && _subLegGravCompWrench.getNumPublishers()!=0)
	 {
		_ros_forceSensor_controlled = _ros_forceModified;// + _legWrenchGravityComp;                               
	 }

	 else
	 {
		_ros_forceSensor_controlled = _ros_forceModified;
	 }

}

void footVarSynchronizer::readForceModified(
   const geometry_msgs::WrenchStamped::ConstPtr &msg) {
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
    
	for (unsigned int i = 0; i<NB_AXIS; i++)
	{
		_leg_grav_comp_effort(i) = msg->ros_effort[rosAxis[i]];
	}

  if (!_flagLegCompTorquesRead) {
    ROS_INFO("[%s footVarSync]: Receiving Leg Compensation Torques",Platform_Names[_platform_name]);
  }
  _flagLegCompTorquesRead = true;
}


void footVarSynchronizer::readDesiredFootInputs(const custom_msgs::FootInputMsg::ConstPtr &msg,unsigned int n_)
{
	
	_msgDesiredFootInput[n_].ros_effort = msg->ros_effort;
	_msgDesiredFootInput[n_].ros_kp = msg->ros_kp;
	_msgDesiredFootInput[n_].ros_ki = msg->ros_ki;
	_msgDesiredFootInput[n_].ros_kd = msg->ros_kd;
	_msgDesiredFootInput[n_].ros_forceSensor = msg->ros_forceSensor;
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

void footVarSynchronizer::getPIDParams(){
	if (_mixedPlatformOn)
	{
		std::string mainPlatform_="right";
		if (!_n.getParam("/mixed_platform/mainPlatform", mainPlatform_))
		{ 
		ROS_ERROR(" [%s footVarSync]: No /mixed_platform/mainPlatform  param",Platform_Names[_platform_name]); 
		} 

		_mainPlatform = mainPlatform_.compare("right")==0 ? RIGHT_PLATFORM_ID : LEFT_PLATFORM_ID;
		
		std::vector<double> pToolPosControl_;
		std::vector<double> dToolPosControl_;
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolPos/p", pToolPosControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolPos/p",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP[MP_TOOL_POS_PID][i] = pToolPosControl_[i];
			}
		}
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolPos/d", dToolPosControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolPos/d",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramD[MP_TOOL_POS_PID][i] = dToolPosControl_[i];
			}
		}

		std::vector<double> pToolSpeedControl_;
		std::vector<double> dToolSpeedControl_;
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolSpeed/p", pToolSpeedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolSpeed/p",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP[MP_TOOL_SPEED_PID][i] = pToolSpeedControl_[i];
			}
		}
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolSpeed/d", dToolSpeedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolSpeed/d",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramD[MP_TOOL_SPEED_PID][i] = dToolSpeedControl_[i];
			}
		}
		
		std::vector<double> pToolMixedControl_;
		std::vector<double> dToolMixedControl_;
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolMixed/p", pToolMixedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolMixed/p",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramP[MP_TOOL_MIXED_PID][i] = pToolMixedControl_[i];
			}
		}
		
		if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/toolMixed/d", dToolMixedControl_))
		{ 
			ROS_ERROR("[%s footVarSync]: Missing /toolControl/toolMixed/d",Platform_Names[_platform_name]); 
		}
		else
		{
			for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
			{
				_ros_paramD[MP_TOOL_MIXED_PID][i] = dToolMixedControl_[i];
			}
		}

			std::string toolControlTopic_ = "/mixedPlatform/platformState";
			if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/toolControl/topic", toolControlTopic_))
			{ 
			
				ROS_ERROR("[%s footVarSync]: Missing /toolControl/topic",Platform_Names[_platform_name]); 
			}
			else
			{
				_subPlatformControlFromTool = _n.subscribe<custom_msgs::TwoFeetOneToolMsg>(toolControlTopic_, 1, boost::bind(&footVarSynchronizer::readTwoFeetOneToolMsg, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
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

			_ros_paramP[S_TELEOP_PID][i] = pTeloperation_[i];
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
			_ros_paramP[S_ROBOT_CTRL_PID][i] = pRobotStControl_[i];
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