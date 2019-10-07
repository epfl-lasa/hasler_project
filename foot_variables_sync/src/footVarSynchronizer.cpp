#include "footVarSynchronizer.h"
#include "../../5_axis_platform/lib/platform/src/definitions.h"
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

footVarSynchronizer* footVarSynchronizer::me = NULL;

footVarSynchronizer::footVarSynchronizer(ros::NodeHandle &n_1, double frequency,footVarSynchronizer::Platform_Name platform_id): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_dt(1.0f/frequency)
{

	me=this;
	_stop = false;
	_flagControlThisPosition=false;
	_flagCapturePlatformPosition = false;
	_flagWasDynReconfCalled= false;
	_flagOutputMessageReceived=false;
	_flagInitialConfig = false;
	_flagParamsActionsTaken= false;
	_flagPlatformActionsTaken= false;

	_flagPlatformInCommStarted=false;
	_flagPositionOnlyPublished=false;

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
    
	for (int k=0; k<NB_AXIS; k++)
	{       
		//Platform

		_platform_position[k]=0.0f;
		_platform_speed[k]=0.0f;
		_platform_effortD[k]=0.0f;
		_platform_effortM[k]=0.0f;
		
	}	
		_platform_controllerType=TORQUE_ONLY;
		_platform_machineState=EMERGENCY;
		_platform_id=0;
		//ROS
		for (int k=0; k<NB_AXIS; k++)
	{

		_ros_position[k]=0.0f;
		_ros_speed[k]=0.0f;
		_ros_effort[k]=0.0f;

		_ros_posP[k]=0.0f;
		_ros_posI[k]=0.0f;
		_ros_posD[k]=0.0f;
		_ros_speedP[k]=0.0f;
		_ros_speedI[k]=0.0f;
		_ros_speedD[k]=0.0f;

	}
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++){
		_ros_effortComp[j];
	}
		
	_ros_newState=(uint8_t)_platform_machineState;
	_ros_controllerType=(uint8_t)_platform_controllerType;
	_ros_defaultControl=true;
	
	_ros_controlledAxis= (int8_t) -1;

}
footVarSynchronizer::~footVarSynchronizer()
{
	me->_n.shutdown();
}

bool footVarSynchronizer::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

	if (_platform_name==LEFT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_LEFT,1, boost::bind(&footVarSynchronizer::fetchFootOutput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_subFootInput = _n.subscribe<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_LEFT,1, boost::bind(&footVarSynchronizer::sniffFootInput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	
		_clientSetState=_n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_LEFT);
		_clientSetController=_n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_LEFT);
	}
	if (_platform_name==RIGHT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_RIGHT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_RIGHT,1, boost::bind(&footVarSynchronizer::fetchFootOutput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_subFootInput = _n.subscribe<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_RIGHT,1, boost::bind(&footVarSynchronizer::sniffFootInput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
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
		ROS_INFO("The supervisory variables synchronization of the foot interface is about to start ");
		//! Load the default gains for position control

		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void footVarSynchronizer::stopNode(int sig)
{
    me->_stop= true;
}

void footVarSynchronizer::run()
{
  while (!_stop) 
  {	

	if (_flagPlatformOutCommStarted)
	{
		if ((_platform_id!=(uint8_t) _platform_name)&&(_platform_id!=UNKNOWN))
		{
			ROS_ERROR("This node for variables synchronization is acting on the wrong platform");
			break;
		}
		else
		{
			if (!_flagInitialConfig && _flagPlatformOutCommStarted)
			{
				controlGainsDefault(-1);
				_config.machine_state = (uint8_t)_platform_machineState;
				_ros_newState = _config.machine_state;
				_config.controller_type = (uint8_t)_platform_controllerType;
				_ros_controllerType = _config.controller_type;
				_config.desired_Position_X=_platform_position[X];
				_config.desired_Position_Y=_platform_position[Y];
				_config.desired_Position_PITCH = _platform_position[PITCH];
				_config.desired_Position_ROLL = _platform_position[ROLL];
				_config.desired_Position_YAW = _platform_position[YAW];
				_config.use_default_gains=true;
				_ros_defaultControl=true;
				_ros_position=_platform_position;
				_dynRecServer.setConfigDefault(_config);
				_dynRecServer.updateConfig(_config);
				_configPrev = _config;
				_msgFootOutputPrev=_msgFootOutput;
				_flagInitialConfig=true;
				ROS_INFO("Updating default parameters from the platform in the rqt_reconfig...");
			}
			else
			{
				changeParamCheck();
				if (_flagWasDynReconfCalled){
					changeParamCheck();
					_flagParamsActionsTaken= false;
					requestDoActionsParams();
					if (_flagParamsActionsTaken)
						{ updateConfigAfterParamsChanged();}
					_flagWasDynReconfCalled = false;
				}
				changedPlatformCheck();
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
			}
		}
	}
	ros::spinOnce();
	_loopRate.sleep();
  }
  ROS_INFO("Parameters setting and variables synchronization stoped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
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

	if ( (_config.effortComp_compensation == _configPrev.effortComp_compensation) &&
		 (_config.effortComp_constrains == _configPrev.effortComp_constrains) &&
		 (_config.effortComp_feedforward == _configPrev.effortComp_feedforward) &&
		 (_config.effortComp_normal == _configPrev.effortComp_normal) )
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
	if ((_config.desired_Position_X == _configPrev.desired_Position_X) && (_config.desired_Position_Y == _configPrev.desired_Position_Y) &&
		(_config.desired_Position_PITCH == _configPrev.desired_Position_PITCH) && (_config.desired_Position_ROLL == _configPrev.desired_Position_ROLL) &&
		(_config.desired_Position_YAW == _configPrev.desired_Position_YAW))
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

	if ((_config.kp_Position_X == _configPrev.kp_Position_X) &&
		(_config.kp_Position_Y == _configPrev.kp_Position_Y) &&
		(_config.kp_Position_PITCH == _configPrev.kp_Position_PITCH) &&
		(_config.kp_Position_ROLL == _configPrev.kp_Position_ROLL) &&
		(_config.kp_Position_YAW == _configPrev.kp_Position_YAW) &&
		(_config.kp_Position_Y == _configPrev.kp_Position_Y) &&
		(_config.ki_Position_X == _configPrev.ki_Position_X) &&
		(_config.ki_Position_Y == _configPrev.ki_Position_Y) &&
		(_config.ki_Position_PITCH == _configPrev.ki_Position_PITCH) &&
		(_config.ki_Position_ROLL == _configPrev.ki_Position_ROLL) &&
		(_config.ki_Position_YAW == _configPrev.ki_Position_YAW) &&
		(_config.kd_Position_X == _configPrev.kd_Position_X) &&
		(_config.kd_Position_Y == _configPrev.kd_Position_Y) &&
		(_config.kd_Position_PITCH == _configPrev.kd_Position_PITCH) &&
		(_config.kd_Position_ROLL == _configPrev.kd_Position_ROLL) &&
		(_config.kd_Position_YAW == _configPrev.kd_Position_YAW) )
		
	{
		_flagIsParamStillSame[Params_Category::PID_POS] = true;
	}

	else
	{
		_flagIsParamStillSame[Params_Category::PID_POS] = false;
	}

	// Check PID Gains Speed

	if ((_config.kp_Speed_X == _configPrev.kp_Speed_X) &&
		(_config.kp_Speed_Y == _configPrev.kp_Speed_Y) &&
		(_config.kp_Speed_PITCH == _configPrev.kp_Speed_PITCH) &&
		(_config.kp_Speed_ROLL == _configPrev.kp_Speed_ROLL) &&
		(_config.kp_Speed_YAW == _configPrev.kp_Speed_YAW) &&
		(_config.kp_Speed_Y == _configPrev.kp_Speed_Y) &&
		(_config.ki_Speed_X == _configPrev.ki_Speed_X) &&
		(_config.ki_Speed_Y == _configPrev.ki_Speed_Y) &&
		(_config.ki_Speed_PITCH == _configPrev.ki_Speed_PITCH) &&
		(_config.ki_Speed_ROLL == _configPrev.ki_Speed_ROLL) &&
		(_config.ki_Speed_YAW == _configPrev.ki_Speed_YAW) &&
		(_config.kd_Speed_X == _configPrev.kd_Speed_X) &&
		(_config.kd_Speed_Y == _configPrev.kd_Speed_Y) &&
		(_config.kd_Speed_PITCH == _configPrev.kd_Speed_PITCH) &&
		(_config.kd_Speed_ROLL == _configPrev.kd_Speed_ROLL) &&
		(_config.kd_Speed_YAW == _configPrev.kd_Speed_YAW))

	{
		_flagIsParamStillSame[Params_Category::PID_SPEED] = true;
	}

	else
	{
		_flagIsParamStillSame[Params_Category::PID_SPEED] = false;
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

	if ((_msgFootOutput.platform_position == _msgFootOutputPrev.platform_position))
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_POS] = false;
	}

	// Check Speed

	if ((_msgFootOutput.platform_speed == _msgFootOutputPrev.platform_speed))
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_SPEED] = false;
	}

	// Check EffortD

	if ((_msgFootOutput.platform_effortD == _msgFootOutputPrev.platform_effortD))
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_EFFORTD] = false;
	}

	// Check EffortM

	if ((_msgFootOutput.platform_effortM == _msgFootOutputPrev.platform_effortM))
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
		_flagIsPlatformStillSame[FootOutput_Category::FO_M_STATE] = false;
		ROS_INFO("The Platform changed its state!");
	}

	// Check Controller Type

	if (_msgFootOutput.platform_controllerType == _msgFootOutputPrev.platform_controllerType)
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE] = true;
	}
	else
	{
		_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE] = false;
		ROS_INFO("The Platform changed its controller type!");
	}

}


void footVarSynchronizer::updateConfigAfterParamsChanged()
	{
		_flagUpdateConfig = false;

		if (_ros_defaultControl)
		{
			controlGainsDefault(-1);
			_flagUpdateConfig = true;
		}

		if (_flagPositionOnlyPublished && _flagControlThisPosition)
		{
			_config.send_this_position = false;
			_flagControlThisPosition = false;
			_flagUpdateConfig = true;
		}

		if (_flagUpdateConfig)
		{
			_mutex.lock();
			_dynRecServer.updateConfig(_config);
			_mutex.unlock();
			_flagUpdateConfig = false;
		}

	}

void footVarSynchronizer::updateConfigAfterPlatformChanged()
	{
		_flagUpdateConfig = true;

		if (_flagUpdateConfig)
		{
			_mutex.lock();
			_dynRecServer.updateConfig(_config);
			_mutex.unlock();
			_flagUpdateConfig = false;
		}
	}

	void footVarSynchronizer::requestDoActionsParams()
	{

		//! Send Variables of the Set State Service
		if ( (!_flagIsParamStillSame[Params_Category::M_STATE]) ||
			 (!_flagIsParamStillSame[Params_Category::EFF_COMP]) )
		{
			_flagSetStateRequested=false;
			_flagResponseSetState=false;
			requestSetState();
			_flagParamsActionsTaken=true;
			if (_flagResponseSetState){ //! Verify the service went through
				ROS_INFO("A new state has been sent to the platform ");
  				ROS_INFO("Resetting GAINS ");
				return;
			}
			else{
				ROS_INFO("The state machine of the platform is already up to date ");
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
						{ ROS_INFO("The controlled axis has changed to: ALL" );}
						else {
						  ROS_INFO("The controlled axis has changed to: %s", Axis_names[_config.controlled_axis]);
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
		publishPositionOnly();
		ROS_INFO("A new desired position has been sent to the platform!");
		_flagParamsActionsTaken= true;
	}
	if (_flagControlThisPosition && _flagCapturePlatformPosition)
	{
		ROS_WARN("You cannot sent a position while capturing the platform position");
	}
}

void footVarSynchronizer::requestDoActionsPlatform()
{
	if ((!_flagIsPlatformStillSame[FootOutput_Category::FO_M_STATE]) 
		// && (_flagIsParamStillSame[Params_Category::M_STATE])
	)
	{
		_config.machine_state=_platform_machineState;
		_ros_newState=_config.machine_state;
		_flagPlatformActionsTaken= true;
	}

	if ((!_flagIsPlatformStillSame[FootOutput_Category::FO_C_TYPE]) 
		// && (_flagIsParamStillSame[Params_Category::C_TYPE]) 
		)
	{
		_config.controller_type = _platform_controllerType;
		_ros_controllerType=_config.controller_type;
		_flagPlatformActionsTaken= true;
	}

	if (
		(!_flagIsPlatformStillSame[FootOutput_Category::FO_POS]
		 && _flagCapturePlatformPosition) 
		// && (_flagIsParamStillSame[Params_Category::DES_POS])
		)
	{
		_config.desired_Position_X = _platform_position[X];
		_config.desired_Position_Y = _platform_position[Y];
		_config.desired_Position_PITCH = _platform_position[PITCH];
		_config.desired_Position_ROLL = _platform_position[ROLL];
		_config.desired_Position_YAW = _platform_position[YAW];
		_ros_position=_platform_position;
		_flagPlatformActionsTaken= true;
	}
}

void footVarSynchronizer::publishPositionOnly() 
{
	_mutex.lock();
	for (int k=0; k<NB_AXIS; k++)
		{
			_msgFootInput.ros_position[k]=_ros_position[k];
			//! Keep send the same valuest that the platform is broadcasting
			_msgFootInput.ros_speed[k]=_ros_speed[k];
			_msgFootInput.ros_effort[k]=_ros_effort[k];
		}
	_pubFootInput.publish(_msgFootInput);
	_flagPositionOnlyPublished = true;
	_mutex.unlock();
}

void footVarSynchronizer::sniffFootInput(const custom_msgs::FootInputMsg_v2::ConstPtr& msg)
{
	for (int k=0; k<NB_AXIS; k++)
		{
			if (!_flagControlThisPosition){
				_ros_position[k]=msg->ros_position[k];
			}
				_ros_speed[k]=msg->ros_speed[k];
				_ros_effort[k]=msg->ros_effort[k];
		}	
	if(!_flagPlatformInCommStarted)
	{_flagPlatformInCommStarted=true;}
} 

void footVarSynchronizer::fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg)
{
	_msgFootOutput = *msg;
	_flagOutputMessageReceived=true;
	_platform_id = msg->platform_id; 
	for (int k=0; k<NB_AXIS; k++)
		{
			_platform_position[k]=msg->platform_position[k];
			_platform_speed[k]=msg->platform_speed[k];
			_platform_effortD[k]=msg->platform_effortD[k];
			_platform_effortM[k]=msg->platform_effortM[k];
		}	
	_platform_machineState= (footVarSynchronizer::State) msg->platform_machineState;
	_platform_controllerType= (footVarSynchronizer::Controller) msg->platform_controllerType;
	if(!_flagPlatformOutCommStarted)
	{_flagPlatformOutCommStarted=true;}
} 


void footVarSynchronizer::requestSetState(){
	_mutex.lock();
	_flagSetStateRequested=true;
	_srvSetState.request.ros_machineState=_ros_newState;
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++)
	{
		_srvSetState.request.ros_effortComp[j]=_ros_effortComp[j];
	}

	_clientSetState.call(_srvSetState);

	_flagResponseSetState = _srvSetState.response.platform_newState;
	_mutex.unlock();
}

void footVarSynchronizer::requestSetController(){
	_mutex.lock();
	_flagSetControllerRequested=true;
	_srvSetController.request.ros_controllerType=_ros_controllerType;
	_srvSetController.request.ros_defaultControl=_ros_defaultControl;
	_srvSetController.request.ros_controlledAxis=_ros_controlledAxis;

		for (int k=0; k<NB_AXIS; k++)
		{
			_srvSetController.request.ros_posP[k]=_ros_posP[k];
			_srvSetController.request.ros_posI[k]=_ros_posI[k];
			_srvSetController.request.ros_posD[k]=_ros_posD[k];

			_srvSetController.request.ros_speedP[k]=_ros_speedP[k];
			_srvSetController.request.ros_speedI[k]=_ros_speedI[k];
			_srvSetController.request.ros_speedD[k]=_ros_speedD[k];
		}

	_clientSetController.call(_srvSetController);
	
	_flagResponseSetController = _srvSetController.response.platform_controlOk;
	_mutex.unlock();
}


void footVarSynchronizer::dynamicReconfigureCallback(foot_variables_sync::machineStateParamsConfig &config, uint32_t level)
{
//   ROS_INFO("Reconfigure request. Updating the parameters of the machine state");
  	_flagWasDynReconfCalled=true;
	_ros_newState = (uint8_t)config.machine_state;

	_ros_controlledAxis = (int8_t) config.controlled_axis;
	_ros_controllerType = (uint8_t) config.controller_type;
	_ros_defaultControl = (bool) config.use_default_gains;

    _flagControlThisPosition = (bool) config.send_this_position;
	_flagCapturePlatformPosition = (bool)config.capture_platform_position;
	
	if (_flagControlThisPosition)
	{
		_ros_position << config.desired_Position_X, config.desired_Position_Y, config.desired_Position_PITCH, config.desired_Position_ROLL, config.desired_Position_YAW;
		ROS_DEBUG("Updating_POSITION");
	}

	_ros_effortComp[NORMAL]=(uint8_t) config.effortComp_normal;
	_ros_effortComp[CONSTRAINS]=(uint8_t) config.effortComp_constrains;
	_ros_effortComp[COMPENSATION]=(uint8_t) config.effortComp_compensation;
	_ros_effortComp[FEEDFORWARD]=(uint8_t) config.effortComp_feedforward;

	
		_ros_posP[X]=config.kp_Position_X; 
		_ros_posP[Y]=config.kp_Position_Y;
		_ros_posP[PITCH]=config.kp_Position_PITCH;
		_ros_posP[ROLL]=config.kp_Position_ROLL;
		_ros_posP[YAW]=config.kp_Position_YAW;

		_ros_posI[X]=config.ki_Position_X; 
		_ros_posI[Y]=config.ki_Position_Y;
		_ros_posI[PITCH]=config.ki_Position_PITCH;
		_ros_posI[ROLL]=config.ki_Position_ROLL;
		_ros_posI[YAW]=config.ki_Position_YAW;

		_ros_posD[X]=config.kd_Position_X; 
		_ros_posD[Y]=config.kd_Position_Y;
		_ros_posD[PITCH]=config.kd_Position_PITCH;
		_ros_posD[ROLL]=config.kd_Position_ROLL;
		_ros_posD[YAW]=config.kd_Position_YAW;
		
		_ros_speedP[X]=config.kp_Speed_X; 
		_ros_speedP[Y]=config.kp_Speed_Y;
		_ros_speedP[PITCH]=config.kp_Speed_PITCH;
		_ros_speedP[ROLL]=config.kp_Speed_ROLL;
		_ros_speedP[YAW]=config.kp_Speed_YAW;
		_ros_speedI[X]=config.ki_Speed_X; 
		_ros_speedI[Y]=config.ki_Speed_Y;
		_ros_speedI[PITCH]=config.ki_Speed_PITCH;
		_ros_speedI[ROLL]=config.ki_Speed_ROLL;
		_ros_speedI[YAW]=config.ki_Speed_YAW;
		_ros_speedD[X]=config.kd_Speed_X; 
		_ros_speedD[Y]=config.kd_Speed_Y;
		_ros_speedD[PITCH]=config.kd_Speed_PITCH;
		_ros_speedD[ROLL]=config.kd_Speed_ROLL;
		_ros_speedD[YAW]=config.kd_Speed_YAW;
		
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
		float scale = 0.0;
		if (axis_ < PITCH)
		{
			scale = SCALE_GAINS_LINEAR_POSITION;
		}
		else
		{
			scale = SCALE_GAINS_ANGULAR_POSITION;
		}
		switch (axis_)
		{
		case (X):
		{
			_config.kp_Position_X = GT_KP_POSITION_X / scale;
			_config.ki_Position_X = GT_KI_POSITION_X / scale;
			_config.kd_Position_X = GT_KD_POSITION_X / scale;
			_ros_posP[X] = _config.kp_Position_X;
			_ros_posI[X] = _config.ki_Position_X;
			_ros_posD[X] = _config.kd_Position_X;
			break;
		}
		case (Y):
		{
			_config.kp_Position_Y = GT_KP_POSITION_Y / scale;
			_config.ki_Position_Y = GT_KI_POSITION_Y / scale;
			_config.kd_Position_Y = GT_KD_POSITION_Y / scale;
			_ros_posP[Y] = _config.kp_Position_Y;
			_ros_posI[Y] = _config.ki_Position_Y;
			_ros_posD[Y] = _config.kd_Position_Y;
			break;
		}
		case (PITCH):
		{
			_config.kp_Position_PITCH = GT_KP_POSITION_PITCH / scale;
			_config.ki_Position_PITCH = GT_KI_POSITION_PITCH / scale;
			_config.kd_Position_PITCH = GT_KD_POSITION_PITCH / scale;
			_ros_posP[PITCH] = _config.kp_Position_PITCH;
			_ros_posI[PITCH] = _config.ki_Position_PITCH;
			_ros_posD[PITCH] = _config.kd_Position_PITCH;
			break;
		}
		case (ROLL):
		{
			_config.kp_Position_ROLL = GT_KP_POSITION_ROLL / scale;
			_config.ki_Position_ROLL = GT_KI_POSITION_ROLL / scale;
			_config.kd_Position_ROLL = GT_KD_POSITION_ROLL / scale;
			_ros_posP[ROLL] = _config.kp_Position_ROLL;
			_ros_posI[ROLL] = _config.ki_Position_ROLL;
			_ros_posD[ROLL] = _config.kd_Position_ROLL;
			break;
		}
		case (YAW):
		{
			_config.kp_Position_YAW = GT_KP_POSITION_YAW / scale;
			_config.ki_Position_YAW = GT_KI_POSITION_YAW / scale;
			_config.kd_Position_YAW = GT_KD_POSITION_YAW / scale;
			_ros_posP[YAW] = _config.kp_Position_YAW;
			_ros_posI[YAW] = _config.ki_Position_YAW;
			_ros_posD[YAW] = _config.kd_Position_YAW;
			break;
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