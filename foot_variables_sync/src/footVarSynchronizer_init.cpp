#include "footVarSynchronizer.h" 

bool footVarSynchronizer::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{	

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
	_flagInertiaCoriolisRead=false;
	_flagLegCompWrenchRead = false;

	_flagTwoFeetOneToolRead = false;

	_myToolControl = TOOL_POSITION_CTRL;

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
		_ros_filterAxisFS[k]=0.8f;

		_ros_posP[k]=0.0f;
		_ros_posI[k]=0.0f;
		_ros_posD[k]=0.0f;
		_ros_speedP[k]=0.0f;
		_ros_speedI[k]=0.0f;
		_ros_speedD[k]=0.0f;

		for (size_t c = 0; c < NB_POS_PID_C; c++)
		{
			
			_ros_paramP_A[c][k]=0.0f;
			_ros_paramP_B[c][k]=0.0f;
			_ros_paramI[c][k]=0.0f;
			_ros_paramD[c][k]=0.0f;
		}
		

	}
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++){
		_ros_effortComp[j];
	}
	_ros_effortM.setZero();
		
    _ros_forceSensor_controlled.setZero();
	_ros_forceModified.setZero();
	
    _ros_newState = (uint8_t)_platform_machineState;
    _ros_controllerType = (uint8_t)_platform_controllerType;
    _ros_defaultControl = true;

    _ros_controlledAxis = (int8_t)-1;

    _leg_grav_comp_effort.setZero();
	_inertial_coriolis_comp_effort.setZero();
    _legWrenchGravityComp.setZero();
	

	_mixedPlatformOn=false;
    
	if (!_n.getParam("/useOneFootForTwoTools", _mixedPlatformOn))
    { 
      ROS_INFO(" [%s footVarSync]: No /useOneFootForTwoTools  param, default to false",Platform_Names[_platform_name]); 
	  _mixedPlatformOn=false;
    } 

	_controlTools=false;
    if (!_n.getParam("controlTools", _controlTools))
    { 
      ROS_ERROR(" [%s footVarSync]: No controlTools  param",Platform_Names[_platform_name]); 
    } 

	getPIDParams();

	
	std::vector<std::string> _fiPublishers;			
	
	if (!_n.getParam("/"+std::string(Platform_Names[_platform_name])+"/fi_input/topics", _fiPublishers))
	{ 
		ROS_ERROR("[%s footVarSync]: Missing fi_input/topics param",Platform_Names[_platform_name]); 
	}

	_nbDesiredFootInputPublishers = _fiPublishers.size();
	std::cout<<_nbDesiredFootInputPublishers<<std::endl;
	
	if (_nbDesiredFootInputPublishers>0)
	{
		_subDesiredFootInput.resize(_nbDesiredFootInputPublishers);
		_msgDesiredFootInput.resize(_nbDesiredFootInputPublishers);
		_flagDesiredFootInputsRead.resize(_nbDesiredFootInputPublishers);
		ROS_INFO("[%s footVarSync]: The list of publishers for the platform input are: ",Platform_Names[_platform_name]);
		for (size_t i  = 0; i<_nbDesiredFootInputPublishers; i++)
		{	
			_flagDesiredFootInputsRead[i]=false;
			ROS_INFO("[%s footVarSync]: Topic %i: %s",Platform_Names[_platform_name], (int) i,_fiPublishers[i].c_str());
			_subDesiredFootInput[i] = _n.subscribe<custom_msgs::FootInputMsg>("/"+std::string(Platform_Names[_platform_name])+"/"+_fiPublishers[i], 1, boost::bind(&footVarSynchronizer::readDesiredFootInputs, this, _1,i), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		}
	}
	


	if (_platform_name==LEFT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg>(PLATFORM_PUBLISHER_NAME_LEFT, 1, boost::bind(&footVarSynchronizer::readFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		 
		 
		_subForceModified = _n.subscribe<geometry_msgs::WrenchStamped>("/left_platform/force_sensor_modifier/force_modified", 1,boost::bind(&footVarSynchronizer::readForceModified, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subLegGravCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&footVarSynchronizer::readLegGravCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());		
	
		_subInertiaCoriolisCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/foot_comp_inertia_coriolis", 1,boost::bind(&footVarSynchronizer::readInertiaCoriolisCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());		
		
		_subTorquesModified = _n.subscribe<custom_msgs::FootOutputMsg>("/left_platform/force_sensor_modifier/torques_modified", 1,boost::bind(&footVarSynchronizer::readTorquesModified, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());		

    	_subLegGravCompWrench = _n.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench", 1,boost::bind(&footVarSynchronizer::readLegGravityCompWrench, this, _1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_clientSetState=_n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_LEFT);
		_clientSetController=_n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_LEFT);
	}
	if (_platform_name==RIGHT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg>(PLATFORM_SUBSCRIBER_NAME_RIGHT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&footVarSynchronizer::readFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		
		_subForceModified = _n.subscribe<geometry_msgs::WrenchStamped>("/right_platform/force_sensor_modifier/force_modified", 1,boost::bind(&footVarSynchronizer::readForceModified, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subLegGravCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&footVarSynchronizer::readLegGravCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subInertiaCoriolisCompTorques = _n.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/foot_comp_inertia_coriolis", 1,boost::bind(&footVarSynchronizer::readInertiaCoriolisCompFI, this, _1),
					    	ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

		_subTorquesModified = _n.subscribe<custom_msgs::FootOutputMsg>("/right_platform/force_sensor_modifier/torques_modified", 1,boost::bind(&footVarSynchronizer::readTorquesModified, this, _1),
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
