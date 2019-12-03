
#include "frictionLogger.h"
#include "ros/package.h"
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"
#include "geometry_msgs/WrenchStamped.h"


#define NB_RUNS 1
#define NB_DATA_POINTS 2
#define NB_DIVISIONS 5
#define NB_POSITIONS (NB_DIVISIONS + 1)
#define NB_REPETITIONS 3

// // #define NB_POSITIONS 3
// #define NB_POSITIONS 2

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

char const *Platform_Names[]{"none", "right", "left"};

float const SPEED_DEADZONE[NB_AXIS] = {0.001f, 0.001f, 0.05f, 0.09f,0.09f};
float const MIN_SPEED[NB_AXIS] = {0.00001f, 0.00001f, 0.00002f, 0.01f, 0.01f};
float const RESOLUTION_EFFORT[NB_AXIS] = {0.001f, 0.001f, 0.00005f, 0.00005f, 0.00005f};
float const FILTER_GAIN =  0.9f;
// float const des_positions_[NB_POSITIONS+1] = {-1.0f, 0.0f, 1.0f, 0.0f};
Eigen::Matrix<float, ((NB_POSITIONS)*NB_REPETITIONS+1),1> des_positions_;
Eigen::Matrix<float,NB_AXIS,1> tolerance_;

float const LIMITS[NB_AXIS] = {0.08, 0.08,17,17,20};

frictionLogger *frictionLogger::me = NULL;

frictionLogger::frictionLogger(ros::NodeHandle &n_1, double frequency,frictionLogger::Platform_Name platform_id, Axis axis_, std::string filename_): 
_n(n_1),
_platform_name(platform_id),
_frictionAxis(axis_),
_loopRate(frequency),
_dt(1.0f/frequency),
_rawFilename(filename_)
{
	me = this;
	_stop = false;
	_flagOutputMessageReceived=false;
	_flagPlatformOutCommStarted=false;
	_flagPositionOnlyPublished=false;
	_flagFrictionIDed = false;
	_flagLoggingOk = false;
	_flagNextPositionOk=false;

	_flagFirstTime= false;

	_flagReqSetControllerForPosition= false;
	_flagReqSetStateForPosition= false;

	_flagReqSetControllerForFrictionID= false;
	_flagReqSetStateForFrictionID= false;

	_frictionMotorsEffort.setConstant(0.0f);
	_desiredMotorsEffort.setConstant(0.0f);
	_desiredMotorsPosition.setConstant(0.0f);

	_motionSign = 1;

	_platform_position.setConstant(0.0f);
	_platform_position_last.setConstant(0.0f);
	_platform_speed.setConstant(0.0f);
	_platform_effortD.setConstant(0.0f);
	_platform_effortM.setConstant(0.0f);

	_platform_controllerType = TORQUE_ONLY;
	_platform_machineState = EMERGENCY;
	_platform_id = 0;

	// _decisionState = GOTOPOSITION;

	for (int i = 0; i < NB_REPETITIONS; i++)
	{
		for (int j = 0; j < NB_POSITIONS; j++)
		{
			des_positions_(i * NB_POSITIONS + j) = -1.0f + j * (2.0f / NB_DIVISIONS);
			des_positions_(i * NB_POSITIONS + j) = clamp(des_positions_(i * NB_POSITIONS + j), -1.0f, 1.0f);
		}
		 std::cerr << des_positions_.transpose() << std::endl;
	}

	

	des_positions_(NB_POSITIONS * NB_REPETITIONS + 1) = 0.0f;

	_nSignChanges = 0;
	_nDataPoints=0;
	_nPositions=0;

	tolerance_<< 0.003f, 0.003f, 3.0f, 0.5f, 0.5f;

	if (_frictionAxis == -1)
	{
		_nAxis=X;
	}
	else{
		_nAxis=_frictionAxis;
	}

}
frictionLogger::~frictionLogger()
{
	me->_n.shutdown();
}

bool frictionLogger::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

	if (_platform_name==LEFT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_LEFT, 1, boost::bind(&frictionLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_pubFriction = _n.advertise<custom_msgs::FootOutputMsg_v2>("FI_Data/Left/friction_data", 1);
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_clientSetState = _n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_LEFT);
		_clientSetController = _n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_LEFT);
	}
	if (_platform_name==RIGHT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&frictionLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_pubFriction = _n.advertise<custom_msgs::FootOutputMsg_v2>("FI_Data/Right/friction_data", 1);
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_RIGHT, 1);
		_clientSetState = _n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_RIGHT);
		_clientSetController = _n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_RIGHT);
	}

	//Subscriber definitions
	signal(SIGINT, frictionLogger::stopNode);

	if (_rawFilename != std::string("no_file"))
	{
		_flagLoggingOk = true;
	}
	if(_flagLoggingOk)
	{
		_filename=_rawFilename+ "_" + std::string(Axis_names[_nAxis]);
		_outputFile.open(ros::package::getPath(std::string("foot_variables_log")) + "/data/friction/" + _filename + ".txt");
	}


	
	// std::cerr << "next position : " << _desiredMotorsPosition(_nAxis) << endl;

	if (_n.ok()) 
	{   
		ros::spinOnce();
		ROS_INFO("The friction logging node of the foot interface is about to start ");
		//! Load the default gains for position control

		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void frictionLogger::stopNode(int sig)
{
    me->_stop= true;
}

void frictionLogger::logData()
{
	if (_filename != std::string("no_file"))
	{
		if (fabs(_frictionMotorsEffort(_nAxis))>=0.005f){
			_outputFile << ros::Time::now() << " "
						<< (int)_platform_id << " "
						<< _platform_position_last.transpose() << " "
						<< _platform_position.transpose() << " "
						<< _platform_speed.transpose() << " "
						<< _frictionMotorsEffort(_nAxis) << std::endl;
		}
	}
}

void frictionLogger::run()
{
  while (!_stop) 
  {

    if (_flagPlatformOutCommStarted) 
	
	{

		_mutex.lock();
		if ((_platform_id != (uint8_t)_platform_name) &&
			(_platform_id != UNKNOWN)) {
			ROS_ERROR(
				"This node for variables looging is acting on the wrong platform");
			break;
		}
		
		else {
			
			if (_nAxis <= YAW) //! Only works for the 5DOF 
			{
				if (_platform_id != 0) //! Makes sure that the platform already identified itself
				{			
					if (_flagNextPositionOk)
					{
						if (_platform_machineState == TELEOPERATION && _platform_controllerType==TORQUE_ONLY)
						{
							// std::cerr<<"here"<<std::endl;
							computeStaticFriction();
							publishFootEffort();
						}
						else
						{
							if (!_flagResponseSetState && !_flagReqSetStateForFrictionID)
							{
								requestSetState(TELEOPERATION, &_flagReqSetStateForFrictionID);
								if (_flagResponseSetState)
								{
									_flagResponseSetState = false;
								}
							}
														
							if (_platform_machineState == TELEOPERATION)
							{
								if (!_flagResponseSetController && !_flagReqSetControllerForFrictionID)
								{
									requestSetController(TORQUE_ONLY, &_flagReqSetControllerForFrictionID);
									if (_flagResponseSetController)
									{
										_flagResponseSetController=false;
									}
								}

							}
						}
					}
					else
					{
						if (_platform_machineState == ROBOT_STATE_CONTROL && _platform_controllerType == POSITION_ONLY && _flagFirstTime)
						{					
							
							if ((_desiredMotorsPosition - _platform_position).norm() <= tolerance_.norm())
							{
								
								usleep(1.5e6);
								_flagNextPositionOk=true;
								// _flagFirstTimeFriction = false;
								_flagReqSetStateForFrictionID = false;
								_flagReqSetControllerForFrictionID = false;
								_nPositions++;
								if (_nPositions >= NB_POSITIONS)
								{
									_nPositions = 0;
									_nAxis++;
									if (_nAxis <= YAW)
									{
										if (_flagLoggingOk)
										{
											_outputFile.close();
											_filename = _rawFilename + "_" + std::string(Axis_names[_nAxis]);
											_outputFile.open(ros::package::getPath(std::string("foot_variables_log")) + "/data/friction/" + _filename + ".txt");
										}
										std::cerr << "Next Axis: " << Axis_names[_nAxis] << std::endl;
									}
									else
									{
										break;
									}
									
								}

								_desiredMotorsEffort.setConstant(0.0f);
								_desiredMotorsPosition.setConstant(0.0f);
								_desiredMotorsPosition(_nAxis) = LIMITS[_nAxis] * des_positions_[_nPositions];
							}
						}
						else
						{
							if (!_flagResponseSetState && !_flagReqSetStateForPosition)
								requestSetState(ROBOT_STATE_CONTROL, &_flagReqSetStateForPosition);
							// ros::spinOnce();
							if (_flagResponseSetState)
							{
								_flagResponseSetState = false;
							}

							if (_platform_machineState == ROBOT_STATE_CONTROL)
							{
								if (!_flagResponseSetController && !_flagReqSetControllerForPosition)
								{
									if (!_flagFirstTime)
									{

										for (int k = 0; k < NB_AXIS; k++)
										{
											if (k == _nAxis)
											{
												_desiredMotorsPosition(_nAxis) = LIMITS[_nAxis] * des_positions_[0];
											}
											else
											{
												_desiredMotorsPosition(k) = LIMITS[k] * des_positions_[int(rand() % NB_POSITIONS)];
											}
										}
									}
									_flagFirstTime = true;
									publishDesiredPosition();
									std::cerr << "desired position : " << _desiredMotorsPosition(_nAxis) << endl;
									requestSetController(POSITION_ONLY, &_flagReqSetControllerForPosition);
									if (_flagResponseSetController)
									{
										usleep(1e-6);
										_flagResponseSetController = false;
									}
									// ros::spinOnce();
									// _loopRate.sleep();
								}
							}	
						}
					}
				
				}
			
			}
			else
				{
					frictionLogger::stopNode;
				}

		}
		_mutex.unlock();
	}
		ros::spinOnce();
		_loopRate.sleep();
  }

  ROS_INFO("Friction Logging stoped");
  
  if (_filename != std::string("no_file") && _nAxis<= YAW)
  {
	  logData();
	  _outputFile.close();
  }
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}



void frictionLogger::publishIDFriction()
{
	if (_frictionMotorsEffort(_nAxis)>=0.001f)
	{
		_msgStaticFriction.platform_stamp=ros::Time::now();
		_msgStaticFriction.platform_id = (int)_platform_id;
		for (int k = 0; k<NB_AXIS; k++)
		{
			_msgStaticFriction.platform_effortM[k]=_frictionMotorsEffort[k];
			_msgStaticFriction.platform_position[k] = _platform_position_last[k];
			_msgStaticFriction.platform_speed[k] = _platform_speed[k];	
		}	
		_pubFriction.publish(_msgStaticFriction);
	}

}

void frictionLogger::publishFootEffort()
{

	if (_platform_machineState == TELEOPERATION)
	{

		for (int k = 0; k < NB_AXIS; k++)
		{			
			_msgFootInput.ros_effort[k] = _desiredMotorsEffort(k);
		}
	}
	_pubFootInput.publish(_msgFootInput);
}

void frictionLogger::publishDesiredPosition()
{

	if (_platform_machineState == ROBOT_STATE_CONTROL)
	{

		for (int k = 0; k < NB_AXIS; k++)
		{
			_msgFootInput.ros_position[k] = _desiredMotorsPosition(k);
		}
	}
	_pubFootInput.publish(_msgFootInput);
}

void frictionLogger::fetchFootOutput(
    const custom_msgs::FootOutputMsg_v2::ConstPtr &msg) {
  _flagOutputMessageReceived = true;
  _platform_id = msg->platform_id;
  for (int k = 0; k < NB_AXIS; k++) {
    _platform_position[k] = msg->platform_position[k];
    _platform_speed[k] = msg->platform_speed[k];
    _platform_effortD[k] = msg->platform_effortD[k];
    _platform_effortM[k] = msg->platform_effortM[k];
  }
  _platform_machineState = (frictionLogger::State)msg->platform_machineState;
  _platform_controllerType =
      (frictionLogger::Controller)msg->platform_controllerType;
  if (!_flagPlatformOutCommStarted) {
    _flagPlatformOutCommStarted = true;
  }
}

void frictionLogger::computeStaticFriction()
{
	if (fabs(_platform_speed[_nAxis]) >= SPEED_DEADZONE[_nAxis])
	{
		_flagFrictionIDed = true;
	}

	if (!_flagFrictionIDed)
	{
		if (_desiredMotorsEffort(_nAxis) >= 0.01f)
		{
			_platform_position_last = _platform_position;
		}

		if (fabs(_platform_speed[_nAxis]) <=MIN_SPEED[_nAxis])
		{
			_desiredMotorsEffort(_nAxis)+=_motionSign*RESOLUTION_EFFORT[_nAxis];
		}
	}
	else
	{
		if (fabs(_desiredMotorsEffort(_nAxis)) >= 0.01f)
		{
			_frictionMotorsEffort(_nAxis) = _desiredMotorsEffort(_nAxis);
			std::cerr <<"latest identified friction: "<< _frictionMotorsEffort(_nAxis) << endl;
			_nDataPoints++;
			std::cerr << "nDataPoints: " << _nDataPoints << std::endl;
			if (_nDataPoints >= NB_DATA_POINTS)
			{
				_motionSign *= -1;
				_nDataPoints = 0;
				_nSignChanges++;
				std::cerr << "nSignChanges: " << _nSignChanges << std::endl;
				if (_nSignChanges >= 2 * NB_RUNS)
				{
					for (int k = 0; k < NB_AXIS; k++)
					{
						if (k == _nAxis)
						{
							_desiredMotorsPosition(_nAxis) = LIMITS[_nAxis] * des_positions_[_nPositions];
						}
						else
						{
							_desiredMotorsPosition(k) = LIMITS[k] * des_positions_[int(rand() % NB_POSITIONS)];
						}
					}
					_flagNextPositionOk = false;
					_flagReqSetControllerForPosition = false;
					_flagReqSetStateForPosition = false;
					_nSignChanges = 0;
				}
			}
		
		logData();
		_desiredMotorsEffort(_nAxis) = 0.0f;
		publishIDFriction();
	
		}
		
		_flagFrictionIDed=false;
		
	}
}

void frictionLogger::requestSetState(State state_, bool* StateRequested_)
{
	// _mutex.lock();
	*StateRequested_ = true;
	_flagResponseSetState= false;
	_srvSetState.request.ros_machineState = state_;

	float components[NB_EFFORT_COMPONENTS] = {1, 0, 0, 0};
	
	for (int j = 0; j < NB_EFFORT_COMPONENTS; j++)
	{
		_srvSetState.request.ros_effortComp[j] = components[j];
	}

	_clientSetState.call(_srvSetState);

	_flagResponseSetState = _srvSetState.response.platform_newState;
	// _mutex.unlock();
}

void frictionLogger::requestSetController(Controller controller_, bool* controllerRequested_)
{
	// _mutex.lock();
	*controllerRequested_ = true;
	_flagResponseSetController = false;

	_srvSetController.request.ros_controllerType = controller_;
	_srvSetController.request.ros_defaultControl = true;
	_srvSetController.request.ros_controlledAxis = -1;

	_clientSetController.call(_srvSetController);

	_flagResponseSetController = _srvSetController.response.platform_controlOk;
	// _mutex.unlock();
}

float frictionLogger::clamp(float x, float out_min, float out_max)
{
	return x < out_min ? out_min : (x > out_max ? out_max : x);
}
