
#include "frictionLogger.h"
#include "ros/package.h"
#include "geometry_msgs/WrenchStamped.h"


#define NB_RUNS 5
#define NB_DATA_POINTS 1
#define NB_DIVISIONS 30
#define NB_POSITIONS (NB_DIVISIONS + 1)
#define NB_REPETITIONS 1

int total_points; 
int nPoint_;
// // #define NB_POSITIONS 3
// #define NB_POSITIONS 2

#define ListofAxes(enumeration, names) names,
						   char const *
						   Axis_names[]{
							   AXES};
#undef ListofAxes

char const *Platform_Names[]{"none", "right", "left"};

float const SPEED_DEADZONE[NB_AXIS] = {0.001f, 0.001f, 0.05f, 0.09f,0.09f};
float const MIN_SPEED[NB_AXIS] = {0.00002f, 0.00002f, 0.00002f, 0.02f, 0.02f};
float const RESOLUTION_EFFORT[NB_AXIS] = {0.001f, 0.001f, 0.00005f, 0.00005f, 0.00005f};
float const FILTER_GAIN =  0.9f;
// float const des_positions_[NB_POSITIONS+1] = {-1.0f, 0.0f, 1.0f, 0.0f};
Eigen::Matrix<float, ((NB_POSITIONS)*NB_REPETITIONS+1),1> des_positions_;
Eigen::Matrix<float,NB_AXIS,1> tolerance_;

float const LIMITS[NB_AXIS] = {0.08, 0.08,17,17,20};

ros::Time beginning;

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
	_decisionState = COMM_BEGIN;
	_lastDecisionState = _decisionState;

	for (int i = 0; i < NB_REPETITIONS; i++)
	{
		for (int j = 0; j < NB_POSITIONS; j++)
		{
			des_positions_(i * NB_POSITIONS + j) = -1.0f + j * (2.0f / NB_DIVISIONS);
			des_positions_(i * NB_POSITIONS + j) = clamp(des_positions_(i * NB_POSITIONS + j), -1.0f, 1.0f);
		}
		//  std::cerr << des_positions_.transpose() << std::endl;
	}

	

	des_positions_(NB_POSITIONS * NB_REPETITIONS + 1) = 0.0f;

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

    total_points = NB_RUNS * NB_DATA_POINTS * NB_POSITIONS * NB_REPETITIONS * (NB_AXIS+1-_nAxis);
}
frictionLogger::~frictionLogger()
{
	me->_n.shutdown();
}

bool frictionLogger::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

	if (_platform_name==LEFT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v3>(PLATFORM_PUBLISHER_NAME_LEFT, 1, boost::bind(&frictionLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_pubFriction = _n.advertise<custom_msgs::FootOutputMsg_v3>("FI_Data/Left/friction_data", 1);
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_clientSetState = _n.serviceClient<custom_msgs::setStateSrv>(SERVICE_CHANGE_STATE_NAME_LEFT);
		_clientSetController = _n.serviceClient<custom_msgs::setControllerSrv>(SERVICE_CHANGE_CTRL_NAME_LEFT);
	}
	if (_platform_name==RIGHT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v3>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&frictionLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_pubFriction = _n.advertise<custom_msgs::FootOutputMsg_v3>("FI_Data/Right/friction_data", 1);
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
          _filename = _rawFilename + "_" +
                      std::string(Platform_Names[_platform_name]) + "_" +
                      std::string(Axis_names[_nAxis]);
          _outputFile.open(
              ros::package::getPath(std::string("foot_variables_log")) +
              "/data/friction/" + _filename + ".txt");
	}

	nPoint_=0;
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
	ros::Duration relativeTime = ros::Time::now() - beginning;
	if (_filename != std::string("no_file"))
	{
		if (fabs(_frictionMotorsEffort(_nAxis))>=0.005f){
			_outputFile << relativeTime.toSec() << '\t'
						<< (int)_platform_id << '\t'
						<< _platform_position_last.transpose() << '\t'
						<< _platform_position.transpose() << '\t'
						<< _platform_speed.transpose() << '\t'
						<< _frictionMotorsEffort(_nAxis) <<std::endl;
		}
	}
}

void frictionLogger::run()
{
	while (!_stop) 
	{	

		switch (_decisionState)
		{
			case COMM_BEGIN:
			{
				_lastDecisionState = _decisionState;

				if (_flagPlatformOutCommStarted)
				{
					_decisionState = PLATFORM_IDENTIFY;
					ROS_INFO("Communication began");
				}
				break;
			}
			case PLATFORM_IDENTIFY:
			{
				_lastDecisionState = _decisionState;

				if ((_platform_id != (uint8_t)_platform_name) &&
					(_platform_id != UNKNOWN))
				{
					ROS_ERROR(
						"This node for variables looging is acting on the wrong platform");
					break;
				}
				else
				{
					ROS_INFO("Platform Identified");
					beginning = ros::Time::now();
					_decisionState=REQ_ROBOT_STATE;
				}
				break;
			}
			case REQ_ROBOT_STATE:
			{
				_lastDecisionState = _decisionState;

				// entering state

				if (_platform_machineState==ROBOT_STATE_CONTROL)
				{
					_flagResponseSetState=true;
				}
				else
				{
					if (!_flagReqSetStateForPosition)
					{
						requestSetState(ROBOT_STATE_CONTROL, &_flagReqSetStateForPosition);
						ros::spinOnce();
						_loopRate.sleep();
						_flagReqSetStateForPosition=false;
					}
				}
					
				if (_flagResponseSetState)
				{
					ROS_INFO("Robot State Control Requested");
					_decisionState = REQ_POS_CTRL;
					_flagResponseSetState = false;
				}
				else
				{
					ROS_ERROR("Couldn't move to robot state control, trying again...");
					usleep(5e6f); // wait five seconds and try again
				}

				break;
			}
			case REQ_POS_CTRL:
			{

				_lastDecisionState = _decisionState;
				
				publishDesiredPosition();
				ros::spinOnce();
				_loopRate.sleep();

				if (_platform_controllerType==POSITION_ONLY)
				{
					_flagResponseSetController=true;
				}
				else
				{
					if (!_flagReqSetControllerForPosition)
					{
						// std::cerr << "desired position : " << _desiredMotorsPosition(_nAxis) << endl;
						requestSetController(POSITION_ONLY, &_flagReqSetControllerForPosition);
						ros::spinOnce();
						// _loopRate.sleep();
						_flagReqSetControllerForPosition=false;
					}
				}
				//Conditional to leave state

				if (_flagResponseSetController)
				{
					_decisionState = POS_TOLERANCE;
					_flagResponseSetController = false;
				}
				else
				{
					ROS_ERROR("Couldn't request to the platform to control a position, trying again...");
					usleep(5e6f); // wait five seconds and try again
				}
				
				break;
			}

			case POS_TOLERANCE:
			{
				// Main
				_lastDecisionState = _decisionState;

				// Leave State
				//Conditional to leave state
				if ((_desiredMotorsPosition - _platform_position).norm() <= tolerance_.norm())
				{
					usleep(5e6f); // delay for stability;
					ROS_INFO("New point in %s reached", Axis_names[_nAxis]);
					_decisionState = REQ_TELEOP_CTRL;
				}
				else
				{
					// wait
				}
				break;
			}

			case REQ_TELEOP_CTRL :
			{
				// Main
				_lastDecisionState = _decisionState;
				if (_platform_machineState==TELEOPERATION)
				{
					_flagResponseSetState=true;
				}
				else
				{
					if (!_flagReqSetStateForFrictionID)
					{
						requestSetState(TELEOPERATION, &_flagReqSetStateForFrictionID);
						ros::spinOnce();
						// _loopRate.sleep();
						_flagReqSetStateForFrictionID = false;
					}
				}
					// Leave the state

				if (_flagResponseSetState)
				{
					ROS_INFO("Teleoperation State Control Requested");
					_decisionState = REQ_TORQUE_CTRL;
					_flagResponseSetState = false;
				}
				else
				{
					ROS_ERROR("Couldn't start teleoperation, trying again...");
					usleep(5e6f); // wait five seconds and try again
				}

				break;

			}
			
			case REQ_TORQUE_CTRL:
			{
				
				_desiredMotorsEffort.setConstant(0.0f);
				_lastDecisionState = _decisionState;
				publishFootEffort(); //! Publish zero torque
				ros::spinOnce();
				// _loopRate.sleep();

				//Main
				if (_platform_controllerType==TORQUE_ONLY)
				{
					_flagResponseSetController =true;
				}
				else
				{
					if (!_flagReqSetControllerForFrictionID)
					{
						requestSetController(TORQUE_ONLY, &_flagReqSetControllerForFrictionID);
						ros::spinOnce();
						// _loopRate.sleep();
						_flagReqSetControllerForFrictionID = false;
					}
				}	

				if (_flagResponseSetController)
					{
						ROS_INFO("Torque Control Requested");
						usleep(3e6f);
						_decisionState = COMPUTE_FRICTION;
						_flagResponseSetController = false;
					}

				else
					{
						ROS_ERROR("Couldn't could control for torque, trying again...");
						usleep(5e6); // wait five seconds and try again
					}

					//Leave State
					//Conditionals to leave state
					break;
			}
			
			case COMPUTE_FRICTION:
			{
				//Main
				
				if (_lastDecisionState!=COMPUTE_FRICTION)
				{
					ROS_INFO("Computing the friction"); 
					_platform_position_last = _platform_position;
					_desiredMotorsEffort.setConstant(0.0f);
				}

				_lastDecisionState = _decisionState;

				//Conditionals to leave state

				if (fabs(_platform_speed[_nAxis]) <= MIN_SPEED[_nAxis])
				{
					_desiredMotorsEffort(_nAxis) += _motionSign * RESOLUTION_EFFORT[_nAxis];
					if (fabs(_desiredMotorsEffort(_nAxis)>=USER_MAX_EFFORTS[_nAxis]))
					{
						_decisionState=REQ_ROBOT_STATE;
						ROS_ERROR("Limit of the motor %s reached, setting position again",
						Axis_names[_nAxis]);
					}
                                        //Main
				}

				else if (fabs(_platform_speed[_nAxis]) >= SPEED_DEADZONE[_nAxis])
				{
					ROS_INFO("Speed threshold reached... checking the effort");
					_decisionState = ID_FRICTION;
				}

				else
				{
					// Do nothing	
				}

				//Send the foot effort
				publishFootEffort();
				break;
			}

			case ID_FRICTION:
			
			{
				_lastDecisionState = _decisionState;

				//Conditionals to leave state
				if (fabs(_desiredMotorsEffort(_nAxis)) >= 0.01f)
				{
					_frictionMotorsEffort(_nAxis) = _desiredMotorsEffort(_nAxis);
					// std::cerr << "latest identified friction: " << _frictionMotorsEffort(_nAxis) << endl;
					ROS_INFO("Identified Friction : %f [N | N/m]", _frictionMotorsEffort(_nAxis));
					logData();
					publishIDFriction();
					_decisionState = NEW_DATA_POINT;		
				}
				
				else
				{
					//ROS_INFO("It's weird, this effort is too low, trying again...");
					usleep(3e6f);
					ros::spinOnce();
					_loopRate.sleep();
					_decisionState = COMPUTE_FRICTION;
				}
				
				
				break;

			}

			//ITERATIONS

			case NEW_DATA_POINT:

			{
				_lastDecisionState = _decisionState;
				//Main

				_nDataPoints++;
				nPoint_++;

				ROS_INFO("Taking a new data point at %f in axis %s", _desiredMotorsPosition(_nAxis),
				Axis_names[_nAxis]);
				// std::cerr << "nDataPoints: " << _nDataPoints << std::endl;

				//Conditionals to leave state
				if (_nDataPoints >= NB_DATA_POINTS)
				{
					ROS_INFO("Limit of data points reached. Changing direction...");
					_nDataPoints = 0;
					_decisionState = CHANGE_SIGN;
				}

				else
				{
					usleep(3e6f);
					ros::spinOnce();
					_loopRate.sleep();
					_decisionState = COMPUTE_FRICTION;
				}

				_lastDecisionState = _decisionState;
				break;
			}

			case CHANGE_SIGN:
			{
				//Main
				
				_lastDecisionState = _decisionState;
				_motionSign *= -1;
				_nSignChanges++;
				ROS_INFO("Direction change  # %i to %i on point %f on axis %s", _nSignChanges,
						 _motionSign, _desiredMotorsPosition(_nAxis),
						 Axis_names[_nAxis]);
				float progress= ((float)nPoint_/(float)total_points)*100.0f;
				ROS_INFO("OVERALL PROGRESS: %f PERCENT", progress); 
				// std::cerr << "nSignChanges: " << _nSignChanges << std::endl;

				//Conditionals to leave state
				if (_nSignChanges >= 2 * NB_RUNS)
				{
					ROS_INFO("Limit of run reached. Changing position...");
					_decisionState = CHANGE_POSITION;
					_nSignChanges = 0;
				}

				else
				{
					usleep(3e6f);
					ros::spinOnce();
					_loopRate.sleep();
					_decisionState = COMPUTE_FRICTION;
				}

				break;
			}

			case CHANGE_POSITION:
			{
				_lastDecisionState = _decisionState;
				_nPositions++;

				// Leave State

				// Conditional to leave state

				if (_nPositions >= (NB_POSITIONS*NB_REPETITIONS + 1))
				{
					ROS_INFO("Limit of positions reached...");
					_nPositions = -1;
					_decisionState = CHANGE_AXIS;
				}
				else
				{
					ROS_INFO("Generating new configurations for other joints...");
					generateNewRefPoints(); // Generate new reference points
											// Leave State
					ROS_INFO("New position for %s: %f [m | deg]", Axis_names[_nAxis], _desiredMotorsPosition(_nAxis));
					_decisionState = REQ_ROBOT_STATE;
				}

				break;
			}

			case CHANGE_AXIS:
			{
				//Main
				_lastDecisionState = _decisionState;

				_nAxis++;

				// Conditionals to remain in this state
				if (_nAxis > YAW) //! Only works for the 5DOF
				{
					ROS_INFO("We have identified all joints...");
					_decisionState = STOP_ALL;
				}

				else
				{
					if (_flagLoggingOk)
					{
						ROS_INFO("Writing data in log file...");
						_outputFile.close();
						_filename = _rawFilename + "_"+std::string(Platform_Names[_platform_id])+"_" + std::string(Axis_names[_nAxis]);
						_outputFile.open(ros::package::getPath(std::string("foot_variables_log")) + "/data/friction/" + _filename + ".txt");					}
					// ROS_INFO("Changing to axis %s", Axis_names[_nAxis]);
					 std::cerr << "Changing to Axis: " << Axis_names[_nAxis] << std::endl;
					_decisionState = CHANGE_POSITION;
				}

				break;
			}

			case STOP_ALL:
			{
				_lastDecisionState = _decisionState;

				_stop=true;

			}
		}
		
		// _mutex.unlock();
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



void frictionLogger::generateNewRefPoints()
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
}

void frictionLogger::publishIDFriction()
{
	if (_frictionMotorsEffort(_nAxis) >= 0.01f)
	{
		_msgStaticFriction.platform_stamp = ros::Time::now();
		_msgStaticFriction.platform_id = (int)_platform_id;
		for (int k = 0; k < NB_AXIS; k++)
		{
			_msgStaticFriction.platform_effortM[rosAxis[k]] = _frictionMotorsEffort[k];
			_msgStaticFriction.platform_position[rosAxis[k]] = _platform_position_last[k];
			_msgStaticFriction.platform_speed[rosAxis[k]] = _platform_speed[k];
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
			_msgFootInput.ros_effort[rosAxis[k]] = clamp(_desiredMotorsEffort(k), -USER_MAX_EFFORTS[k], USER_MAX_EFFORTS[k]);
		}
	}
	_pubFootInput.publish(_msgFootInput);
}

void frictionLogger::publishDesiredPosition()
{
	for (int k = 0; k < NB_AXIS; k++)
	{
		_msgFootInput.ros_position[rosAxis[k]] = _desiredMotorsPosition(k);
	}
	_pubFootInput.publish(_msgFootInput);
}


void frictionLogger::fetchFootOutput(
	const custom_msgs::FootOutputMsg_v3::ConstPtr &msg)
{
	_flagOutputMessageReceived = true;
	_platform_id = msg->platform_id;
	for (int k = 0; k < NB_AXIS; k++)
	{
		_platform_position[k] = msg->platform_position[rosAxis[k]];
		_platform_speed[k] = msg->platform_speed[rosAxis[k]];
		_platform_effortD[k] = msg->platform_effortD[rosAxis[k]];
		_platform_effortM[k] = msg->platform_effortM[rosAxis[k]];
	}
	_platform_machineState = (frictionLogger::State)msg->platform_machineState;
	_platform_controllerType =
		(frictionLogger::Controller)msg->platform_controllerType;
	if (!_flagPlatformOutCommStarted)
	{
		_flagPlatformOutCommStarted = true;
	}
}



void frictionLogger::requestSetState(State state_, bool *StateRequested_)
{
	// _mutex.lock();
	*StateRequested_ = true;
	_flagResponseSetState = false;
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

void frictionLogger::requestSetController(Controller controller_, bool *controllerRequested_)
{
	// _mutex.lock();
	*controllerRequested_ = true;
	_flagResponseSetController = false;

	_srvSetController.request.ros_controllerType = controller_;
	_srvSetController.request.ros_defaultControl = false;
	_srvSetController.request.ros_controlledAxis = -1;
	_srvSetController.request.ros_posP[X] = 5000;
    _srvSetController.request.ros_posP[Y] = 5000;
    _srvSetController.request.ros_posP[PITCH] = 10000;
    _srvSetController.request.ros_posP[ROLL] = 5000;
    _srvSetController.request.ros_posP[YAW] = 5000;
    _srvSetController.request.ros_posI[X] = 10000;
    _srvSetController.request.ros_posI[Y] = 10000;
    _srvSetController.request.ros_posI[PITCH] = 10000;
    _srvSetController.request.ros_posI[ROLL] = 10000;
    _srvSetController.request.ros_posI[YAW] = 10000;
    _srvSetController.request.ros_posD[X] = 10;
    _srvSetController.request.ros_posD[Y] = 10;
    _srvSetController.request.ros_posD[PITCH] = 30;
    _srvSetController.request.ros_posD[ROLL] = 35;
    _srvSetController.request.ros_posD[YAW] = 35;

    _clientSetController.call(_srvSetController);

    _flagResponseSetController = _srvSetController.response.platform_controlOk;
    // _mutex.unlock();
}

float frictionLogger::clamp(float x, float out_min, float out_max)
{
	return x < out_min ? out_min : (x > out_max ? out_max : x);
}
