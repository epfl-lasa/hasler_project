#include "multiAxisFrictionID.h"
#include "../../5_axis_platform/lib/platform/src/definitions.h"
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

#define FORWARD 1
#define REVERSE -1

multiAxisFrictionID* multiAxisFrictionID::me = NULL;

multiAxisFrictionID::multiAxisFrictionID(ros::NodeHandle &n_1, double frequency,multiAxisFrictionID::Platform_Name platform_id): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_dt(1.0f/frequency)
{
	
	 me=this;
	_stop = false;
	_flagOutputMessageReceived=false;
	_flagPlatformInCommStarted=false;
	_flagPlatformOutCommStarted = false;
	_flagPositionOnlyPublished = false;
	_flagInitialConfig=false;
	
	

	for (int k = 0; k < NB_AXIS; k++)
	{
		//Platform

		_platform_position[k] = 0.0f;
		_platform_speed[k] = 0.0f;
		_platform_effortD[k] = 0.0f;
		_platform_effortM[k] = 0.0f;
	}

	_platform_controllerType = TORQUE_ONLY;
	_platform_machineState = EMERGENCY;
	_platform_id = 0;

	//ROS
	for (int k = 0; k < NB_AXIS; k++)
	{
		_ros_position[k]=0.0f;
		_ros_speed[k]=0.0f;
		_ros_effort[k]=0.0f;
						
	}

	//Staircase

	_limitWS << X_RANGE / 2.0 , Y_RANGE / 2.0, PITCH_RANGE / 2.0f , ROLL_RANGE / 2.0f , YAW_RANGE / 2.0f;

	_currentTime =ros::Time::now();
	
	for(int k = 0 ; k<NB_AXIS; k++)
	{
		_prevTime[k] = ros::Time::now();
		_tau[k] = 330 * _dt; //! If freq 700 hz : Tau ~ 3.6s If freq 90 hz : Tau ~ 3.67 s
		_nSteps[k] = 10;
		_currentStep[k] = 0;
		_flagStairCaseFinished[k] = false;
		_flagStairCaseStarted[k] = false;
		_flagNextStep[k][0] = false;
		_flagNextStep[k][1] = false;
		_flagIntegratorsZeroed[k]=true;
	}


}
multiAxisFrictionID::~multiAxisFrictionID()
{
	me->_n.shutdown();
}

bool multiAxisFrictionID::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{
	if (_platform_name==LEFT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_LEFT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_LEFT,1, boost::bind(&multiAxisFrictionID::fetchFootOutput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	}
	if (_platform_name==RIGHT){
		_pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v2>(PLATFORM_SUBSCRIBER_NAME_RIGHT, 1);
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_RIGHT,1, boost::bind(&multiAxisFrictionID::fetchFootOutput,this,_1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	}
  	

	//Subscriber definitions	
	signal(SIGINT,multiAxisFrictionID::stopNode);
	
	if (_n.ok()) 
	{   
		ros::spinOnce();
		ROS_INFO("The staircase generator for friction identification is about to start");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void multiAxisFrictionID::stopNode(int sig)
{
    me->_stop= true;
}

void multiAxisFrictionID::run()
{
  while (!_stop) 
  {	
	_currentTime=ros::Time::now();
	if (_flagPlatformOutCommStarted)
	{
		if (!_flagInitialConfig && _flagPlatformOutCommStarted)
			{
				_flagStairCaseFinished[X]=false;
				_flagStairCaseStarted[X]=false;
				_flagInitialConfig=true;
				ROS_INFO("Initializing...");
				ROS_INFO("Waiting for the integrator terms to go to zero...");
			}
		else
			{
				

				if (!_flagStairCaseStarted[X]) 
				{
					ROS_INFO("Generating Staircase for X");
				}

				if (fabs(_platform_effortM[X] - 0.0) <= 1e-4f)
				{
					_flagIntegratorsZeroed[X]=true;
				}
				if (_flagIntegratorsZeroed[X])
					{
						staircaseJointGen(X, FORWARD);
					}
				if (_currentStep[X]==_nSteps[X]) 
				{
					_flagIntegratorsZeroed[X] = false;
				}
				
				publishPositionOnly();
			}
	}
	ros::spinOnce();
	_loopRate.sleep();
  }

  //! Finish the node putting the set position to zero

  for (int k = 0; k < NB_AXIS; k++)
  {
	  _msgFootInput.ros_position[k] = 0.0f;
	  _msgFootInput.ros_speed[k] = _platform_speed[k];
	  _msgFootInput.ros_effort[k] = _platform_effortD[k];
  }

  publishPositionOnly();
  ROS_INFO("Friction Identification stoped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

double multiAxisFrictionID::stepFunGen(int axis_, int dir_)
{
	double outputQ = 0.0;
	bool flagStep = false;

	if (_flagNextStep[axis_][0] && _flagNextStep[axis_][1])
	{
		ROS_INFO("Step %i completed", _currentStep[axis_]);
		ROS_INFO("Friction value %f", _platform_effortM[axis_]);
		_currentStep[axis_] = _currentStep[axis_] + 1;
		outputQ = dir_ * 1.0;
		_flagNextStep[axis_][0] = false;
		_flagNextStep[axis_][1] = false;
		_prevTime[axis_] = ros::Time::now();
	}

	_currentTime = ros::Time::now();

	outputQ = dir_ * (1.0 - std::exp(-(_currentTime.toSec() - _prevTime[axis_].toSec()) / _tau[axis_]));
	
	if (((abs((double)dir_ - outputQ)) <= 1e-4f)) { _flagNextStep[axis_][1] = true; }
	
	// ROS_INFO("output %f", outputQ);
	return outputQ;
}

void multiAxisFrictionID::staircaseJointGen(int axis_, int dir_)
{
	if (axis_==-1)
	{
		for (int k=0; k<NB_AXIS; k++)
		{
			staircaseJointGen(k, dir_);
		}
	}

	else
	{
		if (!_flagStairCaseFinished[axis_])
		{	
			if (!_flagStairCaseStarted[axis_])
			{
				_currentStep[axis_]=0;
				_flagStairCaseStarted[axis_] = true;
				_prevTime[axis_] = ros::Time::now();
			}

			if (_currentStep[axis_] <= _nSteps[axis_] )
			{
				if ( ( abs(_ros_position[axis_] - _platform_position[axis_]) <= 1e-4f) )
				// && ( (_ros_position[axis_]-_platform_position[axis_]) >= 0) )
				{
					ROS_DEBUG_THROTTLE(300,"The error in position is less than 0.1 mm");
					_flagNextStep[axis_][0]=true;
				}

				_ros_position[axis_] = _limitWS[axis_] * (1.0 / _nSteps[axis_])  * (stepFunGen(axis_, dir_) + _currentStep[axis_]);
				
			}

			else
			{
				_flagStairCaseFinished[axis_] = true;
				_flagStairCaseStarted[axis_] = false;
				ROS_INFO("Staircase finished for axis %i", axis_);
			}
		}
	}
	
}

void multiAxisFrictionID::publishPositionOnly()
{
	_mutex.lock();
	for (int k=0; k<NB_AXIS; k++)
		{
			_msgFootInput.ros_position[k]=_ros_position[k];
			_msgFootInput.ros_speed[k]=_ros_speed[k];
			_msgFootInput.ros_effort[k]=_ros_effort[k];
		}
	_pubFootInput.publish(_msgFootInput);
	_flagPositionOnlyPublished = true;
	_mutex.unlock();
}

void multiAxisFrictionID::fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg)
{
	_flagOutputMessageReceived=true;
	_platform_id = msg->platform_id; 
	for (int k=0; k<NB_AXIS; k++)
		{
			_platform_position[k]=msg->platform_position[k];
			_platform_speed[k]=msg->platform_speed[k];
			_platform_effortD[k]=msg->platform_effortD[k];
			_platform_effortM[k]=msg->platform_effortM[k];
		}	
	_platform_machineState= (multiAxisFrictionID::State) msg->platform_machineState;
	_platform_controllerType= (multiAxisFrictionID::Controller) msg->platform_controllerType;
	if(!_flagPlatformOutCommStarted)
	{_flagPlatformOutCommStarted=true;}
} 
