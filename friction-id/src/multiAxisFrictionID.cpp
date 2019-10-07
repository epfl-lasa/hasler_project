#include "multiAxisFrictionID.h"
#include "../../5_axis_platform/lib/platform/src/definitions.h"
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

char const *Strategy_names[] {"none", "steps", "ramp"};
char const *Direction_names[] {"forward", "reverse"};

#define FUNC_ERROR 0
#define POS_ERROR 1

float clampSymmetric(float input_, float limit_)
{
	float output = input_ > limit_ ? limit_ : (input_ < -limit_ ? -limit_ : input_);
	return output;
}

multiAxisFrictionID* multiAxisFrictionID::me = NULL;

multiAxisFrictionID::multiAxisFrictionID(ros::NodeHandle &n_1, double frequency,multiAxisFrictionID::Platform_Name platform_id, multiAxisFrictionID::Strategy strategy_, float tau_, int8_t whichAxis_): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_strategy(strategy_),
_tau(tau_),
_whichAxis(whichAxis_),
_dt(1.0f/frequency)
{
	
	 me=this;
	_stop = false;
    _flagOutputMessageReceived = false;
	_flagPlatformInCommStarted=false;
	_flagPlatformOutCommStarted = false;
	_flagPositionOnlyPublished = false;
	_flagNextAxis=false;

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

    _whichAxis=-1;
    _currentAxis=X;
	_direction=FORWARD;
    
	    //Staircase

	_currentTime =ros::Time::now();
		
	for(int k = 0 ; k<NB_AXIS; k++)
	{
		_prevTime[k] = ros::Time::now();
		_nSteps[k]= NB_STEPS_DEFAULT;
		_currentStep[k] = 0;
		_currentOffset[k]= 0;
		_flagStrategyFinished[k][FORWARD] = false;
		_flagStrategyFinished[k][REVERSE] = false;
		_flagStrategyStarted[k][FORWARD] = false;
		_flagStrategyStarted[k][REVERSE] = false;
		_flagNextStep[k][FORWARD][FUNC_ERROR] = false;
		_flagNextStep[k][FORWARD][POS_ERROR] = false;
		_flagNextStep[k][REVERSE][FUNC_ERROR] = false;
		_flagNextStep[k][REVERSE][POS_ERROR] = false;

		_flagIntegratorsZeroed[k][FORWARD]=false;
		_flagIntegratorsZeroed[k][REVERSE]=false;
		_flagWSDefined[k][FORWARD]=false;
		_flagWSDefined[k][REVERSE]=false;				

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
		ROS_INFO("The functions generator for friction identification is about to start");
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
		_currentTime = ros::Time::now();
		if (_flagPlatformOutCommStarted &&
			(_platform_controllerType == POSITION_ONLY) && 
			(_platform_machineState == ROBOT_STATE_CONTROL))
		{
			//! Direction Check
			if (_direction_prev != _direction)
				{
					ROS_INFO("Switching to direction %s", Direction_names[_direction]);
					_direction_prev = _direction;
				}

			axisSequenceControl(X, ONE_AFTER_THE_OTHER);

			//! Empty integrators Check

			 if (abs(_platform_effortM[_currentAxis]) <= 0.001) //! [N] || [N/m]
			 {
			 	ROS_INFO("Integrator in %s went to zero!", Axis_names[_currentAxis]);
			 	_flagIntegratorsZeroed[_currentAxis][_direction]=true;
			 }

			if (_flagIntegratorsZeroed[_currentAxis][_direction])
				{

					if (!_flagStrategyStarted[_currentAxis][_direction])
					{
						if (!_flagWSDefined[_currentAxis][FORWARD] && !_flagWSDefined[_currentAxis][REVERSE])
						{
							{
								ROS_INFO("Generating %s for %s in %s direction ", Strategy_names[_strategy], Axis_names[_currentAxis], Direction_names[_direction]);
								int dir_ = _direction == FORWARD ? 1 : -1;
								_WS[_currentAxis] = abs(dir_ * (WS_LIMITS[_currentAxis] - _platform_position[_currentAxis]));
								_stepSize[_currentAxis] = _WS[_currentAxis] / NB_STEPS_DEFAULT;
								ROS_INFO("Stepsize fixed at %f", _stepSize[_currentAxis]);
								_flagWSDefined[_currentAxis][_direction] = true;
							}
						}

					}

					funcJointGen(_currentAxis, _direction, _tau);
				}

				if (_flagStrategyFinished[_currentAxis][FORWARD])
				{
					_direction = REVERSE;
				}
				if (_flagStrategyFinished[_currentAxis][REVERSE])
				{
					ROS_INFO("Identification using %s in axis %s completed", Strategy_names[_strategy], Axis_names[_currentAxis]);
				}
			
			publishPositionOnly();
		}
	ros::spinOnce();
	_loopRate.sleep();
	}

	//! Finish the node putting the set position to zero
	while ((ros::Time::now()-_currentTime).toSec() <= 1.0f)
	{
			for (int k = 0; k < NB_AXIS; k++)
			{
				_ros_position[k] = 0.0f;
				_ros_speed[k] = _platform_speed[k];
				_ros_effort[k] = _platform_effortD[k];
			}

		publishPositionOnly();
		ros::spinOnce();
		_loopRate.sleep();
	}
	ROS_INFO("Friction Identification stoped");
	ros::spinOnce();
	_loopRate.sleep();
	ros::shutdown();
}

void multiAxisFrictionID::axisSequenceControl(int whichAxis_, Sequence whichSequence_)
{
	//! TODO 
	if (whichAxis_ == -1)
	{

		switch (whichSequence_)
		{

			case ONE_AFTER_THE_OTHER :

			{
				if (_flagStrategyFinished[_currentAxis][FORWARD] &&
					_flagStrategyFinished[_currentAxis][REVERSE] && _currentAxis < YAW)
				{
					_flagNextAxis = true;
					_currentAxis++;
				}

				
				else
				{
					_flagNextAxis = false;
					if (_currentAxis == YAW &&
						_flagStrategyFinished[_currentAxis][FORWARD] &&
						_flagStrategyFinished[_currentAxis][REVERSE])
					{
						ROS_INFO("Sequencial Axis Identification Completed");
						_stop = true;
					}
				}
			break;
			}

			case ALL_TOGETHER:
			{
				// TDB
				break;
			}
		}
	}
	else
	{
		_currentAxis = whichAxis_;
		if (_flagStrategyFinished[_currentAxis][FORWARD] &&
			_flagStrategyFinished[_currentAxis][REVERSE])
		{
			ROS_INFO("%s axis idenfification completed", Axis_names[_currentAxis]);
			_stop = true;
		}
	}
}



















double multiAxisFrictionID::stepFunGen(int axis_, Direction direction_, float tau_, Strategy strategy_)
{
	int dir_ = direction_ == FORWARD ? 1 : -1; 
	double outputQ = 0.0;
	bool flagStep = false;



	_currentTime = ros::Time::now();

	if (strategy_==STEPS)
		{
			outputQ = (1.0f - std::exp(-(_currentTime.toSec() - _prevTime[axis_].toSec()) / tau_));
			outputQ = outputQ <= 1.0f ? outputQ : 1.0f; //! clamp the value
		}
	
	else if (strategy_==RAMP)
	{
		outputQ = ((_currentTime.toSec() - _prevTime[axis_].toSec()) / tau_);
		outputQ = outputQ<=1.0f ? outputQ : 1.0f; //! clamp the value
		
	}

	if (1.0f == outputQ) 
		{ _flagNextStep[axis_][_direction][FUNC_ERROR] = true;
		  _currentOffset[axis_] += dir_;}

		if (_flagNextStep[axis_][direction_][FUNC_ERROR])// &&
			//_flagNextStep[axis_][direction_][POS_ERROR])
		{
			ROS_INFO("Step %i completed for axis %s", _currentStep[axis_], Axis_names[axis_]);
			ROS_INFO("Friction value %f in %s", _platform_effortM[axis_], Axis_names[axis_]);
			_flagNextStep[axis_][direction_][FUNC_ERROR] = false;
			_currentStep[axis_] += 1;
			_prevTime[axis_] = ros::Time::now();
	}

		ROS_INFO("output %f", outputQ);
		return outputQ;
}






void multiAxisFrictionID::funcJointGen(int axis_, Direction direction_, float tau_)
{
	if (axis_==-1)
	{
		for (int k=0; k<NB_AXIS; k++)
		{
			funcJointGen(k, direction_,tau_);
		}
	}

	else
	{

		if (!_flagStrategyFinished[axis_][_direction])
		{	
			if (!_flagStrategyStarted[axis_][_direction])
			{
			_currentStep[axis_]=1;
			ROS_INFO("HERE");
			_prevTime[axis_] = ros::Time::now();
			_flagStrategyStarted[axis_][_direction] = true;
			}
			
			if (_currentStep[axis_] <= _nSteps[axis_])
			{
				if ( ( abs(_ros_position[axis_] - _platform_position[axis_]) <= 28e-6f) ) //!28um
				{
					_flagNextStep[axis_][direction_][POS_ERROR]=true;
				}
				else{
					_flagNextStep[axis_][direction_][POS_ERROR] = false;
				}
				int dir_ = direction_ == FORWARD ? 1 : -1;
				_ros_position[axis_] = _stepSize[axis_] * (_currentOffset[axis_] + dir_*stepFunGen(axis_, direction_, tau_, _strategy));
				_ros_position[axis_] = clampSymmetric(_ros_position[axis_], WS_LIMITS[axis_]);
			}

			else
			{
				_flagStrategyStarted[axis_][_direction] = false;
				_flagStrategyFinished[axis_][_direction] = true;
                 ROS_INFO("Staircase finished for axis %i for direction %s", axis_,Direction_names[_direction]);
				_flagIntegratorsZeroed[axis_][_direction] = false;
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
