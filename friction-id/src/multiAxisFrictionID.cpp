#include "multiAxisFrictionID.h"


#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

char const *Strategy_names[] {"NONE", "STAIRCASE", "RAMP"};
char const *Direction_names[] {"FORWARD_1", "REVERSE_1", "REVERSE_2", "FORWARD_2"};

#define FUNC_ERROR 0
#define POS_ERROR 1

float clampSymmetric(float input_, float limit_)
{
	float output = input_ > limit_ ? limit_ : (input_ < -limit_ ? -limit_ : input_);
	return output;
}

multiAxisFrictionID* multiAxisFrictionID::me = NULL;

multiAxisFrictionID::multiAxisFrictionID(ros::NodeHandle &n_1, double frequency,multiAxisFrictionID::Platform_Name platform_id, multiAxisFrictionID::Strategy strategy_, float tau_, int whichAxis_): 
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
	_sequence = ALL_TOGETHER;
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
		_ros_positionFilter[k] = new lp_filter(0.5);
		_ros_speed[k]=0.0f;
		_ros_effort[k]=0.0f;
		_direction[k] = FORWARD_1;
	}
    _currentAxis=X;
	
    
	    //Staircase

	_currentTime =ros::Time::now();
		
	for(unsigned int k = 0 ; k<NB_AXIS; k++)
	{
		_prevTime[k] = ros::Time::now();
		_nSteps[k]= NB_STEPS_DEFAULT;
		_currentStep[k] = 0;
		_currentOffset[k]= 0;

		for (unsigned int j = 0; j<NB_DIR; j++)
		{
			_flagStrategyFinished[k][j] = false;
			_flagStrategyStarted[k][j] = false;
			_flagNextStep[k][j][FUNC_ERROR] = false;
			_flagNextStep[k][j][POS_ERROR] = false;
			_flagIntegratorsZeroed[k][j] = false;
		}
		_flagWSDefined[k] = false;
	}


}
multiAxisFrictionID::~multiAxisFrictionID()
{
	for (int k=0; k<NB_AXIS; k++)
	{
		delete(_ros_positionFilter[k]); 
	}
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
			axisSequenceControl(_whichAxis, _sequence);
			functionControl(_whichAxis, _sequence);
		
			publishPositionOnly();
		}
	ros::spinOnce();
	_loopRate.sleep();
	}

	//! Finish the node putting the set position to zero
	// while ((ros::Time::now()-_currentTime).toSec() <= 1.0f)
	// {
	// 		for (int k = 0; k < NB_AXIS; k++)
	// 		{
	// 			_ros_position[k] = 0.0f;
	// 			_ros_speed[k] = _platform_speed[k];
	// 			_ros_effort[k] = _platform_effortD[k];
	// 		}

	// 	publishPositionOnly();
	// 	ros::spinOnce();
	// 	_loopRate.sleep();
	// }
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
				//! Direction Check
				if (_direction_prev[_currentAxis] != _direction[_currentAxis])
				{
					ROS_INFO("Switching to direction %s", Direction_names[_direction[_currentAxis]]);
					_direction_prev[_currentAxis] = _direction[_currentAxis];
				}

				if (_flagStrategyFinished[_currentAxis][FORWARD_1] &&
					_flagStrategyFinished[_currentAxis][REVERSE_1] &&
					_flagStrategyFinished[_currentAxis][FORWARD_2] &&
					_flagStrategyFinished[_currentAxis][REVERSE_2] &&
					_currentAxis < YAW)
				{
					_flagNextAxis = true;
					_currentAxis++;
					_direction[_currentAxis]=FORWARD_1;
					_direction_prev[_currentAxis]=FORWARD_2;
				}

				
				else
				{
					_flagNextAxis = false;
					if (_currentAxis == YAW &&
					_flagStrategyFinished[_currentAxis][FORWARD_1] &&
					_flagStrategyFinished[_currentAxis][REVERSE_1] &&
					_flagStrategyFinished[_currentAxis][FORWARD_2] &&
					_flagStrategyFinished[_currentAxis][REVERSE_2]) 
					{
						ROS_INFO("Sequencial Axis Identification Completed");
						_stop = true;
					}
				}
			break;
			}

			case ALL_TOGETHER:
			{
				bool flagAllFinished = true;

				for (int k = 0; k<NB_AXIS; k++)
				{
					for (int j = 0 ; j < NB_DIR ; j++)
					{
						flagAllFinished = flagAllFinished && _flagStrategyFinished[k][j];
					}
					 
				}
				if (flagAllFinished)
				{
					ROS_INFO("Simultaneous Axis Identification Completed");
					_stop = true;
				}

				break;
			}
		}
	}
	else
	{
		_currentAxis = whichAxis_;
		if (_flagStrategyFinished[_currentAxis][FORWARD_1] &&
			_flagStrategyFinished[_currentAxis][REVERSE_1] &&
			_flagStrategyFinished[_currentAxis][FORWARD_2] &&
			_flagStrategyFinished[_currentAxis][REVERSE_2])
		{
			ROS_INFO("%s axis idenfification completed", Axis_names[_currentAxis]);
			_stop = true;
		}
	}
}

void multiAxisFrictionID::functionControl(int whichAxis_, Sequence whichSequence_)
{
	if (whichAxis_ == -1)
	{

		switch (whichSequence_)
		{

			case ONE_AFTER_THE_OTHER:

			{
				//! Empty integrators Check

				//if (abs(_platform_effortM[_currentAxis]) <= 0.001) //! [N] || [N/m]
				if (true)
				{
					//ROS_INFO("Integrator in %s went to zero!", Axis_names[_currentAxis]);
					_flagIntegratorsZeroed[_currentAxis][_direction[_currentAxis]] = true;
				}

				if (_flagIntegratorsZeroed[_currentAxis][_direction[_currentAxis]])
				{

					if (!_flagStrategyStarted[_currentAxis][_direction[_currentAxis]])
					{

						if (!_flagWSDefined[_currentAxis])
						{
							{
								ROS_INFO("Generating %s for %s in %s direction ", Strategy_names[_strategy], Axis_names[_currentAxis], Direction_names[_direction[_currentAxis]]);
								int dir_ = (_direction[_currentAxis] == FORWARD_1 || _direction[_currentAxis] == FORWARD_2) ? 1 : -1;
								_WS[_currentAxis] = abs(dir_ * (WS_LIMITS[_currentAxis] - _platform_position[_currentAxis]));
								_stepSize[_currentAxis] = _WS[_currentAxis] / NB_STEPS_DEFAULT;
								ROS_INFO("Stepsize fixed at %f", _stepSize[_currentAxis]);
								_flagWSDefined[_currentAxis] = true;
							}
						}
					}

					funcJointGen(_currentAxis, _tau);
				}

				if (_flagStrategyFinished[_currentAxis][_direction[_currentAxis]])
				{
					_direction[_currentAxis] = (Direction)((int)_direction[_currentAxis] + 1);
					_direction[_currentAxis] = (int)_direction[_currentAxis] < NB_DIR ? _direction[_currentAxis] : (Direction)(NB_DIR - 1);
				}

				if (_flagStrategyFinished[_currentAxis][FORWARD_2])
				{
					ROS_INFO("Identification using %s in axis %s completed", Strategy_names[_strategy], Axis_names[_currentAxis]);
				}

				break;
			}

			case ALL_TOGETHER:
			{
				bool finished = true;
				for (int k = 0; k<NB_AXIS; k++)
				{
					//! Empty integrators Check

					//if (abs(_platform_effortM[_currentAxis]) <= 0.001) //! [N] || [N/m]
					if (true)
					{
						//ROS_INFO("Integrator in %s went to zero!", Axis_names[_currentAxis]);
						_flagIntegratorsZeroed[k][_direction[k]] = true;
					}

					if (_flagIntegratorsZeroed[k][_direction[k]])
					{

						if (!_flagStrategyStarted[k][_direction[k]])
						{

							if (!_flagWSDefined[k])
							{
								{
									ROS_INFO("Generating %s for %s in %s direction ", Strategy_names[_strategy], Axis_names[k], Direction_names[_direction[k]]);
									int dir_ = (_direction[k] == FORWARD_1 || _direction[k] == FORWARD_2) ? 1 : -1;
									_WS[k] = abs(dir_ * (WS_LIMITS[k] - _platform_position[k]));
									_stepSize[k] = _WS[k] / NB_STEPS_DEFAULT;
									ROS_INFO("Stepsize fixed at %f", _stepSize[k]);
									_flagWSDefined[k] = true;
								}
							}
						}

						funcJointGen(k, _tau);
					}

					if (_flagStrategyFinished[k][_direction[k]])
					{
						_direction[k] = (Direction)((int)_direction[k] + 1);
						_direction[k] = (int)_direction[k] < NB_DIR ? _direction[k] : (Direction)(NB_DIR - 1);
					}

					if (_flagStrategyFinished[k][FORWARD_2])
					{
						ROS_INFO("Identification using %s in axis %s completed", Strategy_names[_strategy], Axis_names[_currentAxis]);
						finished = true; 
					}
					else{
						finished = false;
					}

				}
				if (finished) { _stop = true;}
				break;	
			}
		}
	}
	else
	{
		_currentAxis = whichAxis_;
		//! Empty integrators Check

		//if (abs(_platform_effortM[_currentAxis]) <= 0.001) //! [N] || [N/m]
		if (true)
		{
			//ROS_INFO("Integrator in %s went to zero!", Axis_names[_currentAxis]);
			_flagIntegratorsZeroed[_currentAxis][_direction[_currentAxis]] = true;
		}

		if (_flagIntegratorsZeroed[_currentAxis][_direction[_currentAxis]])
		{

			if (!_flagStrategyStarted[_currentAxis][_direction[_currentAxis]])
			{

				if (!_flagWSDefined[_currentAxis])
				{
					{
						ROS_INFO("Generating %s for %s in %s direction ", Strategy_names[_strategy], Axis_names[_currentAxis], Direction_names[_direction[_currentAxis]]);
						int dir_ = (_direction[_currentAxis] == FORWARD_1 || _direction[_currentAxis] == FORWARD_2) ? 1 : -1;
						_WS[_currentAxis] = abs(dir_ * (WS_LIMITS[_currentAxis] - _platform_position[_currentAxis]));
						_stepSize[_currentAxis] = _WS[_currentAxis] / NB_STEPS_DEFAULT;
						ROS_INFO("Stepsize fixed at %f", _stepSize[_currentAxis]);
						_flagWSDefined[_currentAxis] = true;
					}
				}
			}

			funcJointGen(_currentAxis, _tau);
		}

		if (_flagStrategyFinished[_currentAxis][_direction[_currentAxis]])
		{
			_direction[_currentAxis] = (Direction)((int)_direction[_currentAxis] + 1);
			_direction[_currentAxis] = (int)_direction[_currentAxis] < NB_DIR ? _direction[_currentAxis] : (Direction)(NB_DIR - 1);
		}

		if (_flagStrategyFinished[_currentAxis][FORWARD_2])
		{
			ROS_INFO("Identification using %s in axis %s completed", Strategy_names[_strategy], Axis_names[_currentAxis]);
		}
	}
	

}








double multiAxisFrictionID::stepFunGen(int axis_, float tau_, Strategy strategy_)
{
	int dir_ = (_direction[axis_] == FORWARD_1 || _direction[axis_] == FORWARD_2) ? 1 : -1;
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
		{ _flagNextStep[axis_][_direction[_currentAxis]][FUNC_ERROR] = true;
		  _currentOffset[axis_] += dir_;}

		if (_flagNextStep[axis_][_direction[axis_]][FUNC_ERROR]) // &&
		//_flagNextStep[axis_][direction_][POS_ERROR])
		{
			ROS_INFO("Step %i completed for axis %s", _currentStep[axis_], Axis_names[axis_]);
			// ROS_INFO("Friction value %f in %s", _platform_effortM[axis_], Axis_names[axis_]);
			_flagNextStep[axis_][_direction[axis_]][FUNC_ERROR] = false;
			_currentStep[axis_] += 1;
			_prevTime[axis_] = ros::Time::now();
	}

		ROS_INFO("output %f", outputQ);
		return outputQ;
}






void multiAxisFrictionID::funcJointGen(int axis_, float tau_)
{
	if (axis_==-1)
	{
		for (int k=0; k<NB_AXIS; k++)
		{
			funcJointGen(k, tau_);
		}
	}

	else
	{

		if (!_flagStrategyFinished[axis_][_direction[_currentAxis]])
		{	
			if (!_flagStrategyStarted[axis_][_direction[_currentAxis]])
			{
			_currentStep[axis_]=1;
			//ROS_INFO("HERE");
			_prevTime[axis_] = ros::Time::now();
			_flagStrategyStarted[axis_][_direction[_currentAxis]] = true;
			}
			
			if ( (_currentStep[axis_] <= _nSteps[axis_]) )
			
			{
				if ( ( abs(_ros_position[axis_] - _platform_position[axis_]) <= 28e-6f) ) //!28um
				{
					_flagNextStep[axis_][_direction[_currentAxis]][POS_ERROR]=true;
				}
				else{
					_flagNextStep[axis_][_direction[_currentAxis]][POS_ERROR] = false;
				}
				int dir_ = (_direction[_currentAxis] == FORWARD_1 || _direction[_currentAxis] == FORWARD_2 ) ? 1 : -1;
				_ros_position[axis_] = _stepSize[axis_] * (_currentOffset[axis_] + dir_*stepFunGen(axis_, tau_, _strategy));
				_ros_position[axis_] = clampSymmetric(_ros_position[axis_], WS_LIMITS[axis_]);
				_ros_position[axis_] = _ros_positionFilter[axis_]->update(_ros_position[axis_]);
			}

			else
			{
				_flagStrategyStarted[axis_][_direction[_currentAxis]] = false;
				_flagStrategyFinished[axis_][_direction[_currentAxis]] = true;
				ROS_INFO("%s finished for axis %i for direction %s", Strategy_names[_strategy], axis_, Direction_names[_direction[_currentAxis]]);
				_flagIntegratorsZeroed[axis_][_direction[_currentAxis]] = false;
			}
		}
	}
}



void multiAxisFrictionID::publishPositionOnly()
{
	_mutex.lock();
	for (int k=0; k<NB_AXIS; k++)
		{
			_msgFootInput.ros_position[rosAxis[k]]=_ros_position[k];
			_msgFootInput.ros_speed[rosAxis[k]]=_ros_speed[k];
			_msgFootInput.ros_effort[rosAxis[k]]=_ros_effort[k];
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
			_platform_position[k]=msg->platform_position[rosAxis[k]];
			_platform_speed[k]=msg->platform_speed[rosAxis[k]];
			_platform_effortD[k]=msg->platform_effortD[rosAxis[k]];
			_platform_effortM[k]=msg->platform_effortM[rosAxis[k]];
		}	
	_platform_machineState= (multiAxisFrictionID::State) msg->platform_machineState;
	_platform_controllerType= (multiAxisFrictionID::Controller) msg->platform_controllerType;
	if(!_flagPlatformOutCommStarted)
	{_flagPlatformOutCommStarted=true;}
} 
