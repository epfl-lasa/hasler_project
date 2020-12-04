#include "MoveToDesiredJoints.h"
// #include <tf/transform_datatypes.h>


MoveToDesiredJoints::MoveToDesiredJoints(ros::NodeHandle &n, float frequency, Mode mode, float jointTolerance):
	_n(n),
  _loopRate(frequency),
  _mode(mode),
	_jointTolerance(jointTolerance)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredJoints::init() 
{


  // Set the number of joints
	for(int k = 0; k < NB_ROBOTS; k++)
	{

		_desiredJoints[k].data.resize(NB_JOINTS);
		_targetJoints[k].data.resize(NB_JOINTS);
		_currentJoints[k].data.resize(NB_JOINTS);

		for(int m = 0; m < NB_JOINTS; m++)
		{
			_jointFilter[k][m].setAlpha(0.98);
			_currentJoints[k].data[m] = 0.0f;
			_desiredJoints[k].data[m] = 0.0f;
			_targetJoints[k].data[m] = 0.0f;
		}
  	_firstJointsUpdate[k] = false;
	}

  // Subscribe to joint states topic
  _subCurrentJoints[RIGHT] = _n.subscribe<sensor_msgs::JointState>("/right_lwr/joint_states", 1, boost::bind(&MoveToDesiredJoints::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subCurrentJoints[LEFT] = _n.subscribe<sensor_msgs::JointState>("/left_lwr/joint_states", 1, boost::bind(&MoveToDesiredJoints::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

  // Publish to the joint position controller topic
  _pubDesiredJoints[RIGHT] = _n.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/command_joint_pos", 1);
  _pubDesiredJoints[LEFT] = _n.advertise<std_msgs::Float64MultiArray>("/left_lwr/joint_controllers/command_joint_pos", 1);

	if (_n.ok())
	{ 
	  // Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void MoveToDesiredJoints::run() {

	while (_n.ok()) 
	{


		if(_mode == SINGLE_LEFT && _firstJointsUpdate[LEFT])
		{
			computeSmoothProfile(LEFT);
			for(int m = 0; m < NB_JOINTS; m++)
			{
				std::cerr << _desiredJoints[LEFT].data[m] << " ";
			}
			std::cerr << std::endl;
			_pubDesiredJoints[LEFT].publish(_desiredJoints[LEFT]);
			if(checkJointsError(LEFT))
			{
				ROS_INFO("The desired joints configuration is reached for left robot");
				break;
			}
		}
		else if(_mode == SINGLE_RIGHT && _firstJointsUpdate[RIGHT])
		{
			computeSmoothProfile(RIGHT);
			_pubDesiredJoints[RIGHT].publish(_desiredJoints[RIGHT]);
			if(checkJointsError(RIGHT))
			{
				ROS_INFO("The desired joints configuration is reached for right robot");
				break;
			}
		}
		else if(_mode == BOTH && _firstJointsUpdate[LEFT] && _firstJointsUpdate[RIGHT])
		{
			computeSmoothProfile(LEFT);
			computeSmoothProfile(RIGHT);
			_pubDesiredJoints[LEFT].publish(_desiredJoints[LEFT]);
			_pubDesiredJoints[RIGHT].publish(_desiredJoints[RIGHT]);
			if(checkJointsError(LEFT) && checkJointsError(RIGHT))
			{
				ROS_INFO("The desired joints configuration is reached for both robots");
				break;
			}
		}	

		ros::spinOnce();

		_loopRate.sleep();

	}
}


void MoveToDesiredJoints::setDesiredJoints(std_msgs::Float64MultiArray desiredJoints) 
{
	if(_mode == SINGLE_LEFT)
	{
		_targetJoints[LEFT] = desiredJoints;
	}
	else if(_mode == SINGLE_RIGHT)
	{
		_targetJoints[RIGHT] = desiredJoints;
	}
	else if(_mode == BOTH)
	{
		_targetJoints[LEFT] = desiredJoints;
		_targetJoints[RIGHT] = desiredJoints;	
	}
}

void MoveToDesiredJoints::computeSmoothProfile(int k)
{

	for(int m = 0; m < NB_JOINTS; m++)
	{
		_desiredJoints[k].data[m] = _jointFilter[k][m].update(_targetJoints[k].data[m]);
	}
}


void MoveToDesiredJoints::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k) 
{
	for(int m = 0; m < NB_JOINTS; m++)
	{
		_currentJoints[k].data[m] = msg->position[m];
	}
	if(!_firstJointsUpdate[k])
	{

		for(int m = 0; m < NB_JOINTS; m++)
		{
			_jointFilter[k][m].setInit(_currentJoints[k].data[m]);
		}
		_firstJointsUpdate[k] = true;

	}
}


bool MoveToDesiredJoints::checkJointsError(int k) 
{
	_mutex.lock();

	// Check of all joint angles reached their respectives targets
	bool reached = true;
	for(int m = 0; m < NB_JOINTS; m++)
	{
    if(fabs(_currentJoints[k].data[m]-_desiredJoints[k].data[m])>_jointTolerance)
    {
      reached = false;
      break;
    }
	}

	_mutex.unlock();

	return reached;
}