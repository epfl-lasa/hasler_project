#include "reach_target.h"


ReachTarget* ReachTarget::me = NULL;

ReachTarget::ReachTarget(ros::NodeHandle &n, double frequency, std::string filename): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_filename(filename)
{
  me=this;
  _stop = false;
  for(int k = 0; k < 2; k++)
  {
    _footPosition[k].setConstant(0.0f);
  }

  _cubePosition.setConstant(0.0f);

  _firstCubePose = false;
  _cubeGrasped = false;
  _targetReached = false;
  _firstFootPosition[LEFT] = false;
  _firstFootPosition[RIGHT] = false;
  _firstForce[LEFT] = false;
  _firstForce[RIGHT] = false;

}

bool ReachTarget::init()
{
  
  _subFootPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/right_foot/simulated_pose",1, boost::bind(&ReachTarget::updateFootPose,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/left_foot/simulated_pose",1, boost::bind(&ReachTarget::updateFootPose,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCubePose = _n.subscribe<geometry_msgs::Pose>("/cube/pose",1, &ReachTarget::updateCubePose,this,ros::TransportHints().reliable().tcpNoDelay());

  _subForce[RIGHT] = _n.subscribe<geometry_msgs::Vector3>("/right_foot/force",1, boost::bind(&ReachTarget::updateForce,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subForce[LEFT] = _n.subscribe<geometry_msgs::Vector3>("/left_foot/force",1, boost::bind(&ReachTarget::updateForce,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  //Publisher definitions
  _pubTargetPose = _n.advertise<geometry_msgs::PoseStamped>("target/pose", 1);
  _pubCurrentTime = _n.advertise<std_msgs::Float32>("/currentTime", 1);
  
  ROS_INFO("[ReachTarget]: Filename: %s", _filename.c_str());

  _outputFile.open(ros::package::getPath(std::string("demo_scientastic"))+"/data/"+_filename+".txt");

  if(!_outputFile.is_open())
  {
    ROS_ERROR("[ReachTarget]: Cannot open output data file, the data directory might be missing");
    return false;
  }

  signal(SIGINT,ReachTarget::stopNode);
  
  if (_n.ok()) 
  { 
    ros::spinOnce();
    ROS_INFO("The joystick node started");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void ReachTarget::stopNode(int sig)
{
    me->_stop= true;
}


void ReachTarget::run()
{
  while (!_stop) 
  {
    _currentTime = ros::Time::now().toSec();

    if(_firstFootPosition[LEFT] && _firstFootPosition[RIGHT] && _firstCubePose)
    {
      _mutex.lock();

      receiveFrames();

      checkIfObjectGrasped();

      checkIfTargetReached();

      logData();

      publishData();
        
      _mutex.unlock();
    }

    if(_targetReached && _currentTime-_reachedTime>3.0f)
    {
      break;
    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  _outputFile.close();

  ros::shutdown();

}

void ReachTarget::receiveFrames()
{
  try
  { 
    _lr.lookupTransform("/world","/target/base_link",ros::Time(0), _transform);

    _targetPosition(0) = _transform.getOrigin().x();
    _targetPosition(1) = _transform.getOrigin().y();
    _targetPosition(2) = _transform.getOrigin().z();
    // std::cerr << _targetPosition.transpose() << std::endl;
  } 
  catch (tf::TransformException ex)
  {

  }
}

void ReachTarget::checkIfObjectGrasped()
{
  Eigen::Vector3f centerFeetPosition;
  centerFeetPosition = (_footPosition[LEFT]+_footPosition[RIGHT])/2.0f;
  float feetDistance = (_footPosition[RIGHT]-_footPosition[LEFT]).norm();
  float error = (centerFeetPosition-_cubePosition).segment(0,3).norm();

  // std::cerr << error << " " << feetDistance << std::endl;
  if(error< 0.5f && feetDistance < 2.5f && !_cubeGrasped)
  {
    _cubeGrasped = true;
    _startingTime = _currentTime;
    std::cerr << "[ReachTarget]: cube grasped" << std::endl;
  }
}


void ReachTarget::checkIfTargetReached()
{
  Eigen::Vector3f temp;
  temp = _targetPosition;
  temp(2)+=0.5f;
  float error = (temp-_cubePosition).norm();

  if(error<0.5 && !_targetReached)
  {
    _targetReached = true;
    _reachedTime = _currentTime;
    std::cerr << "[ReachTarget]: target reached: " << _reachedTime-_startingTime << std::endl;

  }
}


void ReachTarget::logData()
{
  _outputFile << ros::Time::now() << " "
              << _footPosition[LEFT].transpose() << " "
              << _footPosition[RIGHT].transpose() << " "
              << _targetPosition.transpose() << " "
              << _cubePosition.transpose() << " "
              << _force[LEFT].transpose() << " "
              << _force[RIGHT].transpose() << std::endl;
}


void ReachTarget::publishData()
{
  _msgTargetPose.header.frame_id = "world";
  _msgTargetPose.header.stamp = ros::Time::now();
  _msgTargetPose.pose.position.x = _targetPosition(0);
  _msgTargetPose.pose.position.y = _targetPosition(1);
  _msgTargetPose.pose.position.z = _targetPosition(2);
  _msgTargetPose.pose.orientation.x = 0.0f;
  _msgTargetPose.pose.orientation.y = 0.0f;
  _msgTargetPose.pose.orientation.z = 0.0f;
  _msgTargetPose.pose.orientation.w = 1.0f;
  _pubTargetPose.publish(_msgTargetPose);

  std_msgs::Float32 msg;
  if(_cubeGrasped && !_targetReached)
  {
    msg.data = _currentTime-_startingTime;
  }
  else if(_cubeGrasped && _targetReached)
  {
    msg.data = _reachedTime-_startingTime;
  }
  else
  {
    msg.data = 0;
  }
  _pubCurrentTime.publish(msg);
}

void ReachTarget::updateFootPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  _footPosition[k] << msg->position.x, msg->position.y, msg->position.z;

  if(!_firstFootPosition[k])
  {
    _firstFootPosition[k] = true;
  }
}

void ReachTarget::updateCubePose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _cubePosition << msg->position.x, msg->position.y, msg->position.z;

  if(!_firstCubePose)
  {
    _firstCubePose = true;
  }
}

void ReachTarget::updateForce(const geometry_msgs::Vector3::ConstPtr& msg, int k)
{
  _force[k] << msg->x, msg->y, msg->z;
  if(!_firstForce[k])
  {
    _firstForce[k] = true;
  }
}