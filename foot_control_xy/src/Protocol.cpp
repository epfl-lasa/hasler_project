#include "foot_control_xy/Protocol.h"


Protocol* Protocol::me = NULL;

Protocol::Protocol(ros::NodeHandle &n, double frequency, Eigen::Vector3f initTargetPosition): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_targetPosition(initTargetPosition)
{
  me=this;
  _stop = false;
  _firstChaserPoseReceived = false;
  _chaserPosition.setConstant(0.0f);
  _targetInfo.resize(0);
}

bool Protocol::init()
{
  
  //Subscriber definitions
  _subChaserPose = _n.subscribe("chaser/pose",1,&Protocol::updateChaserPose,this,ros::TransportHints().reliable().tcpNoDelay());
  
  //Publisher definitions
  _pubTargetPose = _n.advertise<geometry_msgs::PoseStamped>("target/pose", 1);

  signal(SIGINT,Protocol::stopNode);
  
  if (_n.ok()) 
  { 
    ros::spinOnce();
    ROS_INFO("The Protocol node is about to start");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void Protocol::stopNode(int sig)
{
    me->_stop= true;
}


void Protocol::run()
{
  srand(time(NULL));

  while (!_stop) 
  {
    
    if(_firstChaserPoseReceived)
    {
      _mutex.lock();

      checkIfTargetReached();

      if(_targetReached);
      {
        updateTargetPose();
      }

      publishData();
      
      _mutex.unlock();
    }

    ros::spinOnce();
    _loopRate.sleep();
  }
  
  ros::shutdown();

  std::cerr << "Result: Target position [m] -- Elapsed Time [s]" << std::endl;
  float totalTimeElapsed = 0.0f;

  Eigen::IOFormat customFormat(3,0);
  for(int k = 0; k < _targetInfo.size(); k++)
  {
    std::cerr << "Target " << k+1 << ": " << _targetInfo[k].position.transpose().format(customFormat) << " -- " << _targetInfo[k].elapsedTime << std::endl;
    totalTimeElapsed += _targetInfo[k].elapsedTime;
  }
  std::cerr << "Total number of target reached: " << _targetInfo.size() << " Total elapsed time: " << totalTimeElapsed << std::endl;
}


void Protocol::checkIfTargetReached()
{
  float error = (_targetPosition-_chaserPosition).norm();
  std::cerr << error << std::endl;

  if(error<TARGET_TOLERANCE)
  {
    _targetReached = true;
  }
  else
  {
    _targetReached = false;
  }
}


void Protocol::updateTargetPose()
{
  if(_targetReached)
  {
    TargetInfo info;
    info.position = _targetPosition;
    info.elapsedTime = float(ros::Time::now().toSec()-_tInit);
    _targetInfo.push_back(info);

    _targetPosition(0) = 5.0f*(-1.0f+2.0f*(float)std::rand()/RAND_MAX);
    _targetPosition(1) = 5.0f*(-1.0f+2.0f*(float)std::rand()/RAND_MAX);
    _targetPosition(2) = 0.0f;

    _tInit = ros::Time::now().toSec();
  }
}


void Protocol::publishData()
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
}


void Protocol::updateChaserPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _chaserPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstChaserPoseReceived)
  {
    _firstChaserPoseReceived = true;
    _tInit = ros::Time::now().toSec();
  }
}

