#include "build_tower.h"


BuildTower* BuildTower::me = NULL;

BuildTower::BuildTower(ros::NodeHandle &n, double frequency, std::string filename): 
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
    _firstFootPosition[k] = false;
    _firstForce[k] = false;
  }

  for(int k = 0; k < 3; k++)
  {
    _cubePosition[k].setConstant(0.0f);
    _firstCubePose[k] = false;
    
  }

  _cubeGrasped = false;
  _towerBuilt = false;

}

bool BuildTower::init()
{
  
  _subFootPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/right_foot/simulated_pose",1, boost::bind(&BuildTower::updateFootPose,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/left_foot/simulated_pose",1, boost::bind(&BuildTower::updateFootPose,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCubePose[0] = _n.subscribe<geometry_msgs::Pose>("/box1/pose",1, boost::bind(&BuildTower::updateCubePose,this,_1,0), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCubePose[1] = _n.subscribe<geometry_msgs::Pose>("/box2/pose",1, boost::bind(&BuildTower::updateCubePose,this,_1,1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCubePose[2] = _n.subscribe<geometry_msgs::Pose>("/box3/pose",1, boost::bind(&BuildTower::updateCubePose,this,_1,2), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subForce[RIGHT] = _n.subscribe<geometry_msgs::Vector3>("/right_foot/force",1, boost::bind(&BuildTower::updateForce,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subForce[LEFT] = _n.subscribe<geometry_msgs::Vector3>("/left_foot/force",1, boost::bind(&BuildTower::updateForce,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  //Publisher definitions
  _pubTargetPose = _n.advertise<geometry_msgs::PoseStamped>("target/pose", 1);
  _pubCurrentTime = _n.advertise<std_msgs::Float32>("/currentTime", 1);
  
  ROS_INFO("[BuildTower]: Filename: %s", _filename.c_str());

  _outputFile.open(ros::package::getPath(std::string("demo_scientastic"))+"/data/"+"test_build_tower_"+_filename+".txt");

  if(!_outputFile.is_open())
  {
    ROS_ERROR("[BuildTower]: Cannot open output data file, the data directory might be missing");
    return false;
  }

  signal(SIGINT,BuildTower::stopNode);
  
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


void BuildTower::stopNode(int sig)
{
    me->_stop= true;
}


void BuildTower::run()
{
  while (!_stop) 
  {
    _currentTime = ros::Time::now().toSec();

    if(_firstFootPosition[LEFT] && _firstFootPosition[RIGHT] && _firstCubePose[0]
       && _firstCubePose[1] && _firstCubePose[2])
    {
      _mutex.lock();

      checkIfCubeGrasped();

      checkIfTowerBuilt();

      logData();

      publishData();
        
      _mutex.unlock();
    }

    if(_towerBuilt && _currentTime-_reachedTime>3.0f)
    {
      break;
    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  _outputFile.close();

  ros::shutdown();

}


void BuildTower::checkIfCubeGrasped()
{

  Eigen::Vector3f centerFeetPosition;
  
  centerFeetPosition = (_footPosition[LEFT]+_footPosition[RIGHT])/2.0f;
  float feetDistance = (_footPosition[RIGHT]-_footPosition[LEFT]).norm();
  
  for(int k = 0; k < 3; k++)
  {
    float error = (centerFeetPosition-_cubePosition[k]).segment(0,3).norm();

    // std::cerr << error << " " << feetDistance << std::endl;
    if(error< 0.5f && feetDistance < 2.5f && _force[LEFT].norm()>3 && _force[RIGHT].norm()>3 && !_cubeGrasped)
    {
      _cubeGrasped = true;
      _startingTime = _currentTime;
      std::cerr << "[BuildTower]: cube grasped" << std::endl;
    } 
  }
}


void BuildTower::checkIfTowerBuilt()
{

  Eigen::Vector3f temp;
  temp << _cubePosition[0](2),_cubePosition[1](2),_cubePosition[2](2);

  Eigen::Vector3f::Index idMax,idMax2,idMin;
  temp.array().maxCoeff(&idMax);
  temp.array().minCoeff(&idMin);
  for(Eigen::Vector3f::Index k = 0; k < 3;k++)
  {
    if(k!=idMax && k !=idMin)
    {
      idMax2 = k;
    }
  }

  float d01 = (_cubePosition[0].segment(0,2)-_cubePosition[1].segment(0,2)).norm();
  float d02 = (_cubePosition[0].segment(0,2)-_cubePosition[2].segment(0,2)).norm();
  float d12 = (_cubePosition[1].segment(0,2)-_cubePosition[2].segment(0,2)).norm();

  if(fabs(temp(idMax)-2.5f)<1e-3f && 
     fabs(temp(idMax2)-1.5f)<1e-3f &&
     d01<1.0f && d02 < 1.0f && d12 <1.0f && _towerBuilt == false)
  {
    _towerBuilt = true;
    _reachedTime = _currentTime;
    std::cerr << "[BuildTower]: tower built: " << _reachedTime-_startingTime << std::endl;    
  }
  else
  {
    // std::cerr <<temp(idMax) << " " <<temp(idMax2) << " " << d01 << " " << d02 << " " << d12 << std::endl;
  }
  // Eigen::Vector3f temp;
  // // temp = _targetPosition;
  // temp(2)+=0.5f;
  // fabs(_temp(idMax)-2.5f)<FLT_EPSILON float error = (temp-_cubePosition).norm();

  // if(error<0.5 && !_towerBuilt)
  // {
  //   _towerBuilt = true;
  //   _reachedTime = _currentTime;
  //   std::cerr << "[BuildTower]: target reached: " << _reachedTime-_startingTime << std::endl;

  // }
}


void BuildTower::logData()
{
  _outputFile << ros::Time::now() << " "
              << _footPosition[LEFT].transpose() << " "
              << _footPosition[RIGHT].transpose() << " "
              << _cubePosition[0].transpose() << " "
              << _cubeOrientation[0].transpose() << " "
              << _cubePosition[1].transpose() << " "
              << _cubeOrientation[1].transpose() << " "
              << _cubePosition[2].transpose() << " "
              << _cubeOrientation[2].transpose() << " "
              << _force[LEFT].transpose() << " "
              << _force[RIGHT].transpose() << " "
              << (int) _cubeGrasped << " " 
              << (int) _towerBuilt << std::endl;
}


void BuildTower::publishData()
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
  if(_cubeGrasped && !_towerBuilt)
  {
    msg.data = _currentTime-_startingTime;
  }
  else if(_cubeGrasped && _towerBuilt)
  {
    msg.data = _reachedTime-_startingTime;
  }
  else
  {
    msg.data = 0;
  }
  _pubCurrentTime.publish(msg);
}

void BuildTower::updateFootPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  _footPosition[k] << msg->position.x, msg->position.y, msg->position.z;


  if(!_firstFootPosition[k])
  {
    _firstFootPosition[k] = true;
  }
}

void BuildTower::updateCubePose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  _cubePosition[k] << msg->position.x, msg->position.y, msg->position.z;
  _cubeOrientation[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;

  if(!_firstCubePose[k])
  {
    _firstCubePose[k] = true;
  }
}

void BuildTower::updateForce(const geometry_msgs::Vector3::ConstPtr& msg, int k)
{
  _force[k] << msg->x, msg->y, msg->z;
  if(!_firstForce[k])
  {
    _firstForce[k] = true;
  }
}