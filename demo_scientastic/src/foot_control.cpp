#include "foot_control.h"


FootControl* FootControl::me = NULL;

FootControl::FootControl(ros::NodeHandle &n, double frequency): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
{
  me=this;
  _stop = false;

  for(int k = 0; k < 2; k++)
  {
    _footPose[k].setConstant(0.0f);
    _footSensor[k].setConstant(0.0f);
    _footWrench[k].setConstant(0.0f);
    _footTwist[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _footState[k] = 0;
    _force[k].setConstant(0.0f);
    _ftSensorForce[k].setConstant(0.0f);
    _filteredForce[k].setConstant(0.0f);
    _desiredFootPosition[k].setConstant(0.0f);
    _desiredFootWrench[k].setConstant(0.0f);

    _footInterfaceSequenceID[k] = 0;

    _firstFootOutput[k] = false;
    _firstForce[k] = false;
    _xyPositionMapping = 14.0f;
    _zPositionMapping = 14.0f;
  }
}

bool FootControl::init()
{
  
  //Subscriber definitions
  _subFootOutput[RIGHT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&FootControl::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[LEFT] = _n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&FootControl::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  _subForce[RIGHT] = _n.subscribe<geometry_msgs::Vector3>("/right_foot/force",1, boost::bind(&FootControl::updateForce,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subForce[LEFT] = _n.subscribe<geometry_msgs::Vector3>("/left_foot/force",1, boost::bind(&FootControl::updateForce,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  _subFtSensorForce[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/right_foot/ft_sensor",1, boost::bind(&FootControl::updateFtSensorForce,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFtSensorForce[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/left_foot/ft_sensor",1, boost::bind(&FootControl::updateFtSensorForce,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  

  //Publisher definitions
  _pubDesiredFootPose[RIGHT] = _n.advertise<geometry_msgs::PoseStamped>("right_foot/pose", 1);
  _pubDesiredFootPose[LEFT] = _n.advertise<geometry_msgs::PoseStamped>("left_foot/pose", 1);

  _pubFootInput[RIGHT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Right", 1);
  _pubFootInput[LEFT] = _n.advertise<custom_msgs::FootInputMsg>("/FI_Input/Left", 1);

  signal(SIGINT,FootControl::stopNode);
  
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


void FootControl::stopNode(int sig)
{
    me->_stop= true;
}


void FootControl::run()
{
  while (!_stop) 
  {
    
    if(_firstFootOutput[LEFT] && _firstForce[LEFT] && _firstFootOutput[RIGHT] && _firstForce[RIGHT])
    {
      _mutex.lock();

      footDataTransformation();

      positionPositionMapping();

      computeDesiredFootWrench();

      publishData();
      
      _mutex.unlock();
    }

    ros::spinOnce();
    _loopRate.sleep();
  }
  
  ros::shutdown();

}

void FootControl::publishData()
{
  for(int k = 0; k < 2; k++)
  {
    _msgDesiredFootPose.header.frame_id = "world";
    _msgDesiredFootPose.header.stamp = ros::Time::now();
    _msgDesiredFootPose.pose.position.x = _desiredFootPosition[k](0);
    _msgDesiredFootPose.pose.position.y = _desiredFootPosition[k](1);
    _msgDesiredFootPose.pose.position.z = _desiredFootPosition[k](2);
    _msgDesiredFootPose.pose.orientation.x = 0.0f;
    _msgDesiredFootPose.pose.orientation.y = 0.0f;
    _msgDesiredFootPose.pose.orientation.z = 0.0f;
    _msgDesiredFootPose.pose.orientation.w = 1.0f;
    _pubDesiredFootPose[k].publish(_msgDesiredFootPose);

    for(int m = 0; m < 5; m++)
    {
      _msgFootInput.ros_position[m] = 0.0f;
      _msgFootInput.ros_speed[m] = 0.0f;
      _msgFootInput.ros_effort[m] = _desiredFootWrench[k](m);
      _msgFootInput.ros_effortM[m] = 0.0f;
      _msgFootInput.ros_filterAxisForce[m] = 1.0f;
      _msgFootInput.ros_kp[m] = 0.0f;
      _msgFootInput.ros_kd[m] = 0.0f;
      _msgFootInput.ros_ki[m] = 0.0f;
    }
    _pubFootInput[k].publish(_msgFootInput);
  }
}


void FootControl::footDataTransformation()
{
  _footPosition[RIGHT](0) = _footPose[RIGHT](0);
  _footPosition[RIGHT](1) = _footPose[RIGHT](1);
  _footPosition[RIGHT](2) = _footPose[RIGHT](2);
  _footPosition[LEFT](0) = _footPose[LEFT](0);
  _footPosition[LEFT](1) = _footPose[LEFT](1);
  _footPosition[LEFT](2) = _footPose[LEFT](2);

}

void FootControl::positionPositionMapping()
{

  Eigen::Vector3f gains[2];
  gains[RIGHT] << _xyPositionMapping/FOOT_INTERFACE_Y_RANGE_RIGHT, _xyPositionMapping/FOOT_INTERFACE_X_RANGE_RIGHT, _zPositionMapping/FOOT_INTERFACE_PHI_RANGE_RIGHT;
  gains[LEFT] << _xyPositionMapping/FOOT_INTERFACE_Y_RANGE_LEFT, _xyPositionMapping/FOOT_INTERFACE_X_RANGE_LEFT, _zPositionMapping/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
  for(int k = 0; k < 2; k++)
  {
    _desiredFootPosition[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_desiredFootPosition[k](2)<0.0f)
    {
      _desiredFootPosition[k](2) = 0.0f;
    }
    // std::cerr << "Foot " << k << " : " <<_desiredFootPosition[k].transpose() << std::endl;
  }
}


void FootControl::computeDesiredFootWrench()
{
  for(int k = 0 ; k < 2; k++)
  {
    _filteredForce[k] = 0.7f*_filteredForce[k]+0.3f*_ftSensorForce[k];
  }

  Eigen::Vector3f temp;

  temp.setConstant(0.0f);
  temp(1) = -_force[RIGHT](0)*1.0f;
  // temp(1) = -_filteredForce[RIGHT](0)/30.0f;
  // temp(0) = _filteredForce[RIGHT](1)/30.0f;
  // temp(2) = _filteredForce[RIGHT](2)/30.0f;

  // temp.setConstant(0.0f);
  _desiredFootWrench[RIGHT](1) = temp(0);
  _desiredFootWrench[RIGHT](0) = -temp(1);
  _desiredFootWrench[RIGHT](2) = temp(2)*0.205/5;


  temp.setConstant(0.0f);
  temp(1) = _force[LEFT](0)*1.0f; 
  // temp(1) = -_filteredForce[LEFT](0)/30.0f;
  // temp(0) = _filteredForce[LEFT](1)/30.0f;
  // temp(2) = _filteredForce[LEFT](2)/30.0f;

 
  _desiredFootWrench[LEFT](1) = temp(0);
  _desiredFootWrench[LEFT](0) = -temp(1);
  _desiredFootWrench[LEFT](2) = temp(2)*0.205/5;


  for(int k = 0 ; k < 2; k++)
  {
    if(_desiredFootWrench[RIGHT](k)>25.0f)
    {
      _desiredFootWrench[RIGHT](k) = 25.0f;
    }
    else if(_desiredFootWrench[RIGHT](k)<-25.0f)
    {
      _desiredFootWrench[RIGHT](k) = -25.0f;
    }

    if(_desiredFootWrench[LEFT](k)>25.0f)
    {
      _desiredFootWrench[LEFT](k) = 25.0f;
    }
    else if(_desiredFootWrench[LEFT](k)<-25.0f)
    {
      _desiredFootWrench[LEFT](k) = -25.0f;
    }
  }

  for(int k = 0 ; k < 3; k++)
  {
    if(_desiredFootWrench[RIGHT](k+2)>0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+2) = 0.187f*40/9.15;
    }
    else if(_desiredFootWrench[RIGHT](k+2)<-0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+2) = -0.187f*40/9.15;
    }

    if(_desiredFootWrench[LEFT](k+2)>0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+2) = 0.212f*40/9.15;
    }
    else if(_desiredFootWrench[LEFT](k+2)<-0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+2) = -0.212f*40/9.15;
    }

  }

  // float dampingGain = 1.0f;
  //   _desiredFootWrench[LEFT](0)+=-dampingGain*_footTwist[LEFT](0);
  //   _desiredFootWrench[RIGHT](0)+=-dampingGain*_footTwist[RIGHT](0);

  //   _desiredFootWrench[LEFT](1)+=-dampingGain*_footTwist[LEFT](1);
  //   _desiredFootWrench[RIGHT](1)+=-dampingGain*_footTwist[RIGHT](1);

  //   _desiredFootWrench[LEFT](3)+=-0.0001*_footTwist[LEFT](3);
  //   _desiredFootWrench[RIGHT](3)+=-0.0001*_footTwist[RIGHT](3);
}

void FootControl::updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k)
{

  for(int m = 0; m < 5; m++)
  {
    _footPose[k](m) = msg->platform_position[m];
    _footWrench[k](m) = msg->platform_effortM[m];
    _footTwist[k](m) = msg->platform_speed[m];
  }
  _footState[k] = msg->platform_machineState;

  if(!_firstFootOutput[k])
  {
    _firstFootOutput[k] = true;
  }
}

void FootControl::updateForce(const geometry_msgs::Vector3::ConstPtr& msg, int k)
{
  _force[k] << msg->x, msg->y, msg->z;
  if(!_firstForce[k])
  {
    _firstForce[k] = true;
  }
}


void FootControl::updateFtSensorForce(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  _ftSensorForce[k] << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  if(!_firstForce[k])
  {
    _firstForce[k] = true;
  }
}
