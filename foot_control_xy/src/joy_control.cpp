#include "foot_control_xy/joy_control.h"


JoyControl* JoyControl::me = NULL;

JoyControl::JoyControl(ros::NodeHandle &n, double frequency): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_scale(2.0f),
_axeX(0),
_axeY(1),
_axeZ(2)
{
  me=this;
  _stop = false;
  _firstChaserPoseReceived = false;
  _chaserPosition.setConstant(0.0f);
  _n.param("axis_x",_axeX);
  _n.param("axis_y",_axeY);
  _n.param("axis_z",_axeZ);
  _n.param("scale_",_scale);
}

bool JoyControl::init()
{
  
  //Subscriber definitions
  _subChaserPose = _n.subscribe("joy",10,&JoyControl::updateChaserPose,this,ros::TransportHints().reliable().tcpNoDelay());
  
  //Publisher definitions
  _pubChaserPose = _n.advertise<geometry_msgs::PoseStamped>("chaser/pose", 1);

  signal(SIGINT,JoyControl::stopNode);
  
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


void JoyControl::stopNode(int sig)
{
    me->_stop= true;
}


void JoyControl::run()
{
  while (!_stop) 
  {
    
    if(_firstChaserPoseReceived)
    {
      _mutex.lock();

      publishData();
      
      _mutex.unlock();
    }

    ros::spinOnce();
    _loopRate.sleep();
  }
  
  ros::shutdown();

}

void JoyControl::publishData()
{
  _chaserPosition+= _chaserVelocity*_dt;
  
  _msgChaserPose.header.frame_id = "world";
  _msgChaserPose.header.stamp = ros::Time::now();
  _msgChaserPose.pose.position.x = _chaserPosition(0);
  _msgChaserPose.pose.position.y = _chaserPosition(1);
  _msgChaserPose.pose.position.z = _chaserPosition(2);
  _msgChaserPose.pose.orientation.x = 0.0f;
  _msgChaserPose.pose.orientation.y = 0.0f;
  _msgChaserPose.pose.orientation.z = 0.0f;
  _msgChaserPose.pose.orientation.w = 1.0f;
  _pubChaserPose.publish(_msgChaserPose);
}


void JoyControl::updateChaserPose(const sensor_msgs::Joy::ConstPtr& joy)
{
  _chaserVelocity << -joy->axes[_axeX], joy->axes[_axeY], joy->axes[_axeZ];
  _chaserVelocity*=2;
  

  if(!_firstChaserPoseReceived)
  {
    _firstChaserPoseReceived = true;
  }
}
