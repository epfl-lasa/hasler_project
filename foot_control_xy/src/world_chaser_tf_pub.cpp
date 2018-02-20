#include "foot_control_xy/world_chaser_tf_pub.h"


Chaser* Chaser::me = NULL;

Chaser::Chaser(ros::NodeHandle &n, double frequency): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
{
	me=this;
	_stop = false;
	
	_chaserPose.position.x = 0.0f;
	_chaserPose.position.y = 0.0f;
	_chaserPose.position.z = 0.0f;
	
}

//Chaser::~Chaser(){rosnode_->shutdown();}

bool Chaser::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{
	
	//Subscriber definitions
	_chaser_pose_sub = _n.subscribe("/chaserPose",1,&Chaser::poseCallback,this,ros::TransportHints().reliable().tcpNoDelay());
	
	
	signal(SIGINT,Chaser::stopNode);
	
	if (_n.ok()) 
	{ 
   
		ros::spinOnce();
		ROS_INFO("The chaser2world broadcast is about to start");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void Chaser::stopNode(int sig)
{
    me->_stop= true;
}

void Chaser::run()
{
  while (!_stop) 
  {
	  
	updateTf();
    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void Chaser::updateTf()
{
	tf::Vector3 origin(_chaserPose.position.x, _chaserPose.position.y, 0.0);
    _transform.setOrigin(origin);
    _transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world", "chaser/base_footprint"));

}

void Chaser::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	_chaserPose = msg->pose;
}

