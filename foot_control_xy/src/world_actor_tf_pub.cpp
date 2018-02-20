#include "foot_control_xy/world_actor_tf_pub.h"


Actor* Actor::me = NULL;

Actor::Actor(ros::NodeHandle &n, double frequency,std::string name): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_actor_name(name)
{
	me=this;
	_stop = false;
	
	_actorPose.position.x = 0.0f; //+ The original pose of the actor
	_actorPose.position.y = 0.0f;
	_actorPose.position.z = 0.0f;
	_actorPose.orientation.x = 0.0f;
	_actorPose.orientation.y = 0.0f;
	_actorPose.orientation.z = 0.0f;
	_actorPose.orientation.w = 1.0f;
}

Actor::~Actor(){ros::shutdown();}

bool Actor::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{
	
	//Subscriber definitions
	_actor_pose_sub = _n.subscribe(_actor_name + "/Pose",1,&Actor::poseCallback,this,ros::TransportHints().reliable().tcpNoDelay());
	
	
	signal(SIGINT,Actor::stopNode);
	
	if (_n.ok()) 
	{ 
   
		ros::spinOnce();
		ROS_INFO("The actor_world broadcast is about to start");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void Actor::stopNode(int sig)
{
    me->_stop= true;
}

void Actor::run()
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


void Actor::updateTf()
{
	tf::Vector3 origin(_actorPose.position.x, _actorPose.position.y, _actorPose.position.z);
    _transform.setOrigin(origin);
    _transform.setRotation(tf::Quaternion(_actorPose.orientation.x, _actorPose.orientation.y, _actorPose.orientation.z, _actorPose.orientation.w));
    _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world", _actor_name + "/base_link"));

}

void Actor::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	_actorPose = msg->pose;
}

