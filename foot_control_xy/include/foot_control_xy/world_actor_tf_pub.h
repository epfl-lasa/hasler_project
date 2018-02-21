#ifndef __WORLD_ACTOR_TF_PUB_H__
#define __WORLD_ACTOR_TF_PUB_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Path.h"
#include <signal.h>

using namespace std;

class Actor
{
	private:
	
	//!ros variables

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

	//!subscribers and publishers declaration

    ros::Subscriber _actor_pose_sub;          
    
    //!boolean variables
    
    bool _stop;
    static Actor* me;
    
    
    
    //!other variables
    geometry_msgs::Pose _actorPose;
    std::string _actorName;

    //std::mutex _mutex;
    //ros::WallTime _last_commanded_time;
    
	public:
	Actor(ros::NodeHandle &n, double frequency, std::string name, geometry_msgs::Pose init_pose);
	~Actor();
	bool  init();
	void run();
	void updateTf();
	
	private:
	
	tf::TransformBroadcaster _br;
	tf::Transform _transform;

	static void stopNode(int sig);
	void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};
#endif  // __WORLD_ACTOR_TF_PUB_H__
