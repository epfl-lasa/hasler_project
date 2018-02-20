#ifndef __WORLD_CHASER_TF_PUB_H__
#define __WORLD_CHASER_TF_PUB_H__

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


class Chaser
{
	private:
	
	//!ros variables

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

	//!subscribers and publishers declaration

    ros::Subscriber _chaser_pose_sub;          
    
    //!boolean variables
    
    bool _stop;
    static Chaser* me;
    
    geometry_msgs::Pose _chaserPose;
    
    //!other variables
    
    //std::mutex _mutex;
    //ros::WallTime _last_commanded_time;
    
	public:
	Chaser(ros::NodeHandle &n, double frequency);
	//~Chaser();
	bool  init();
	void run();
	void updateTf();
	
	private:
	
	tf::TransformBroadcaster _br;
	tf::Transform _transform;

	static void stopNode(int sig);
	void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};
#endif  // __WORLD_CHASER_TF_PUB_H__
