#ifndef __PUBLISH_TROCAR_FRAME_H__
#define __PUBLISH_TROCAR_FRAME_H__

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
#include "Eigen/Eigen"
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace std;

class PublishTrocarFrame
{
	private:
	
	//!ros variables

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

	//!subscribers and publishers declaration    
    //!boolean variables
    
    bool _stop;
    static PublishTrocarFrame* me;
    
    
    
    //!other variables
    Eigen::Vector3f _trocarOffset;
    std::string _frameName;
    bool _getTorsoFrame;
    Eigen::Vector3f _torsoFrameOrigin;

    //std::mutex _mutex;
    //ros::WallTime _last_commanded_time;
    
	public:
	PublishTrocarFrame(ros::NodeHandle &n, double frequency, std::string name, Eigen::Vector3f trocarOffset);
	~PublishTrocarFrame();
	bool  init();
	void run();
	void updateTf();
	
	private:
	
	tf::TransformListener _lr;
	tf::TransformBroadcaster _br;
	// tf::Transform _transform;
    tf::StampedTransform _transform;

	static void stopNode(int sig);
};
#endif  // __PUBLISH_TROCAR_FRAME_H__
