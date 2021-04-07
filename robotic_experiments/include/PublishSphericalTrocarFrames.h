#ifndef __PUBLISH_SPHERICAL_TROCAR_FRAMES_H__
#define __PUBLISH_SPHERICAL_TROCAR_FRAMES_H__

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
#include <dynamic_reconfigure/server.h>
#include "surgical_simulator/publishTrocarFrame_paramsConfig.h"
#include "custom_msgs/FootInputMsg.h"
#include "custom_msgs/FootOutputMsg.h"


using namespace std;

class PublishSphericalTrocarFrames
{
	private:
	
	//!ros variables

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

	//!subscribers and publishers declaration    
    //!boolean variables
    
    bool _stop;
    static PublishSphericalTrocarFrames* me;

    Eigen::MatrixXf _trocarPosition;
    Eigen::MatrixXf _trocarOrientation;

    Eigen::Vector3f _offset;

            
	public:
	PublishSphericalTrocarFrames(ros::NodeHandle &n, double frequency);
	~PublishSphericalTrocarFrames();
	bool  init();
	void run();
	void updateTf();
	
	private:

	tf::TransformBroadcaster _br;
	// tf::Transform _transform;
    tf::StampedTransform _transform;

	static void stopNode(int sig);
};
#endif  // __PUBLISH_SPHERICAL_TROCAR_FRAMES_H__
