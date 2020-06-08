#ifndef __SPHERICAL_TROCAR_FRAMES_H__
#define __SPHERICAL_TROCAR_FRAMES_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <signal.h>
#include "Eigen/Eigen"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Float32MultiArray.h>


using namespace std;

class SphericalTrocarFrames
{
	private:
	
    ros::NodeHandle _nh;
    ros::Rate _loopRate;
    float _dt;

    std_msgs::Float32MultiArray _msgFrames;
    ros::Publisher _pubFrames;


    
    bool _stop;
    static SphericalTrocarFrames* me;

    Eigen::MatrixXf _trocarPosition;
    Eigen::MatrixXf _trocarOrientation;

    Eigen::Vector3f _sphereCenter;

    int _publishTransforms;

            
	public:
	SphericalTrocarFrames(ros::NodeHandle &n, double frequency, Eigen::Vector3f sphereCenter, int publishTransforms);
	~SphericalTrocarFrames();
	bool  init();
	void run();
	void updateTf();
	
	private:

	tf::TransformBroadcaster _br;
	// tf::Transform _transform;
    tf::StampedTransform _transform;

	static void stopNode(int sig);
};
#endif  // __SPHERICAL_TROCAR_FRAMES_H__
