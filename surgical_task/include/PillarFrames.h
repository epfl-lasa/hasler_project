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

class PillarFrames
{
	private:
	
    ros::NodeHandle _nh;
    ros::Rate _loopRate;
    float _dt;

    std_msgs::Float32MultiArray _msgFrames;
    ros::Publisher _pubFrames;


    
    bool _stop;
    static PillarFrames* me;

    Eigen::MatrixXf _pillarPosition;

    Eigen::Vector3f _pillarsCenter;

    int _publishTransforms;

    bool _firstPillarsCenterFrame;

	tf::TransformBroadcaster _br;
	
	// tf::Transform _transform;
    tf::StampedTransform _transform;

    tf::TransformListener _lr;
            
	public:
	PillarFrames(ros::NodeHandle &n, double frequency, int publishTransforms);
	~PillarFrames();
	bool  init();

	void receiveFrames();


	void run();
	void updateTf();
	
	static void stopNode(int sig);
};
#endif  // __SPHERICAL_TROCAR_FRAMES_H__
