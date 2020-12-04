#ifndef __REACH_TARGET_H__
#define __REACH_TARGET_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/PoseStamped.h"
#include <signal.h>
#include <Eigen/Eigen>
#include <string>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <fstream>
#include <ros/package.h>


class ReachTarget
{
	private:
	
		enum FOOT {LEFT = 0, RIGHT = 1};
		
		//! Ros variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		//! Subscribers and publishers
		ros::Subscriber _subFootPose[2];
		ros::Subscriber _subCubePose;
		ros::Subscriber _subForce[2];
		ros::Publisher _pubTargetPose;
		ros::Publisher _pubCurrentTime;

		//! Ros messages
		geometry_msgs::PoseStamped _msgTargetPose;
    Eigen::Vector3f _targetPosition;

    tf::TransformListener _lr;
    tf::StampedTransform _transform;


		//! Boolean variables
		bool _stop;
		bool _firstFootPosition[2];
		bool _firstCubePose;
		bool _cubeGrasped;
		bool _targetReached;
    bool _firstForce[2];

    double _startingTime;		
    double _currentTime;
    double _initialTime;
    double _reachedTime;



		Eigen::Vector3f _footPosition[2];
		Eigen::Vector3f _cubePosition;
		Eigen::Vector4f _cubeOrientation;
    Eigen::Vector3f _force[2];


		//! Other variables
		std::string _filename;
		std::ofstream _outputFile;
  	std::mutex _mutex;
		static ReachTarget* me;

    
	public:
		ReachTarget(ros::NodeHandle &n, double frequency, std::string filename);
		bool  init();
		void run();
	
	private:
		static void stopNode(int sig);
		void receiveFrames();
		void logData();
		void publishData();
		void checkIfObjectGrasped();
		void checkIfTargetReached();
		void updateFootPose(const geometry_msgs::Pose::ConstPtr& msg, int k);
		void updateCubePose(const geometry_msgs::Pose::ConstPtr& msg);
		void updateForce(const geometry_msgs::Vector3::ConstPtr& msg, int k);


};
#endif  // __REACH_TARGET_H__
