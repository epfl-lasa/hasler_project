#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <signal.h>
#include <Eigen/Eigen>

#define TARGET_TOLERANCE 0.5

class Protocol
{
	private:
	
		//! Ros variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		//! Subscribers and publishers
		ros::Subscriber _subChaserPose;          
		ros::Publisher _pubTargetPose;

		//! Ros messages
		geometry_msgs::PoseStamped _msgTargetPose; 

		//! Protocol variables
		Eigen::Vector3f _chaserPosition;
		Eigen::Vector3f _targetPosition;

		//! Boolean variables
		bool _stop;
    bool _firstChaserPoseReceived;
    bool _targetReached;

		//! Other variables
		geometry_msgs::Pose _actorPose;
		std::string _actorName;
		static Protocol* me;

    std::mutex _mutex;
    
	public:
		Protocol(ros::NodeHandle &n, double frequency, Eigen::Vector3f initTargetPosition);
		bool  init();
		void run();
	
	private:
	
		static void stopNode(int sig);

		void checkIfTargetReached();

		void updateTargetPose();

		void publishData();

	  void updateChaserPose(const geometry_msgs::PoseStampedConstPtr& msg);



};
#endif  // __PROTOCOL_H__
