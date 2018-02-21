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
	
		struct TargetInfo
		{
			Eigen::Vector3f position;
			float elapsedTime;
		};

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
		std::vector<TargetInfo> _targetInfo;

		//! Boolean variables
		bool _stop;
    bool _firstChaserPoseReceived;
    bool _targetReached;

		//! Other variables
    double _tInit;


    std::mutex _mutex;
		static Protocol* me;
    
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
