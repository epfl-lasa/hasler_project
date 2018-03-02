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

#define TARGET_TOLERANCE_RADIUS 0.5
#define TARGET_ELAPSED_TIME 5
#define TARGET_TOLERANCE_TIME 1
#define SCENE_SIZE 5
#define MOVING_TARGET_VELOCITY 1
#define MIN_MOVING_TARGET_DURATION 3.0
#define MAX_MOVING_TARGET_DURATION 6.0

class Protocol
{
	public:
		enum Strategy {DISCRETE = 0, CONTINUOUS = 1};
	
	private:
	
		enum DirectionID {PLUS_X = 0, MINUS_X = 1, PLUS_Y = 2, MINUS_Y = 3};

		struct TargetInfo
		{
			Eigen::Vector3f position;
			float elapsedTime;
			float accuracy;
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
		Strategy _strategy;

		//! Boolean variables
		bool _stop;
    bool _firstChaserPoseReceived;
    bool _targetReached;
    bool _setTargetToHome;

		//! Other variables
    double _currentTime;
    double _initialTime;
    double _reachedTime;
    double _duration;

    DirectionID _targetDirectionID;


    std::mutex _mutex;
		static Protocol* me;
		
	public:
    
		Protocol(ros::NodeHandle &n, double frequency, Eigen::Vector3f initTargetPosition, Strategy strategy);
		bool  init();
		void run();
	
	private:
	
		static void stopNode(int sig);

		void checkIfTargetReached();

		void updateTargetPose();

		void generateMovingTarget();

		Eigen::Vector3f getTargetDirection(int directionID);

		bool checkIfCollisionWithBoundaries(Eigen::Vector3f position, int directionID);

		bool checkIfSameDirection(int currentDirectionID, int newDirectionID);

		bool checkIfOppositeDirection(int currentDirectionID, int newDirectionID);


		void publishData();

	  void updateChaserPose(const geometry_msgs::PoseStampedConstPtr& msg);



};
#endif  // __PROTOCOL_H__
