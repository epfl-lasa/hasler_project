#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <signal.h>
#include <Eigen/Eigen>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include "foot_control_xy/protocol_paramsConfig.h"

class Protocol
{
	public:
		enum Strategy {DISCRETE = 0, CONTINUOUS = 1};
	
	private:
	
		enum DirectionID {PLUS_X = 0, MINUS_X = 1, PLUS_Y = 2, MINUS_Y = 3};

		struct DiscreteStrategyResult
		{
			Eigen::Vector3f targetPosition;
			int targetReached;
			float elapsedTime;
			float accuracy;
			float normalizedTrajectoryLength;
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
		Eigen::Vector3f _previousChaserPosition;
		Eigen::Vector3f _initialChaserPosition;
		Eigen::Vector3f _targetPosition;
    double _startingTime;		
    double _currentTime;
    double _initialTime;
    double _reachedTime;
    double _keepingDirectionTime;
		Strategy _strategy;
    DirectionID _targetDirectionID;
		std::vector<DiscreteStrategyResult> _dsResults;
		std::vector<float> _trackingError;
		float _trajectoryLength;
		float _tempMovingTargetBoundary;

		//! Boolean variables
		bool _stop;
    bool _firstChaserPoseReceived;
    bool _chaserReady;
    bool _targetReached;
    bool _setTargetToHome;
    bool _firstTarget;
    bool _timeout;

    //! Dynamic reconfigure variable
    float _discreteStrategyDuration;
		float _targetToleranceRadius;
		float _minTargetDistance;
		float _maxTargetDistance;
		float _targetElapsedTime;
		float _targetToleranceTime;
		float _continuousStrategyDuration;
		float _movingTargetVelocity;
		float _movingTargetBoundary;
		float _minKeepingDirectionTime;
		float _maxKeepingDirectionTime;

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<foot_control_xy::protocol_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<foot_control_xy::protocol_paramsConfig>::CallbackType _dynRecCallback;
  	foot_control_xy::protocol_paramsConfig _config;

		//! Other variables
		std::string _subjectName;
		uint32_t _sequenceID;
		uint32_t _previousSequenceID;
    std::ofstream _outputFile;   // File used to write raw experiment data and data result
    std::mutex _mutex;

		static Protocol* me;
		
	public:
    
		Protocol(ros::NodeHandle &n, double frequency, Eigen::Vector3f initTargetPosition, Strategy strategy, std::string subjectName);
		bool  init();
		void run();
	
	private:
	
		static void stopNode(int sig);

		bool isChaserReady();

		void checkIfTargetReached();

		void updateTargetPose();

		void generateMovingTarget();

		Eigen::Vector3f getTargetDirection(int directionID);

		bool checkIfCollisionWithBoundaries(Eigen::Vector3f position, int directionID);

		bool checkIfSameDirection(int currentDirectionID, int newDirectionID);

		bool checkIfOppositeDirection(int currentDirectionID, int newDirectionID);

		void logData();

		void checkTimeout();

		void logResult();

		void publishData();

	  void updateChaserPose(const geometry_msgs::PoseStampedConstPtr& msg);

    void dynamicReconfigureCallback(foot_control_xy::protocol_paramsConfig &config, uint32_t level);


};
#endif  // __PROTOCOL_H__
