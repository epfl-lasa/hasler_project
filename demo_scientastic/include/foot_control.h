#ifndef __FOOT_CONTROL_H__
#define __FOOT_CONTROL_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/PoseStamped.h"
#include <signal.h>
#include <Eigen/Eigen>
#include <string>
#include <geometry_msgs/Vector3.h>
#include "custom_msgs/FootOutputMsg.h"
#include "custom_msgs/FootInputMsg.h"



#define FOOT_INTERFACE_X_RANGE_RIGHT 0.350
#define FOOT_INTERFACE_Y_RANGE_RIGHT 0.293
#define FOOT_INTERFACE_PHI_RANGE_RIGHT 30
#define FOOT_INTERFACE_X_RANGE_LEFT 0.350
#define FOOT_INTERFACE_Y_RANGE_LEFT 0.293
#define FOOT_INTERFACE_PHI_RANGE_LEFT 30


class FootControl
{
	private:
	
		enum FOOT {LEFT = 0, RIGHT = 1};

		//! Ros variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		//! Subscribers and publishers
		ros::Subscriber _subForce[2];
		ros::Subscriber _subFootOutput[2];       
		ros::Publisher _pubFootInput[2]; 
		ros::Publisher _pubDesiredFootPose[2];


		//! Ros messages
		geometry_msgs::PoseStamped _msgDesiredFootPose;
		geometry_msgs::Vector3 _msgForce; 
		custom_msgs::FootInputMsg _msgFootInput;


		//! FootControl variables
    	Eigen::Matrix<float,6,1> _footPose[2];
    	Eigen::Matrix<float,6,1> _footSensor[2];
    	Eigen::Matrix<float,6,1> _footWrench[2];
		Eigen::Matrix<float,6,1> _footTwist[2];
	    Eigen::Matrix<float,6,1> _desiredFootWrench[2];
	    Eigen::Vector3f _footPosition[2];
	    Eigen::Vector3f _desiredFootPosition[2];
	    Eigen::Vector3f _force[2];
	    int _footState[2];
	    uint32_t _footInterfaceSequenceID[2];
	    float _xyPositionMapping;
		float _zPositionMapping;

		//! Boolean variables
		bool _stop;
        bool _firstFootOutput[2];
        bool _firstForce[2];
		//! Other variables

    	std::mutex _mutex;
		static FootControl* me;
		std::string _topicName;
    
	public:
		FootControl(ros::NodeHandle &n, double frequency);
		bool  init();
		void run();
	
	private:
		void publishData();
		static void stopNode(int sig);
		
		void footDataTransformation();
		void positionPositionMapping();
		void computeDesiredFootWrench();

		void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k); 
		void updateForce(const geometry_msgs::Vector3::ConstPtr& msg, int k);





};
#endif  // __FOOT_CONTROL_H__
