#ifndef __JOY_CONTROL_H__
#define __JOY_CONTROL_H__

#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <tf/tf.h>
#include <mutex>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Path.h"
#include <signal.h>
#include <Eigen/Eigen>



class JoyControl
{
	private:
	

		//! Ros variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		//! Subscribers and publishers
		ros::Subscriber _subJoy;
		ros::Subscriber _subChaserPose;          
		ros::Publisher _pubChaserPose;

		//! Ros messages
		geometry_msgs::PoseStamped _msgChaserPose; 

		//! JoyControl variables
		Eigen::Vector3f _chaserPosition;
		Eigen::Vector3f _chaserVelocity;

		//! Boolean variables
		bool _stop;
        bool _firstChaserPoseReceived;
        bool _firstChaserVelocityReceived;
		//! Other variables

        double _scale;
        int _axeX;
        int _axeY;
        int _axeZ;



    std::mutex _mutex;
		static JoyControl* me;
    
	public:
		JoyControl(ros::NodeHandle &n, double frequency);
		bool  init();
		void run();
	
	private:
		void publishData();
		static void stopNode(int sig);
		void updateChaserPose(const sensor_msgs::Joy::ConstPtr& joy);


};
#endif  // __JOY_CONTROL_H__
