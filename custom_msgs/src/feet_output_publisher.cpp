#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include "custom_msgs/FootOutputMsg.h"
#include "custom_msgs/FeetOutputMsg.h"

#define LEFT 0
#define RIGHT 1

custom_msgs::FootOutputMsg footOutput[2];

void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k)
{
	footOutput[k] = *msg;
} 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "feet_output_publisher");
	ros::NodeHandle n;
	ros::Rate loopRate(200);	
	
	ros::Subscriber subFootOutput[2];
  subFootOutput[RIGHT] = n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(updateFootOutput,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  subFootOutput[LEFT] = n.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(updateFootOutput,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
 	ros::Publisher pubFeetOutput = n.advertise<custom_msgs::FeetOutputMsg>("/FeetOutput", 1);

  custom_msgs::FeetOutputMsg msg;
	while (ros::ok())
	{
		msg.stamp = ros::Time::now();
		msg.leftFoot = footOutput[LEFT];
		msg.rightFoot = footOutput[RIGHT];
	  pubFeetOutput.publish(msg);
	  ros::spinOnce();
	  loopRate.sleep();
	}
	return 0;

}


