#include "foot_control_xy/target.h"
#include <geometry_msgs/PoseStamped.h>


using namespace std;
using namespace ros;

namespace foot_control_xy 
{
	Target::Target (const ros::NodeHandle& nh): nh_(nh){
		 //!velocity_sub_=nh_.subscribe("cmd_vel", 1, &Turtle::velocityCallback, this); in case we want to add moving objects
		 pose_pub_=nh_.advertise<PoseStamped>{"pose_stamped",1};
		 teleport_absolute_srv_ = nh_.advertiseService("teleport_absolute",&Target::teleportAbsoluteCallback, this);
	}
	
bool Target::teleportAbsoluteCallback(foot_control_xy::TeleportAbsolute::Request& req, foot_control_xy::TeleportAbsolute::Response&)
{
  teleport_requests_.push_back(TeleportRequest(req.x, req.y, req.theta, 0, false));
  return true;
}	

bool Target::update (double dt)
{
	bool modified=false;
}
	
