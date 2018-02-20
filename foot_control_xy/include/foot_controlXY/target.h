#ifndef TARGET_HH
#define TARGET_HH

# include <ros/ros.h>
# include <boost/shared_ptr.hpp>
# include <tf/tf.h>
# include <urdf/model.h> //! Very important
# include <tf2_ros/static_transform_broadcaster.h>
# include <tf2_ros/transform_broadcaster.h>
# include <kdl/frames.hpp>
# include <kdl/segments.hpp>
# include <kdl/tree>

//! #include <geometry_msgs/Twist.h>
//! #include <nav_msgs/OccupancyGrid.h>
//! #include <nav_msgs/Odometry.h>
//! #include <ros/advertise_options.h>
//! #include <ros/callback_queue.h>

# define PI 3.14159265

namespace foot_control_xy {

class Target
{
public :
	Target (const ros::NodeHandle& nh){};
//	bool update (  double dt ); //! Teleport, etc	
//	void translate 
	~Target(){};
private:
    bool teleportAbsoluteCallback(foot_control_xy::TeleportAbsolute::Request&, foot_control_xy::TeleportAbsolute::Response&); //! function for the service of teleportation
    ros::NodeHandel nh_;
    
    ros::Subscriber velocity_sub_;
    ros::Publisher poser_pub_;
    ros::ServiceServer teleport_relative_srv_;
    
    ros::WallTime last_commanded_time_;
   
};

typedef boost::shared_ptr<Target> TargetPtr;

}

#endif 
