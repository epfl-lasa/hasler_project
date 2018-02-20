#ifndef TARGET_HH
#define TARGET_HH

# include <ros/ros.h>

namespace foot_control_xy {

class pose_generator
{
public :
	GeneneratePose (ros::NodeHandle& nh){};
//	bool update (  double dt ); //! Teleport, etc	
//	void translate 
	~GeneneratePose(){};
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
