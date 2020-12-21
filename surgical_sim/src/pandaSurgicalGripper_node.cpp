#include "pandaSurgicalGripper.h"
#include <urdf/model.h>
#include "iostream"
using namespace std;

int main(int argc, char **argv) {

  pandaSurgicalGripper::Panda_Tool_Name tool_id_;

  ros::init(argc, argv, "pandaGripperStatePublisher");

  std::string tool_name;

  double frequency = 200.0f;

  ros::NodeHandle nh_("~");
  nh_.getParam("toolID", tool_name);
  nh_.getParam("publish_frequency", frequency);

    if (tool_name.compare("right") == 0) {
      tool_id_ = pandaSurgicalGripper::RIGHT;
    } else if (tool_name.compare("left") == 0) {
      tool_id_ = pandaSurgicalGripper::LEFT;
    } else {
      ROS_ERROR("You didn't enter a toolID left or right");
      return -1;
    }

    urdf::Model modelLoad;
    if (!modelLoad.initParam("robot_description")) {
      ROS_ERROR("Failed to parse urdf file");
      return -1;
    }
    ROS_INFO("Successfully parsed urdf file");

 
  pandaSurgicalGripper pandaSurgicalGripper(nh_, frequency, tool_id_, modelLoad);
  
  if (!pandaSurgicalGripper.init()) {
    
    return -1;
  } else {
   pandaSurgicalGripper.run();
  }
  return 0;
}
