#include "legRobot.h"
#include <urdf/model.h>

int main(int argc, char **argv) {
  legRobot::Leg_Name leg_id_;

  ros::init(argc, argv, "legStatePublisher");

  std::string leg_name;
  std::string urdf_file;

  ros::NodeHandle nh_("~");
  nh_.getParam("legID", leg_name);
  nh_.getParam("urdfID", urdf_file);
  if (leg_name.compare("right") == 0) {
    leg_id_ = legRobot::RIGHT;
  } else if (leg_name.compare("left") == 0) {
    leg_id_ = legRobot::LEFT;
  } else {
    ROS_ERROR("You didn't enter a legID left or right");
    return -1;
  }

  urdf::Model modelLoad;
  if (!modelLoad.initFile(urdf_file)) {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  float frequency = 400.0f;
  legRobot legRobot(nh_, frequency, leg_id_, modelLoad);

  if (!legRobot.init()) {
    return -1;
  } else {
    legRobot.run();
  }
  return 0;
}
