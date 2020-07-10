#include "legRobot.h"

int main(int argc, char **argv) {
  legRobot::Leg_Name leg_id_;

  ros::init(argc, argv, "legInverseKinematics");

  std::string leg_name;

  ros::NodeHandle nh_("~");
  nh_.getParam("legID", leg_name);

  if (leg_name.compare("right") == 0) {
    leg_id_ = legRobot::RIGHT;
  } else if (leg_name.compare("left") == 0) {
    leg_id_ = legRobot::LEFT;
  } else {
    ROS_ERROR("You didn't enter a left or right");
    return -1;
  }

  float frequency = 400.0f;
  legRobot legRobot(nh_, frequency, leg_id_);

  if (!legRobot.init()) {
    return -1;
  } else {
    legRobot.run();
  }
  return 0;
}
