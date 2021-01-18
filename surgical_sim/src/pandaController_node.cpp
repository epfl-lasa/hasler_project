#include "pandaController.h"
#include <urdf/model.h>
#include "iostream"
using namespace std;

int main(int argc, char **argv) {

  pandaController::Panda_Name panda_id_;

  ros::init(argc, argv, "pandaGripperStatePublisher");

  std::string panda_name;

  double frequency = 100.0f;

  ros::NodeHandle nh_("~");
  nh_.getParam("pandaID", panda_name);
  nh_.getParam("publish_frequency", frequency);

    if (panda_name.compare("right") == 0) {
      panda_id_ = pandaController::RIGHT;
    } else if (panda_name.compare("left") == 0) {
      panda_id_ = pandaController::LEFT;
    } else {
      ROS_ERROR("You didn't enter a pandaID left or right");
      return -1;
    }

    urdf::Model modelLoad;
    if (!modelLoad.initParam("robot_description")) {
      ROS_ERROR("Failed to parse urdf file");
      return -1;
    }
    ROS_INFO("Successfully parsed urdf file");

 
  pandaController pandaController(nh_, frequency, panda_id_, modelLoad);
  
  if (!pandaController.init()) {
    
    return -1;
  } else {
   pandaController.run();
  }
  return 0;
}
