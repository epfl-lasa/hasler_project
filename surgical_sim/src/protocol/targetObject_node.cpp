#include "iostream"
#include "targetObject.h"

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "targetSpawner");

  double frequency = 100.0f;

  ros::NodeHandle nh_("~");
  std:: string target_name;
  
  nh_.getParam("targetName", target_name);
  nh_.getParam("publish_frequency", frequency);

  urdf::Model modelLoad;
  if (!modelLoad.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  targetObject targetObject(nh_, frequency, modelLoad, target_name);
  
  
  if (!targetObject.init()) 
  {
    return -1;
  } 
  else 
  {
    targetObject.run();
  }
  
  return 0;
}
