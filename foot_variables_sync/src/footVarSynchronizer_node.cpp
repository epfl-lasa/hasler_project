#include "footVarSynchronizer.h"

int main(int argc, char **argv)
{
  footVarSynchronizer::Platform_Name platform_id_;

  std::string platform_name;
  
  ros::init(argc, argv, "VarSync");

  ros::NodeHandle nh_("~");

  nh_.getParam("platformID", platform_name);

  if (platform_name.compare("right") == 0) {
    platform_id_ = footVarSynchronizer::RIGHT;
  } else if (platform_name.compare("left") == 0) {
    platform_id_ = footVarSynchronizer::LEFT;
  } else {
    ROS_ERROR("You didn't enter a platformID left or right");
    return -1;
  }

  float frequency = 300.0f;
  footVarSynchronizer footVarSynchronizer(nh_,frequency,platform_id_);  

  

  if (!footVarSynchronizer.init()) 
  {
    return -1;
  }
  else
  {
    footVarSynchronizer.run();
  }
  return 0;
}
