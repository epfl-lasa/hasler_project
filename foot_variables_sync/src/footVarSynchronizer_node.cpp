#include "footVarSynchronizer.h"

int main(int argc, char **argv)
{
  footVarSynchronizer::Platform_Name platform_id_;
  
  if (argc==2)
  {
    if (std::string(argv[1])=="left") {platform_id_=footVarSynchronizer::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=footVarSynchronizer::RIGHT;}
  }
  else
  {
    ROS_ERROR("You didn't enter a left or right");
    return -1;
  } 

  ros::init(argc, argv, std::string(argv[1])+"VarSync");

  // ros::NodeHandle nh_machine_state("~/machine_state");
  ros::NodeHandle nh_machine_state;

 

  
  float frequency = 500.0f;
  footVarSynchronizer footVarSynchronizer(nh_machine_state,frequency,platform_id_);  

  

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
