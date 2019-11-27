#include <sstream>
#include "footVarLogger.h"


int main(int argc, char **argv)
{
  footVarLogger::Platform_Name platform_id_;
  std::string filename_;

  if (argc>=2)
  {
    if (std::string(argv[1])=="left") {platform_id_=footVarLogger::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=footVarLogger::RIGHT;}
  }

  if (argc>=2)
  {
    if (std::string(argv[2])!=" ") {filename_=std::string(argv[2]);}
    else {filename_=std::string("default_name");}
  }

  else
  {
    ROS_ERROR("You didn't enter a left or right");
    return -1;
  } 

  ros::init(argc, argv, std::string(argv[1])+"VarLog");

  ros::NodeHandle nh_;

  float frequency = 150.0f;
  footVarLogger footVarLogger(nh_,frequency,platform_id_, filename_);  

  if (!footVarLogger.init()) 
  {
    return -1;
  }
  else
  {
    footVarLogger.run();
  }
  return 0;
}
