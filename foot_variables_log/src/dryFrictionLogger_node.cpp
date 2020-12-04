#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "dryFrictionLogger.h"


int main(int argc, char **argv)
{
  dryFrictionLogger::Platform_Name platform_id_;
  std::string filename_;
  Axis axis_;

  srand(time(NULL));

  if (argc>=2)
  {
    if (std::string(argv[1])=="left") {platform_id_=dryFrictionLogger::LEFT;}
    else if (std::string(argv[1])=="right") {platform_id_=dryFrictionLogger::RIGHT;}
    else
    {
      ROS_ERROR("You didn't enter a left or right");
      return -1;
    }

  if (argc >= 3) {

      try {
          std::cerr<<"Axis: "<< std::string(argv[2])<<endl;
          axis_ = (Axis) (atoi(argv[2]) - 1) ;
      }
      catch (int n) {
        ROS_ERROR("You didn't enter the axis number X=1, Y=2, PITCH=3, ROLL=4, "
                  "YAW=5");
        return -1;
      }
  }

    if (argc>=4)
    {
      
      try{
        if (std::string(argv[3])!="") 
        {
          filename_= std::string(argv[3]);
        }
        else {
          filename_ = std::string("no_file");
        }
      }
      catch(int n2)
        {filename_=std::string("no_file");}
    }
  }
  else
  {
    ROS_ERROR("You didn't enter a left/right the axis number and optionally the name of the log file");
    return -1;
  }

  ros::init(argc, argv, std::string(argv[1])+"VarLog");

  ros::NodeHandle nh_;

  float frequency = 173.0f;
  dryFrictionLogger dryFrictionLogger(nh_,frequency,platform_id_, axis_, filename_);  

  if (!dryFrictionLogger.init()) 
  {
    return -1;
  }
  else
  {
    dryFrictionLogger.run();
  }
  return 0;
}
