#include "foot_control_xy/Protocol.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "protocol");
  ros::NodeHandle n;
  float frequency = 100.0f;


  Eigen::Vector3f initTargetPosition;
  initTargetPosition.setConstant(0.0f);


  Protocol::Strategy strategy = Protocol::Strategy::DISCRETE;
  if (argc < 2)
  {
    ROS_ERROR("You are missing arguments: 1. (Strategy) -s d(discrete) or c(continuous)  2. (Position) -x # -y # -z");
    return 0;
  }
  else 
  {
    if(std::string(argv[2]) == "d")
    {
      strategy = Protocol::Strategy::DISCRETE;
    }
    else if(std::string(argv[2]) == "c")
    {
      strategy = Protocol::Strategy::CONTINUOUS;
    }

    initTargetPosition << atof(argv[4]),atof(argv[6]),atof(argv[8]);
  }

  Protocol protocol(n,frequency,initTargetPosition,strategy);

  if (!protocol.init()) 
  {
    return -1;
  }
  else
  {
    protocol.run();
  }

  return 0;
}
