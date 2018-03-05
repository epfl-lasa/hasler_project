#include "foot_control_xy/Protocol.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "protocol");
  ros::NodeHandle n;
  float frequency = 100.0f;


  Eigen::Vector3f initTargetPosition;
  initTargetPosition.setConstant(0.0f);


  Protocol::Strategy strategy = Protocol::Strategy::DISCRETE;
  std::string subjectName;

  if (argc != 10)
  {
    ROS_ERROR("You are missing arguments: 1. subject name 2. (Strategy) -s d(discrete) or c(continuous)  3. (Position) -x # -y # -z #");
    return 0;
  }
  else 
  {
    subjectName = std::string(argv[1]);

    if(std::string(argv[3]) == "d")
    {
      strategy = Protocol::Strategy::DISCRETE;
    }
    else if(std::string(argv[3]) == "c")
    {
      strategy = Protocol::Strategy::CONTINUOUS;
    }

    initTargetPosition << atof(argv[5]),atof(argv[7]),atof(argv[9]);
  }

  Protocol protocol(n,frequency,initTargetPosition,strategy,subjectName);

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
