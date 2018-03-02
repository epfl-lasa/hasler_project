#include "foot_control_xy/Protocol.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "protocol");
  ros::NodeHandle n;
  float frequency = 100.0f;


  Eigen::Vector3f initTargetPosition;
  initTargetPosition.setConstant(0.0f);
  if (argc < 2)
  {
    ROS_ERROR("You are missing arguments: (Position) -x # -y # -z");
    return 0;
  }
  else 
  {
    initTargetPosition << atof(argv[2]),atof(argv[4]),atof(argv[6]);
  }

  Protocol protocol(n,frequency,initTargetPosition);

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
