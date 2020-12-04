#include "reach_target.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reach_target");
  ros::NodeHandle n;
  float frequency = 200.0f;

  // Eigen::Vector3f targetPosition;
  std::string filename;

  if(argc==2)
  {
    filename = std::string(argv[1]);
    // targetPosition << atof(argv[2]),atof(argv[3]),atof(argv[4]);
  }
  else
  {
    return -1;
  }

  ReachTarget reachTarget(n,frequency,filename);

  if (!reachTarget.init()) 
  {
    return -1;
  }
  else
  {
    reachTarget.run();
  }

  return 0;
}