#include "build_tower.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "build_tower");
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

  BuildTower buildTower(n,frequency,filename);

  if (!buildTower.init()) 
  {
    return -1;
  }
  else
  {
    buildTower.run();
  }

  return 0;
}