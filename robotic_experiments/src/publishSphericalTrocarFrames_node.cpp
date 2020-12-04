#include "PublishSphericalTrocarFrames.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_spherical_trocar_frames");
  
  Eigen::Vector3f trocarOffset;
  std::string name;
  
  ros::NodeHandle n;
  float frequency = 200.0f;

  PublishSphericalTrocarFrames publishSphericalTrocarFrames(n,frequency);

  if (!publishSphericalTrocarFrames.init()) 
  {
    return -1;
  }
  else
  {
    publishSphericalTrocarFrames.run();
  }
  return 0;
}
