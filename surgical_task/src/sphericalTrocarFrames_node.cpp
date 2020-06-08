#include "SphericalTrocarFrames.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sphericalTrocarFrames");
  
  Eigen::Vector3f sphereCenter;
  int publishTransforms = 0;
  
  if (argc == 5)
  {
    sphereCenter(0) = atof(argv[1]);
    sphereCenter(1) = atof(argv[2]);
    sphereCenter(2) = atof(argv[3]);
    publishTransforms = atoi(argv[4]);

  }
  else
  { 
      ROS_ERROR("Wrong input arguments: xc yc zc publishTransforms"); 
      return -1;
  }
   
  ros::NodeHandle n;
  float frequency = 1.0f;

  SphericalTrocarFrames sphericalTrocarFrames(n,frequency,sphereCenter,publishTransforms);

  if (!sphericalTrocarFrames.init()) 
  {
    return -1;
  }
  else
  {
    sphericalTrocarFrames.run();
  }
  return 0;
}
