#include "PillarFrames.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pillar_frames");
  
  int publishTransforms = 0;
  
  if (argc == 2)
  {
    publishTransforms = atoi(argv[1]);

  }
  else
  { 
      ROS_ERROR("Wrong input arguments: publishTransforms"); 
      return -1;
  }
   
  ros::NodeHandle n;
  float frequency = 1.0f;

  PillarFrames pillarFrames(n, frequency, publishTransforms);

  if (!pillarFrames.init()) 
  {
    return -1;
  }
  else
  {
    pillarFrames.run();
  }
  return 0;
}
