#include "foot_control_xy/world_chaser_tf_pub.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worldChaserTransform");
  ros::NodeHandle n;
  float frequency = 500.0f;

  Chaser foot(n,frequency);

  if (!foot.init()) 
  {
    return -1;
  }
  else
  {
    foot.run();
  }
  
  return 0;

}
