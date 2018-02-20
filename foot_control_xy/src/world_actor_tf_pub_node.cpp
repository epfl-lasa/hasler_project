#include "foot_control_xy/world_actor_tf_pub.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worldActorTransform");
   if (argc != 2){ROS_ERROR("need the actor name (chaser/target) as argument"); return -1;};
   std::string name = argv[1];
  
  ros::NodeHandle n;
  float frequency = 500.0f;

  Actor actor(n,frequency,name);

  if (!actor.init()) 
  {
    return -1;
  }
  else
  {
    actor.run();
  }
  
  return 0;

}
