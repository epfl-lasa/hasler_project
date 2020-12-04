#include "foot_control_xy/world_actor_tf_pub.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "worldActorTransform");
   if (argc < 2){ROS_ERROR("You are missing arguments: 1. actor name 2. (Position) -x # -y # -z # 3. (Orientation) -qx # -qy # -qz # -qw # "); return -1;};
   std::string name = argv[1];
   geometry_msgs::Pose init_pose;
   init_pose.position.x = atof(argv [3]);
   init_pose.position.y = atof(argv [5]);
   init_pose.position.z = atof(argv [7]);

   init_pose.orientation.x = atof(argv [9]);
   init_pose.orientation.y = atof(argv [11]);
   init_pose.orientation.z = atof(argv [13]);
   init_pose.orientation.w = atof(argv [15]);
   
  ros::NodeHandle n;
  float frequency = 500.0f;

  Actor actor(n,frequency,name,init_pose);

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
