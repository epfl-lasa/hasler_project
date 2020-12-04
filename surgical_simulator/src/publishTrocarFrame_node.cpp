#include "PublishTrocarFrame.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_trocar_frame");
  
  Eigen::Vector3f trocarOffset;
  std::string name;
  
  if (argc == 8)
  {
    name = argv[1];
    trocarOffset(0) = atof(argv [3]);
    trocarOffset(1) = atof(argv [5]);
    trocarOffset(2) = atof(argv [7]);
  }
  else
  { 
      ROS_ERROR("Wrong input arguments: name -x # -y # -z # 3. (Orientation) -qx # -qy # -qz # -qw # "); 
      return -1;
  }
   
  ros::NodeHandle n;
  float frequency = 200.0f;

  PublishTrocarFrame publishTrocarFrame(n,frequency,name,trocarOffset);

  if (!publishTrocarFrame.init()) 
  {
    return -1;
  }
  else
  {
    publishTrocarFrame.run();
  }
  return 0;
}
