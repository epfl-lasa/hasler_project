#include "SharedFourArmManipulation.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shared_four_arm_manipulation");
  ros::NodeHandle n;
  float frequency = 200.0f;
  std::string filename;
  if(argc==2)
  {
    filename = std::string(argv[1]);
  }
  else
  {
    return -1;
  }

  SharedFourArmManipulation sharedFourArmManipulation(n,frequency,filename);

  if (!sharedFourArmManipulation.init()) 
  {
    return -1;
  }
  else
  {
   
    sharedFourArmManipulation.run();
  }

  return 0;
}

