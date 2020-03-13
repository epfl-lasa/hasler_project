#include "KukaDemo.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kuka_demo");
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

  KukaDemo kukaDemo(n,frequency,filename);

  if (!kukaDemo.init()) 
  {
    return -1;
  }
  else
  {
   
    kukaDemo.run();
  }

  return 0;
}

