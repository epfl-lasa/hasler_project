#include "TrocarFeetTelemanipulation.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trocar_feet_telemanipulation");
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

  TrocarFeetTelemanipulation trocarfeetTelemanipulation(n,frequency,filename);

  if (!trocarfeetTelemanipulation.init()) 
  {
    return -1;
  }
  else
  {
   
    trocarfeetTelemanipulation.run();
  }

  return 0;
}

