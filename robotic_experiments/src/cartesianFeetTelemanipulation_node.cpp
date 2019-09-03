#include "CartesianFeetTelemanipulation.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_feet_telemanipulation");
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

  CartesianFeetTelemanipulation cartesianfeetTelemanipulation(n,frequency,filename);

  if (!cartesianfeetTelemanipulation.init()) 
  {
    return -1;
  }
  else
  {
   
    cartesianfeetTelemanipulation.run();
  }

  return 0;
}

