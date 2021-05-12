#include "SurgicalTask.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surgical_task");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
 
  std::string filename = "test";

  // if(argc==2)
  // {
  //   filename = std::string(argv[1]);
  // }
  // else
  // {
  //   return -1;
  // }

  SurgicalTask surgicalTask(n,frequency, filename);

  if (!surgicalTask.init()) 
  {
    return -1;
  }
  else
  {
   
    surgicalTask.run();
  }

  return 0;
}

