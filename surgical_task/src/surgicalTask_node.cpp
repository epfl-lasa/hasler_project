#include "SurgicalTask.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surgical_task");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  
  SurgicalTask surgicalTask(n,frequency);

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

