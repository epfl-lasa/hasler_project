#include "foot_control_xy/joy_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "JoyControl");
  ros::NodeHandle n;
  float frequency = 100.0f;

  std::string topicName;
  if(argc != 2)
  {
    return 0;
  }
  else
  {
    topicName = std::string(argv[1]);
  }
  JoyControl joyControl(n,frequency,topicName);

  if (!joyControl.init()) 
  {
    return -1;
  }
  else
  {
    joyControl.run();
  }

  return 0;
}
