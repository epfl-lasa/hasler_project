#include "foot_control_xy/joy_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "JoyControl");
  ros::NodeHandle n;
  float frequency = 500.0f;

  JoyControl joyControl(n,frequency);

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
