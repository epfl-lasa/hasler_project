#include "KinectRecording.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinectRecording");
  ros::NodeHandle n;
  float frequency = 30.0f;

  bool calibration = false;

  //If input argument is different > 0 do calibration else not do calibration
  if(argc == 2)
  {
    if(atoi(argv[1]))
    {
      calibration = true;
    }
    else
    {
      calibration = false;
    }
  }

  KinectRecording experiment(n,frequency,calibration);

  if (!experiment.init()) 
  {
    return -1;
  }
  else
  {
    experiment.run();
  }

  return 0;
}
