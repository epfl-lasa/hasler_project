#include "KinectRecording.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinectRecording");
  ros::NodeHandle n;
  float frequency = 30.0f;

  std::string subjectName;
  KinectRecording::ExecutionMode executionMode;
  KinectRecording::FittingMethod fittingMethod;

  if(argc == 6)
  {
    subjectName = std::string(argv[1]);

    if(std::string(argv[3]) == "c")
    {
      executionMode = KinectRecording::ExecutionMode::CALIBRATION;
    }
    else if(std::string(argv[3]) == "g")
    {
      executionMode = KinectRecording::ExecutionMode::GAME;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Fitting method used for game) -f p(plane) or s(sphere)");
      return 0;
    }

    if(std::string(argv[5]) == "p")
    {
      fittingMethod = KinectRecording::FittingMethod::PLANE;
    }
    else if(std::string(argv[5]) == "s")
    {
      fittingMethod = KinectRecording::FittingMethod::SPHERE;
    }
    else
    {
      ROS_ERROR("Wrong fitting method arguments, the command line arguments should be: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Fitting method used for game) -f p(plane) or s(sphere)");
    }
  }
  else
  {
    ROS_ERROR("You are missing arguments: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Fitting method used for game) -f p(plane) or s(sphere)");
    return 0;  
  }

  KinectRecording experiment(n,frequency,subjectName,executionMode,fittingMethod);

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
