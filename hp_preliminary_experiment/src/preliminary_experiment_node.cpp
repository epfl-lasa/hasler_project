#include "PreliminaryExperiment.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "preliminaryExperiment");
  ros::NodeHandle n;
  float frequency = 500.0f;

  std::string subjectName;
  PreliminaryExperiment::ExecutionMode executionMode;
  PreliminaryExperiment::TrackingMode trackingMode;
  PreliminaryExperiment::FittingMethod fittingMethod;


  if(argc == 8)
  {
    subjectName = std::string(argv[1]);

    if(std::string(argv[3]) == "c")
    {
      executionMode = PreliminaryExperiment::ExecutionMode::CALIBRATION;
    }
    else if(std::string(argv[3]) == "g")
    {
      executionMode = PreliminaryExperiment::ExecutionMode::GAME;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Tracking mode) t(toe only) or a(all joints) -t 4. (Fitting method used for game) -f p(plane) or s(sphere)");
      return 0;
    }

    if(std::string(argv[5]) == "t")
    {
      trackingMode = PreliminaryExperiment::TrackingMode::TOE_ONLY;
    }
    else if(std::string(argv[5]) == "a")
    {
      trackingMode = PreliminaryExperiment::TrackingMode::ALL_JOINTS;
    }
    else
    {
      ROS_ERROR("Wrong fitting method arguments, the command line arguments should be: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Tracking mode) t(toe only) or a(all joints) -t 4. (Fitting method used for game) -f p(plane) or s(sphere)");
    }

    if(std::string(argv[7]) == "p")
    {
      fittingMethod = PreliminaryExperiment::FittingMethod::PLANE;
    }
    else if(std::string(argv[7]) == "s")
    {
      fittingMethod = PreliminaryExperiment::FittingMethod::SPHERE;
    }
    else
    {
      ROS_ERROR("Wrong fitting method arguments, the command line arguments should be: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Tracking mode) t(toe only) or a(all joints) -t 4. (Fitting method used for game) -f p(plane) or s(sphere)");
    }
  }
  else
  {
    ROS_ERROR("You are missing arguments: 1. Subject name 2. (Execution Mode) -e c(calibration) or g(game) 3. (Tracking mode) t(toe only) or a(all joints) -t 4. (Fitting method used for game) -f p(plane) or s(sphere)");
    return 0;  
  }

  PreliminaryExperiment experiment(n,frequency,subjectName,executionMode,trackingMode,fittingMethod);

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
