#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredPose.h"


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "move_to_desired_pose");

  Eigen::Vector3f desiredPosition;

  // Initialize desired position
  desiredPosition.setConstant(0.0f);

  MoveToDesiredPose::Mode mode;

  // Check if desired position is specified with the command line
  if(argc == 4)
  {
    for(int k = 0; k < 3; k++)
    {
      desiredPosition(k) = atof(argv[k+1]);
    }
    mode = MoveToDesiredPose::Mode::SINGLE_RIGHT;
  }
  else if(argc == 6)
  {
    for(int k = 0; k < 3; k++)
    {
      desiredPosition(k) = atof(argv[k+1]);
    }

    if(std::string(argv[4]) == "-m" && std::string(argv[5]) == "l")
    {
      mode = MoveToDesiredPose::Mode::SINGLE_LEFT;
    }
    else if(std::string(argv[4]) == "-m" && std::string(argv[5]) == "r")
    {
      mode = MoveToDesiredPose::Mode::SINGLE_RIGHT;
    }
    else if(std::string(argv[4]) == "-m" && std::string(argv[5]) == "b")
    {
      mode = MoveToDesiredPose::Mode::BOTH;
    }
    else
    {
      ROS_ERROR("Wrong mode arguments, the command line arguments should be: p1 p2 p3 -m(mode) l(single left)/r(single right)/b(both)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Wrong number of arguments, the command line arguments should be: p1 p2 p3 -m(mode) l(single left)/r(single right)/b(both)");
    return 0;
  }

  std::cerr << desiredPosition.transpose() << std::endl;
  std::cerr << (int) MoveToDesiredPose::Mode::BOTH << std::endl;
  ros::NodeHandle n;
  float frequency = 200.0f;

  MoveToDesiredPose moveToDesiredPose(n,frequency,mode);

  if (!moveToDesiredPose.init()) 
  {
    return -1;
  }
  else
  {
    moveToDesiredPose.setDesiredPose(desiredPosition);
    moveToDesiredPose.run();
  }

  return 0;

}
