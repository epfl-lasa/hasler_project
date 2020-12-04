/*!
 * @file	echo_node_template.cpp
 * @author	Philipp Leemann
 * @date	Jan, 2018
 */

#include "cosmo_ros_tools/Echo.hpp"
#include "any_measurements_ros/ConversionTraits.hpp"
#include "@SHM_HEADER@"

#include <@ROS_HEADER@>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << argv[0] << " <topic name>\n";
    exit(-1);
  }

  cosmo_ros_tools::Echo<@SHM_TYPE @, @ROS_TYPE @, any_measurements_ros::ConversionTraits> myEcho(argc, argv, argv[1]);
  myEcho.run();
  return 0;
}