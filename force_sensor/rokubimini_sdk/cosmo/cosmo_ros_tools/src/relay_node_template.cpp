/*!
 * @file	relay_node_template.cpp
 * @author	Philipp Leemann
 * @date	Jan, 2018
 */

#include "cosmo_ros_tools/Relay.hpp"
#include "any_measurements_ros/ConversionTraits.hpp"
#include "@SHM_HEADER@"

#include <@ROS_HEADER@>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "Usage: " << argv[0] << " <input topic name> <output topic name>\n";
    exit(-1);
  }
  cosmo_ros_tools::Relay<@SHM_TYPE @, @ROS_TYPE @, any_measurements_ros::ConversionTraits> myRelay(argc, argv, argv[1],
                                                                                                   argv[2]);
  myRelay.run();
  return 0;
}