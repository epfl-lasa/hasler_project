cmake_minimum_required(VERSION 2.8.3)
project(leg_as_robot)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_kdl
  eigen_conversions
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  utils_math
  urdf
  kdl_parser
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters
#generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs tf2 tf2_ros tf2_kdl custom_msgs dynamic_reconfigure  utils_math custom_msgs urdf kdl_parser eigen_conversions
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(legRobot src/legRobot.cpp src/legRobot_node.cpp)

target_link_libraries(legRobot ${catkin_LIBRARIES})


