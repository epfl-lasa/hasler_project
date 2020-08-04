cmake_minimum_required(VERSION 2.8.3)

project(friction-id)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  utils_math
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs custom_msgs dynamic_reconfigure utils_math
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(multiAxisFrictionID src/multiAxisFrictionID.cpp src/multiAxisFrictionID_node.cpp src/lp_filter.cpp)
target_link_libraries(multiAxisFrictionID ${catkin_LIBRARIES})

add_dependencies(multiAxisFrictionID ${PROJECT_NAME}_gencpp)

