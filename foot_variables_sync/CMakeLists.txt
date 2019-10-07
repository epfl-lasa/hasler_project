cmake_minimum_required(VERSION 2.8.3)

project(foot_variables_sync)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  utils
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters
generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)

#generate_dynamic_reconfigure_options(cfg/position_control_params.cfg)
#generate_dynamic_reconfigure_options(cfg/speed_control_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs custom_msgs dynamic_reconfigure utils
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(footVarSynchronizer src/footVarSynchronizer.cpp src/footVarSynchronizer_node.cpp)
target_link_libraries(footVarSynchronizer ${catkin_LIBRARIES})

add_dependencies(footVarSynchronizer ${PROJECT_NAME}_gencpp)

