cmake_minimum_required(VERSION 2.8.3)
project(leg_as_robot)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  utils
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters
#generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs tf tf_conversions custom_msgs dynamic_reconfigure  utils custom_msgs
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(legSolver  src/ikfast61Solver.cpp)
add_executable(legRobot src/legRobot.cpp src/legRobot_node.cpp)

target_link_libraries(legSolver ${catkin_LIBRARIES})
target_link_libraries(legRobot ${catkin_LIBRARIES})


add_dependencies(legSolver ${PROJECT_NAME}_gencpp)
add_dependencies(legSolver ${PROJECT_NAME}_gencfg)
add_dependencies(legRobot ${PROJECT_NAME}_gencpp)
add_dependencies(legRobot ${PROJECT_NAME}_gencfg)


