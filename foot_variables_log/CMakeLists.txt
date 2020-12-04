cmake_minimum_required(VERSION 2.8.3)

project(foot_variables_log)

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
  roslib
)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters
#generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs custom_msgs dynamic_reconfigure utils_math roslib
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(footVarLogger src/footVarLogger.cpp src/footVarLogger_node.cpp src/FootPlatformModel.cpp)
target_link_libraries(footVarLogger ${catkin_LIBRARIES})

add_executable(dryFrictionLogger src/dryFrictionLogger.cpp src/dryFrictionLogger_node.cpp)
target_link_libraries(dryFrictionLogger ${catkin_LIBRARIES})

# add_executable(viscousFrictionLogger src/viscousFrictionLogger.cpp src/viscousFrictionLogger_node.cpp)
# target_link_libraries(viscousFrictionLogger ${catkin_LIBRARIES})

add_dependencies(footVarLogger ${PROJECT_NAME}_gencpp)
add_dependencies(dryFrictionLogger ${PROJECT_NAME}_gencpp)

FILE(GLOB SRCS src/footVarLogger.cpp)
FILE(GLOB INCS include/footVarLogger.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
