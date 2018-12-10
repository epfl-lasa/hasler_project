cmake_minimum_required(VERSION 2.8.3)

project(custom_msgs)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
	FILES 
	FootInputMsg.msg
	FootOutputMsg.msg
	FeetOutputMsg.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation
 )


## Generate added messages and services
include_directories(${catkin_INCLUDE_DIRS})

add_executable(feet_output_publisher src/feet_output_publisher.cpp)
target_link_libraries(feet_output_publisher ${catkin_LIBRARIES})
add_dependencies(feet_output_publisher ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(feet_output_publisher ${PROJECT_NAME}_gencpp)

add_library(${PROJECT_NAME} src/feet_output_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
