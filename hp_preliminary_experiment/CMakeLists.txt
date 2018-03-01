cmake_minimum_required(VERSION 2.8.3)

project(hp_preliminary_experiment)

# Add support for c++11
add_definitions(-std=c++11)

# Find required packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  dynamic_reconfigure
  sg_differentiation
)

find_package(Eigen3 REQUIRED)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
# generate_dynamic_reconfigure_options(cfg/preliminaryExperiment_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure sg_differentiation
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

# Generate executable
add_executable(preliminaryExperiment src/preliminary_experiment_node.cpp src/PreliminaryExperiment.cpp)
target_link_libraries(preliminaryExperiment ${catkin_LIBRARIES})
add_dependencies(preliminaryExperiment ${PROJECT_NAME}_gencpp)

add_executable(kinectRecording src/kinectRecording_node.cpp src/KinectRecording.cpp)
target_link_libraries(kinectRecording ${catkin_LIBRARIES})
add_dependencies(kinectRecording ${PROJECT_NAME}_gencpp)
