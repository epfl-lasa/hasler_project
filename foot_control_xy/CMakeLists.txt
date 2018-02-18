cmake_minimum_required(VERSION 2.8.3)

project(foot_control_xy)

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
)

find_package(gazebo REQUIRED)

include_directories(			${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#find_package(Eigen3 REQUIRED)


## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
#generate_dynamic_reconfigure_options(cfg/preliminaryExperiment_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure
  # DEPENDS Eigen
 )


include_directories(include ${catkin_INCLUDE_DIRS})

# Generate executable

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
