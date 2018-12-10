cmake_minimum_required(VERSION 2.8.3)

project(surgical_simulator)

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
  tf
  utils
)

find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#Generate services 

# add_service_files( 
	# FILES
	# TeleportAbsolute.srv
	# Kill.srv
	# Spawn.srv
# )	

find_package(Eigen3 REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
# generate_dynamic_reconfigure_options(cfg/protocol_params.cfg)
generate_dynamic_reconfigure_options(cfg/robotsTaskGeneration_params.cfg)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation tf gazebo_ros custom_msgs utils
 )


include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GAZEBO_INCLUDE_DIRS}
					${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# Generate executable

add_executable(move_to_desired_joints src/moveToDesiredJoints_node.cpp src/MoveToDesiredJoints.cpp)
target_link_libraries(move_to_desired_joints ${catkin_LIBRARIES})
add_dependencies(move_to_desired_joints ${PROJECT_NAME}_gencpp)

add_executable(move_to_desired_pose src/moveToDesiredPose_node.cpp src/MoveToDesiredPose.cpp)
target_link_libraries(move_to_desired_pose ${catkin_LIBRARIES})
add_dependencies(move_to_desired_pose ${PROJECT_NAME}_gencpp)

add_executable(publish_trocar_frame src/publishTrocarFrame_node.cpp src/PublishTrocarFrame.cpp)
target_link_libraries(publish_trocar_frame ${catkin_LIBRARIES})
add_dependencies(publish_trocar_frame ${PROJECT_NAME}_gencpp)

add_executable(robots_task_generation src/robotsTaskGeneration_node.cpp src/RobotsTaskGeneration.cpp)
target_link_libraries(robots_task_generation ${catkin_LIBRARIES})
add_dependencies(robots_task_generation ${PROJECT_NAME}_gencpp)

# add_library(gzTfUpdate SHARED src/gz_plugins/gzTfUpdate.cc)
# target_link_libraries(gzTfUpdate 
# 					  ${catkin_LIBRARIES}
# 					  ${GAZEBO_LIBRARIES}
# 					  ${Boost_LIBRARIES})

# Mark launch files for installation

# install (FILES
# 	gazebo.launch
# 	visualization.rviz.launch
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
