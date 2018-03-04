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
  tf
)

find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#Generate services 

add_service_files( 
	FILES
	TeleportAbsolute.srv
	Kill.srv
	Spawn.srv
)	

find_package(Eigen3 REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/protocol_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES worldActorTransform
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation tf gazebo_ros
 )


include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIR}
					${catkin_INCLUDE_DIRS}
					${GAZEBO_INCLUDE_DIRS}
					${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# Generate executable

add_executable(worldActorTransform src/world_actor_tf_pub_node.cpp src/world_actor_tf_pub.cpp)
target_link_libraries(worldActorTransform ${catkin_LIBRARIES})
add_dependencies(worldActorTransform ${PROJECT_NAME}_gencpp)

add_executable(protocol src/protocol_node.cpp src/Protocol.cpp)
target_link_libraries(protocol ${catkin_LIBRARIES})
add_dependencies(protocol ${PROJECT_NAME}_gencpp)

add_executable(joy_control src/joy_control.cpp src/joy_control_node.cpp)
target_link_libraries(joy_control ${catkin_LIBRARIES})
add_dependencies(joy_control ${PROJECT_NAME}_gencpp)

add_library(gzTfUpdate SHARED src/gz_plugins/gzTfUpdate.cc)
target_link_libraries(gzTfUpdate 
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
					  ${Boost_LIBRARIES})

# Mark launch files for installation

install (FILES
	gazebo.launch
	visualization.rviz.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
