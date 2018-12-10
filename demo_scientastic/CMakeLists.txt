cmake_minimum_required(VERSION 2.8.3)

project(demo_scientastic)

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
  custom_msgs
)

find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#Generate services 


find_package(Eigen3 REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES worldActorTransform
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation tf gazebo_ros custom_msgs
 )


include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIR}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# Generate executable

add_executable(foot_control src/foot_control_node.cpp src/foot_control.cpp)
target_link_libraries(foot_control ${catkin_LIBRARIES})
add_dependencies(foot_control ${PROJECT_NAME}_gencpp)

add_executable(reset_boxes_pose src/reset_boxes_pose_node.cpp)
target_link_libraries(reset_boxes_pose ${catkin_LIBRARIES})
add_dependencies(reset_boxes_pose ${PROJECT_NAME}_gencpp)

add_library(footTfUpdate SHARED src/gz_plugins/footTfUpdate.cc)
target_link_libraries(footTfUpdate 
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

# cmake_minimum_required(VERSION 2.8.3)

# project(demo_scientastic)

# # Add support for c++11
# add_definitions(-std=c++11)

# # Find required packages

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   gazebo_ros
#   std_msgs	
#   message_generation
#   dynamic_reconfigure
#   tf
# )

# find_package(gazebo REQUIRED)


# link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# #Generate services 

# # add_service_files( 
# # 	FILES
# # 	TeleportAbsolute.srv
# # 	Kill.srv
# # 	Spawn.srv
# # )	

# find_package(Eigen3 REQUIRED)

# ## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

# # Generate dynamic reconfigure parameters
# # generate_dynamic_reconfigure_options(cfg/protocol_params.cfg)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES
#   CATKIN_DEPENDS roscpp rospy std_msgs message_generation tf gazebo_ros
#  )


# include_directories(include ${catkin_INCLUDE_DIRS})

# include_directories(${Boost_INCLUDE_DIR}
# 					${catkin_INCLUDE_DIRS}
# 					${GAZEBO_INCLUDE_DIRS}
# 					${EIGEN3_INCLUDE_DIR})

# link_directories(${GAZEBO_LIBRARY_DIRS})

# #list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# # Generate executable

# # add_executable(joy_control src/joy_control.cpp src/joy_control_node.cpp)
# # target_link_libraries(joy_control ${catkin_LIBRARIES})
# # add_dependencies(joy_control ${PROJECT_NAME}_gencpp)

# add_executable(foot_control src/foot_control.cpp src/foot_control_node.cpp)
# target_link_libraries(foot_control ${catkin_LIBRARIES})
# add_dependencies(foot_control ${PROJECT_NAME}_gencpp)
# # add_dependencies(foot_control ${catkin_EXPORTED_TARGETS})
# # add_dependencies(foot_control custom_msgs_generate_messages_cpp)

# add_executable(reset_boxes_pose src/reset_boxes_pose_node.cpp)
# target_link_libraries(reset_boxes_pose ${catkin_LIBRARIES})
# add_dependencies(reset_boxes_pose ${PROJECT_NAME}_gencpp)

# add_library(footTfUpdate SHARED src/gz_plugins/footTfUpdate.cc)
# target_link_libraries(footTfUpdate 
# 					  ${catkin_LIBRARIES}
# 					  ${GAZEBO_LIBRARIES}
# 					  ${Boost_LIBRARIES})

# # Mark launch files for installation

# install (FILES
# 	gazebo.launch
# 	visualization.rviz.launch
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

# #New from gazebo6 : to compile c++11 flags against gazebo (required)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
