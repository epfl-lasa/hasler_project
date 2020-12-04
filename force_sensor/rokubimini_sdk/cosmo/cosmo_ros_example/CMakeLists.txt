cmake_minimum_required(VERSION 2.8.3)
project(cosmo_ros_example)

find_package(catkin REQUIRED COMPONENTS
  cosmo_ros
  message_generation
  std_msgs
)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    cosmo_ros
    message_runtime
    std_msgs
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



#publisher
add_executable(${PROJECT_NAME}_publisher_node
  src/${PROJECT_NAME}_publisher.cpp
)

add_dependencies(${PROJECT_NAME}_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_publisher_node
  ${catkin_LIBRARIES}
)

#subscriber
add_executable(${PROJECT_NAME}_subscriber_node
  src/${PROJECT_NAME}_subscriber.cpp
)

add_dependencies(${PROJECT_NAME}_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_subscriber_node
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}_publisher_node ${PROJECT_NAME}_subscriber_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)