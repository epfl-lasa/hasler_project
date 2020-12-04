cmake_minimum_required(VERSION 2.8.3)
project(cosmo_ros_tools)

find_package(catkin REQUIRED COMPONENTS
  cosmo_ros
  signal_handler
  any_measurements_ros
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    cosmo_ros
    signal_handler
    any_measurements_ros
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(MESSAGES
  Imu
  ImuWithCovariance
#  JointState
  NavSat
  Pose
  PoseWithCovariance
  Twist
  TwistWithCovariance
  Wrench
)
set(ROS_HEADERS
  sensor_msgs/Imu.h
  sensor_msgs/Imu.h
#  sensor_msgs/JointState.h
  sensor_msgs/NavSatFix.h
  geometry_msgs/PoseStamped.h
  geometry_msgs/PoseWithCovarianceStamped.h
  geometry_msgs/TwistStamped.h
  geometry_msgs/TwistWithCovarianceStamped.h
  geometry_msgs/Wrench.h
)
set(ROS_TYPES
  sensor_msgs::Imu
  sensor_msgs::Imu
#  sensor_msgs::JointState
  sensor_msgs::NavSatFix
  geometry_msgs::PoseStamped
  geometry_msgs::PoseWithCovarianceStamped
  geometry_msgs::TwistStamped
  geometry_msgs::TwistWithCovarianceStamped
  geometry_msgs::Wrench
)

list(LENGTH MESSAGES len1)
math(EXPR len2 "${len1} - 1")

foreach(val RANGE ${len2})
  list(GET MESSAGES ${val} NAME)
  list(GET ROS_HEADERS ${val} ROS_HEADER)
  list(GET ROS_TYPES ${val} ROS_TYPE)
  set(SHM_HEADER "any_measurements/${NAME}.hpp")
  set(SHM_TYPE "any_measurements::${NAME}")


  configure_file(src/echo_node_template.cpp src/${NAME}_echo_node.cpp)
  configure_file(src/relay_node_template.cpp src/${NAME}_relay_node.cpp)
  #relay
  add_executable(${NAME}_relay_node
    src/${NAME}_relay_node.cpp
  )

  target_link_libraries(${NAME}_relay_node
    ${catkin_LIBRARIES}
  )

  # echo
  add_executable(${NAME}_echo_node
    src/${NAME}_echo_node.cpp
  )

  target_link_libraries(${NAME}_echo_node
    ${catkin_LIBRARIES}
  )

endforeach()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
