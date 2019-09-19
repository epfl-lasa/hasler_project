cmake_minimum_required(VERSION 2.8.3)

project(robotic_experiments)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
  sg_differentiation
  utils
)

# find_package(Eigen 3 QUIET) 
# if(NOT Eigen_FOUND)
#   include(cmake/FindEigen3.cmake)
# endif() 


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/feetTelemanipulation_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation dynamic_reconfigure sg_differentiation utils custom_msgs
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(cartesian_feet_telemanipulation src/cartesianFeetTelemanipulation_node.cpp src/CartesianFeetTelemanipulation.cpp src/FootPlatformModel.cpp)
target_link_libraries(cartesian_feet_telemanipulation ${catkin_LIBRARIES})
add_dependencies(cartesian_feet_telemanipulation ${PROJECT_NAME}_gencpp)

add_executable(trocar_feet_telemanipulation src/trocarFeetTelemanipulation_node.cpp src/TrocarFeetTelemanipulation.cpp)
target_link_libraries(trocar_feet_telemanipulation ${catkin_LIBRARIES})
add_dependencies(trocar_feet_telemanipulation ${PROJECT_NAME}_gencpp)


add_executable(publish_spherical_trocar_frames src/publishSphericalTrocarFrames_node.cpp src/PublishSphericalTrocarFrames.cpp)
target_link_libraries(publish_spherical_trocar_frames ${catkin_LIBRARIES})
add_dependencies(publish_spherical_trocar_frames ${PROJECT_NAME}_gencpp)

FILE(GLOB SRCS src/CartesianFeetTelemanipulation.cpp)
FILE(GLOB INCS include/CartesianFeetTelemanipulation.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})