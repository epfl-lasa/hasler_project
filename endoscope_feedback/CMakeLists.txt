cmake_minimum_required(VERSION 3.0.2)
project(endoscope_feedback)

# Add support for c++11
add_definitions(-std=c++11 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  utils_math
  message_generation
  message_runtime
)


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)


add_message_files(
  DIRECTORY msg
	FILES 
  SurgicalTaskStateMsg.msg)
  
generate_messages(DEPENDENCIES  std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge  image_transport  roscpp  sensor_msgs  std_msgs utils_math message_runtime
  DEPENDS 
  )
  find_package(OpenCV)

  
  include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
  
  add_executable(endoscopeOverlay src/endoscopeModifier_node.cpp src/endoscopeModifier.cpp)
  
  target_link_libraries(endoscopeOverlay ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


