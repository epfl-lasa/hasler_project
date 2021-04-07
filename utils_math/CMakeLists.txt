cmake_minimum_required(VERSION 2.8.3)

project(utils_math)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  kdl_parser
  kdl_conversions)

  
  
  find_package(PkgConfig)

pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES utils_math kdl_custom
  CATKIN_DEPENDS roscpp rospy kdl_parser kdl_conversions
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

FILE(GLOB SRCS src/Utils_math.cpp src/LP_Filterd.cpp src/MatLP_Filterd.cpp src/PIDd.cpp src/smoothSignals.cpp src/vibrator.cpp src/gaussianFilter.cpp) # src/torque2TaskSpace_wdls.cpp)
FILE(GLOB INCS include/Utils_math.h include/LP_Filterd.h  include/ MatLP_Filterd.h include/PIDd.h include/smoothSignals.h include/vibrator.h include/gaussianFilter.h ) #include/torque2TaskSpace_wdls.h)

add_library(utils_math ${SRCS} ${INCS})
add_library(kdl_custom src/torque2TaskSpace_wdls.cpp)
target_link_libraries(utils_math ${catkin_LIBRARIES})
target_link_libraries(kdl_custom ${catkin_LIBRARIES})

