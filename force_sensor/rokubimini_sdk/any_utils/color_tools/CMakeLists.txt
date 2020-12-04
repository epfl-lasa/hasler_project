cmake_minimum_required(VERSION 2.8.3)
project(color_tools)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
