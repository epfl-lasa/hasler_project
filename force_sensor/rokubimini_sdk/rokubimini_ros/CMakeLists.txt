cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_ros)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
    any_measurements
    any_measurements_ros
    rokubimini
    rokubimini_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

