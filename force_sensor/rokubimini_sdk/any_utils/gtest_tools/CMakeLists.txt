cmake_minimum_required(VERSION 2.8.3)
project(gtest_tools)

find_package(catkin REQUIRED)
# find_package(GTest REQUIRED)

include_directories(
  include
  #   ${GTEST_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  # ${GTEST_INCLUDE_DIRS}
)

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
