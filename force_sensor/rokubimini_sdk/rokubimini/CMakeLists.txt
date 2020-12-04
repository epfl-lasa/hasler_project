cmake_minimum_required(VERSION 2.8.3)
project(rokubimini)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
    any_measurements
    any_worker  
    yaml_tools
    signal_handler
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES} 
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/calibration/SensorCalibration.cpp
  src/${PROJECT_NAME}/configuration/Configuration.cpp
  src/${PROJECT_NAME}/configuration/ForceTorqueFilter.cpp
  src/${PROJECT_NAME}/configuration/SensorConfiguration.cpp
  src/${PROJECT_NAME}/fsm/StateEnum.cpp
  src/${PROJECT_NAME}/setup/Rokubimini.cpp
  src/${PROJECT_NAME}/setup/Setup.cpp
  src/${PROJECT_NAME}/Reading.cpp
  src/${PROJECT_NAME}/Rokubimini.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/calibration/ForceTorqueCalibration.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
