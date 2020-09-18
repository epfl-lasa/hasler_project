cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_serial)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++14 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
  any_worker  
  rokubimini
  signal_handler
  rokubimini_bus_manager
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
  src/${PROJECT_NAME}/setup/RokubiminiSerial.cpp
  src/${PROJECT_NAME}/RokubiminiSerial.cpp
  src/${PROJECT_NAME}/RokubiminiSerialImpl.cpp
  src/${PROJECT_NAME}/RokubiminiSerialBusManager.cpp
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

