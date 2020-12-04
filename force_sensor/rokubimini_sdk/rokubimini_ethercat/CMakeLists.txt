cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_ethercat)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++14 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
  any_worker  
  rokubimini
  rokubimini_bus_manager
  signal_handler
  soem
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
  src/${PROJECT_NAME}/setup/RokubiminiEthercat.cpp
  src/${PROJECT_NAME}/RokubiminiEthercat.cpp
  src/${PROJECT_NAME}/RokubiminiEthercatBusManager.cpp
  src/${PROJECT_NAME}/RokubiminiEthercatSlave.cpp
  src/${PROJECT_NAME}/soem_interface/EthercatBusBase.cpp
  src/${PROJECT_NAME}/soem_interface/EthercatSlaveBase.cpp
  src/${PROJECT_NAME}/soem_interface/common/Macros.cpp
  src/${PROJECT_NAME}/soem_interface/common/ThreadSleep.cpp
  src/${PROJECT_NAME}/soem_interface/examples/ExampleSlave.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
  
add_executable(
  ${PROJECT_NAME}_soem_interface_example
  src/${PROJECT_NAME}/soem_interface/examples/example_slave.cpp
)
add_dependencies(
  ${PROJECT_NAME}_soem_interface_example
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_soem_interface_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_soem_interface_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)