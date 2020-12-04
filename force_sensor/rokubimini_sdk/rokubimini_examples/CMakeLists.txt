cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_examples)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
    rokubimini
    rokubimini_manager
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES} 
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_simple_example
  src/${PROJECT_NAME}/simple_example.cpp
)


target_link_libraries(
  ${PROJECT_NAME}_simple_example
  ${catkin_LIBRARIES}
  )

add_dependencies(
  ${PROJECT_NAME}_simple_example
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_simple_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)