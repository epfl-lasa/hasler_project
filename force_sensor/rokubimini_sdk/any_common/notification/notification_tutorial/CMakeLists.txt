cmake_minimum_required(VERSION 2.8.3)
project(notification_tutorial)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    notification
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    notification
    roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(publisher_node 
  src/publisher_node.cpp
)
target_link_libraries(publisher_node
  ${catkin_LIBRARIES}
)

add_executable(subscriber_node 
  src/subscriber_node.cpp
)
target_link_libraries(subscriber_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS publisher_node subscriber_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)