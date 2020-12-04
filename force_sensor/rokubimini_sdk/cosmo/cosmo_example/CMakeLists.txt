cmake_minimum_required(VERSION 2.8.3)
project(cosmo_example)

find_package(catkin REQUIRED COMPONENTS
  cosmo
  message_logger
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    cosmo
    message_logger
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#publisher
add_executable(${PROJECT_NAME}_publisher_node
  src/${PROJECT_NAME}_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_publisher_node
  ${catkin_LIBRARIES}
)

#subscriber
add_executable(${PROJECT_NAME}_subscriber_node
  src/${PROJECT_NAME}_subscriber.cpp
)

target_link_libraries(${PROJECT_NAME}_subscriber_node
  ${catkin_LIBRARIES}
)


#sync master
add_executable(syncmaster
  src/syncmaster.cpp
)

target_link_libraries(syncmaster
  ${catkin_LIBRARIES}
)

#sync slave
add_executable(syncslave
  src/syncslave.cpp
)

target_link_libraries(syncslave
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}_publisher_node ${PROJECT_NAME}_subscriber_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
