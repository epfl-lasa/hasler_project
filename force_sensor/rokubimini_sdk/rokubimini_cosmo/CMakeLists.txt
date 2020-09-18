cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_cosmo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PACKAGE_DEPENDENCIES
  any_node
  cosmo
  cosmo_ros
  rokubimini
  rokubimini_manager
  rokubimini_msgs
  rokubimini_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

add_definitions("--std=c++14")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME} src/RokubiminiCosmo.cpp)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY bin config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS bin/rokubimini_launch.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)