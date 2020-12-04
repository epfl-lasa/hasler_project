cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_flashloader)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    soem_rsl
)

catkin_package(
  CATKIN_DEPENDS soem_rsl
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}    
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/Flashloader.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(flashloader src/rokubimini_flashloader.cpp)
add_dependencies(flashloader ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flashloader
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} flashloader
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
