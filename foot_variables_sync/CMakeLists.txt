cmake_minimum_required(VERSION 2.8.3)

project(foot_variables_sync)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  utils_math
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#Generate dynamic_reconfigure parameters
generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs custom_msgs dynamic_reconfigure utils_math custom_msgs
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_executable(footVarSynchronizer 
               src/footVarSynchronizer_run.cpp 
               src/footVarSynchronizer_config.cpp
               src/footVarSynchronizer_create.cpp
               src/footVarSynchronizer_dynReconfig.cpp
               src/footVarSynchronizer_gains.cpp
               src/footVarSynchronizer_init.cpp
               src/footVarSynchronizer_legComp.cpp
               src/footVarSynchronizer_node.cpp
               src/footVarSynchronizer_param.cpp
               src/footVarSynchronizer_pub.cpp
               src/footVarSynchronizer_srv.cpp 
               src/footVarSynchronizer_sub.cpp 
               src/footVarSynchronizer_synchronize.cpp 
               )
target_link_libraries(footVarSynchronizer ${catkin_LIBRARIES})

# add_dependencies(footVarSynchronizer ${PROJECT_NAME}_gencpp)
# add_dependencies(footVarSynchronizer ${PROJECT_NAME}_gencfg)

