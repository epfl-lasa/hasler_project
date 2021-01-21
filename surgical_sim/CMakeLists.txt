cmake_minimum_required(VERSION 2.8.3)
project(surgical_sim)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_kdl
  eigen_conversions
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
  custom_msgs
  custom_msgs_gripper
  utils_math
  urdf
  kdl_parser
  kdl_conversions
  gazebo_ros
  controller_manager
  controller_manager_msgs
  controller_manager_tests
  controller_interface
  hardware_interface
  joint_limits_interface
  transmission_interface
  realtime_tools 
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS date_time)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)





#Generate dynamic_reconfigure parameters
#generate_dynamic_reconfigure_options(cfg/machineStateParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros tf2 tf2_ros tf2_kdl custom_msgs dynamic_reconfigure  utils_math custom_msgs urdf kdl_parser eigen_conversions controller_manager controller_manager_msgs realtime_tools transmission_interface hardware_interface controller_manager_tests  joint_limits_interface hardware_interface kdl_conversions custom_msgs_gripper utils_math endoscope_feedback
  DEPENDS gazebo_ros
  )

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(surgicalToolSim src/surgicalTool.cpp src/surgicalTool_node.cpp)
target_link_libraries(surgicalToolSim ${catkin_LIBRARIES})

add_executable(pandaControllerSim src/pandaController.cpp src/pandaController_node.cpp)
target_link_libraries(pandaControllerSim ${catkin_LIBRARIES})

add_executable(targetObject src/protocol/targetObject.cpp src/protocol/targetObject_node.cpp)
target_link_libraries(targetObject ${catkin_LIBRARIES})

add_executable(sharedGrasp src/protocol/sharedControlGrasp.cpp src/protocol/sharedControlGrasp_node.cpp src/protocol/vibrator.cpp src/protocol/smoothSignals.cpp)
target_link_libraries(sharedGrasp ${catkin_LIBRARIES})

install(PROGRAMS scripts/init_franka_right.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/init_franka_left.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/init_tool_right.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/init_camera_left.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(gzTfUpdate SHARED src/gz_plugins/gzTfUpdate.cc)
target_link_libraries(gzTfUpdate 
					  ${catkin_LIBRARIES}
					  ${GAZEBO_LIBRARIES}
            ${Boost_LIBRARIES})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


