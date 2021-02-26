cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_description)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PACKAGE_DEPENDENCIES
  sensor_msgs
  std_msgs
  xacro
  message_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
  ${PACKAGE_DEPENDENCIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY meshes/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
		)

install(DIRECTORY urdf_src/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf_src
		)

