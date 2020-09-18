cmake_minimum_required(VERSION 2.8.3)
project(rokubimini_msgs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-std=c++11 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    ${PACKAGE_DEPENDENCIES}
)

add_message_files(
  FILES
    Command.msg
    Reading.msg
)

add_service_files(
  FILES
    GetSensorConfiguration.srv
    SetSensorConfiguration.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)
