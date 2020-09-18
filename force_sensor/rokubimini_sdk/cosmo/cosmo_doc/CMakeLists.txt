cmake_minimum_required(VERSION 2.8.3)
project(cosmo_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    cosmo
    cosmo_ros
    cosmo_node
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      cosmo
      cosmo_ros
      cosmo_node
    DEPENDS
  )

  # Find cosmo for documentation
  find_path (SOURCE_COSMO_DIR NAMES cosmo/Communicator.hpp PATHS ${cosmo_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_COSMO_DIR}")
    message (STATUS "Found cosmo at: ${SOURCE_COSMO_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find cosmo! ${SOURCE_COSMO_DIR}")
  endif()

  # Find parameter_handler_msgs for documentation
  find_path (SOURCE_COSMO_ROS_DIR NAMES cosmo_ros/cosmo_ros.hpp PATHS ${cosmo_ros_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_COSMO_ROS_DIR}")
    message (STATUS "Found cosmo_ros at: ${SOURCE_COSMO_ROS_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find cosmo_ros! ${SOURCE_COSMO_ROS_DIR}")
  endif()

  # Find parameter_handler_ros for documentation
  find_path (SOURCE_COSMO_NODE_DIR NAMES cosmo_node/cosmo_node.hpp PATHS ${cosmo_node_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_COSMO_NODE_DIR}")
    message (STATUS "Found cosmo_node at: ${SOURCE_COSMO_NODE_DIR}")
  else()
    message (FATAL_ERROR "Couldn't find cosmo_node! ${SOURCE_COSMO_NODE_DIR}")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/\" \"${SOURCE_COSMO_DIR}/../\" \"${SOURCE_COSMO_ROS_DIR}/../\" \"${SOURCE_COSMO_NODE_DIR}/../\" ") # \"${SOURCE_PARA_RQT_DIR}/../\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${SOURCE_PARA__DIR} CACHE)
  unset(${SOURCE_PARA_MSGS_DIR} CACHE)
  unset(${SOURCE_PARA_ROS_DIR} CACHE)
  unset(${SOURCE_PARA_STD_DIR} CACHE)
  unset(${SOURCE_PARA_RQT_DIR} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
