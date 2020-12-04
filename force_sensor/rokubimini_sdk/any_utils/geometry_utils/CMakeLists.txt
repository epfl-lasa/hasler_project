cmake_minimum_required(VERSION 2.8.3)
project(geometry_utils)

add_definitions(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
    any_measurements
    kindr
    message_logger
    std_utils
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    any_measurements
    kindr
    message_logger
    std_utils
)

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CoordinateSpace.cpp
  src/PoseStamped.cpp
  src/TransformListener.cpp
  src/TransformStamped.cpp
  src/TwistStamped.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#################
##    Tests    ##
#################
if(CATKIN_ENABLE_TESTING)
    find_package(roscpp REQUIRED)
    include_directories( ${roscpp_INCLUDE_DIRS})

    catkin_add_gtest(test_${PROJECT_NAME}
        test/TestCoordinateSpace.cpp
        test/TestPoseStamped.cpp
        test/test_main.cpp
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
    )

    if(TARGET test_${PROJECT_NAME})
        target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
        )
    endif()
endif()

#################
## Clang Tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    add_clang_tooling(
            TARGET ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_FIX
    )
endif(cmake_clang_tools_FOUND)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
