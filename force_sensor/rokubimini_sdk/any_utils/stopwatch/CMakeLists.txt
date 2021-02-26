cmake_minimum_required(VERSION 2.8.3)
project(stopwatch)

add_definitions(-std=c++11 -Wall -Werror)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/Statistics.cpp
  src/${PROJECT_NAME}/Stopwatch.cpp
  src/${PROJECT_NAME}/Stopwatches.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_WERROR
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if(CATKIN_ENABLE_TESTING)
#  # Tests in order of their dependency.
#  catkin_add_gtest(
#    ${PROJECT_NAME}_test
#    test/Test.cpp
#    test/${PROJECT_NAME}_test.cpp
#  )
#endif()
#
#if(TARGET ${PROJECT_NAME}_test)
#  target_link_libraries(
#    ${PROJECT_NAME}_test
#    ${PROJECT_NAME}
#  )
#endif()

