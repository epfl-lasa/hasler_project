cmake_minimum_required(VERSION 2.8.3)
project(notification_desktop)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    notification
    roscpp
)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_search_module(LIBNOTIFY REQUIRED libnotify)

catkin_package(
  CATKIN_DEPENDS
    notification
    roscpp
  DEPENDS
    LIBNOTIFY
    GLIB
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${LIBNOTIFY_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/notifcation_desktop.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${LIBNOTIFY_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)