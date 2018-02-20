# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "foot_control_xy: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(foot_control_xy_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_custom_target(_foot_control_xy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "foot_control_xy" "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" ""
)

get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_custom_target(_foot_control_xy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "foot_control_xy" "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" ""
)

get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_custom_target(_foot_control_xy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "foot_control_xy" "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy
)
_generate_srv_cpp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy
)
_generate_srv_cpp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy
)

### Generating Module File
_generate_module_cpp(foot_control_xy
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(foot_control_xy_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(foot_control_xy_generate_messages foot_control_xy_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_cpp _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_cpp _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_cpp _foot_control_xy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foot_control_xy_gencpp)
add_dependencies(foot_control_xy_gencpp foot_control_xy_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foot_control_xy_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy
)
_generate_srv_eus(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy
)
_generate_srv_eus(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy
)

### Generating Module File
_generate_module_eus(foot_control_xy
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(foot_control_xy_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(foot_control_xy_generate_messages foot_control_xy_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_eus _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_eus _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_eus _foot_control_xy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foot_control_xy_geneus)
add_dependencies(foot_control_xy_geneus foot_control_xy_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foot_control_xy_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy
)
_generate_srv_lisp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy
)
_generate_srv_lisp(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy
)

### Generating Module File
_generate_module_lisp(foot_control_xy
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(foot_control_xy_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(foot_control_xy_generate_messages foot_control_xy_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_lisp _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_lisp _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_lisp _foot_control_xy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foot_control_xy_genlisp)
add_dependencies(foot_control_xy_genlisp foot_control_xy_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foot_control_xy_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy
)
_generate_srv_nodejs(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy
)
_generate_srv_nodejs(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy
)

### Generating Module File
_generate_module_nodejs(foot_control_xy
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(foot_control_xy_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(foot_control_xy_generate_messages foot_control_xy_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_nodejs _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_nodejs _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_nodejs _foot_control_xy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foot_control_xy_gennodejs)
add_dependencies(foot_control_xy_gennodejs foot_control_xy_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foot_control_xy_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy
)
_generate_srv_py(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy
)
_generate_srv_py(foot_control_xy
  "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy
)

### Generating Module File
_generate_module_py(foot_control_xy
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(foot_control_xy_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(foot_control_xy_generate_messages foot_control_xy_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Spawn.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_py _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/TeleportAbsolute.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_py _foot_control_xy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacob/catkin_ws/src/hasler_project/foot_control_xy/srv/Kill.srv" NAME_WE)
add_dependencies(foot_control_xy_generate_messages_py _foot_control_xy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(foot_control_xy_genpy)
add_dependencies(foot_control_xy_genpy foot_control_xy_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS foot_control_xy_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/foot_control_xy
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(foot_control_xy_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/foot_control_xy
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(foot_control_xy_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/foot_control_xy
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(foot_control_xy_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/foot_control_xy
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(foot_control_xy_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/foot_control_xy
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(foot_control_xy_generate_messages_py std_msgs_generate_messages_py)
endif()
