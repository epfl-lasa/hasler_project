cmake_minimum_required(VERSION 2.8.3)

project(surgical_task)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  sensor_msgs	
  message_generation
  dynamic_reconfigure
  tf
  custom_msgs
  custom_msgs_gripper
  utils
)

find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)


add_message_files(
  DIRECTORY msg
  FILES 
  SurgicalTaskStateMsg.msg)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_generation tf gazebo_ros custom_msgs utils custom_msgs_gripper
 )

if(NOT qpOASES_FOUND)
  include(cmake/FindqpOASES.cmake)
endif() 
find_package(qpOASES QUIET)



include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIR}
          ${catkin_INCLUDE_DIRS}
          ${GAZEBO_INCLUDE_DIRS}
          ${EIGEN3_INCLUDE_DIR})

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${qpOASES_INCLUDE_DIRS})

# ${qpOASES_LIBRARIES} jacobian qpSolverRCM
add_library(jacobian src/JacobianDefinitions.cpp include/JacobianDefinitions.h)
add_library(qpSolverRCM STATIC src/QpSolverRCM.cpp include/QpSolverRCM.h
                               src/QpSolverRCM2.cpp include/QpSolverRCM2.h
                               src/QpSolverRCM3.cpp include/QpSolverRCM3.h)
target_link_libraries(qpSolverRCM ${qpOASES_LIBRARIES} jacobian)

add_library(cvxgenSolverRCM include/cvxgen/solver.c 
                            include/cvxgen/matrix_support.c 
                            include/cvxgen/ldl.c 
                            src/CvxgenSolverRCM.cpp 
                            include/CvxgenSolverRCM.h  
                            include/cvxgen/solver.h)

add_executable(test_cvxgen src/test_cvxgen.cpp)
target_link_libraries(test_cvxgen ${catkin_LIBRARIES} cvxgenSolverRCM jacobian)
add_dependencies(test_cvxgen ${PROJECT_NAME}_gencpp)
# set_target_properties(qpSolverRCM PROPERTIES POSITION_INDEPENDENT_CODE ON)

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
# Generate executable

# add_executable(move_to_desired_joints src/moveToDesiredJoints_node.cpp src/MoveToDesiredJoints.cpp)
# target_link_libraries(move_to_desired_joints ${catkin_LIBRARIES})
# add_dependencies(move_to_desired_joints ${PROJECT_NAME}_gencpp)

# add_executable(move_to_desired_pose src/moveToDesiredPose_node.cpp src/MoveToDesiredPose.cpp)
# target_link_libraries(move_to_desired_pose ${catkin_LIBRARIES})
# add_dependencies(move_to_desired_pose ${PROJECT_NAME}_gencpp)

add_executable(spherical_trocar_frames src/sphericalTrocarFrames_node.cpp src/SphericalTrocarFrames.cpp)
target_link_libraries(spherical_trocar_frames ${catkin_LIBRARIES})
add_dependencies(spherical_trocar_frames ${PROJECT_NAME}_gencpp)

add_executable(pillar_frames src/pillarFrames_node.cpp src/PillarFrames.cpp)
target_link_libraries(pillar_frames ${catkin_LIBRARIES})
add_dependencies(pillar_frames ${PROJECT_NAME}_gencpp)

add_executable(surgical_task src/surgicalTask_node.cpp src/SurgicalTask.cpp)
target_link_libraries(surgical_task ${catkin_LIBRARIES} qpSolverRCM cvxgenSolverRCM)
add_dependencies(surgical_task ${PROJECT_NAME}_gencpp)

add_executable(test_qp src/test_qp.cpp)
target_link_libraries(test_qp ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} jacobian qpSolverRCM)
add_dependencies(test_qp ${PROJECT_NAME}_gencpp)

add_executable(test_ik src/test_ik.cpp)
target_link_libraries(test_ik ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} qpSolverRCM cvxgenSolverRCM)
add_dependencies(test_ik ${PROJECT_NAME}_gencpp)

# add_library(gzTfUpdate SHARED src/gz_plugins/gzTfUpdate.cc)
# target_link_libraries(gzTfUpdate 
# 					  ${catkin_LIBRARIES}
# 					  ${GAZEBO_LIBRARIES}
# 					  ${Boost_LIBRARIES})

# Mark launch files for installation

# install (FILES
# 	gazebo.launch
# 	visualization.rviz.launch
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

#New from gazebo6 : to compile c++11 flags against gazebo (required)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
