#ifndef __leg_robot_H__
#define __leg_robot_H__


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <custom_msgs/FootInputMsg_v3.h>
#include <custom_msgs/FootOutputMsg_v2.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv_v2.h>
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include "../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../5_axis_platform/lib/platform/src/definitions_security.h"
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>


//! Joint Space
#define LEG_AXES                                                                   \
  ListofLegAxes(hip_adduction, "hip_adduction")    \
  ListofLegAxes(hip_extension, "hip_extension")    \
  ListofLegAxes(hip_roll, "hip_roll")              \
  ListofLegAxes(knee_extension, "knee_extension")  \
  ListofLegAxes(ankle_pitch, "ankle_pitch")  \
  ListofLegAxes(ankle_roll, "ankle_roll")  \
  ListofLegAxes(ankle_yaw, "ankle_yaw")  \
  ListofLegAxes(NB_LEG_AXIS, "total_joints") 
#define ListofLegAxes(enumeration, names) enumeration,
enum Leg_Axis : size_t { LEG_AXES };
#undef ListofLegAxes
extern const char *Leg_Axis_Names[];

using namespace std;

class legRobot {

public:
  enum Leg_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};

private:
  
  Leg_Name _leg_id;

  // internal variables

  Eigen::Matrix<double, NB_LEG_AXIS, 1> _leg_joints;


  // ros variables
  sensor_msgs::JointState _msgJointStates;

  ros::NodeHandle _n;
  ros::Rate _loopRate;
  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
  tf::TransformListener _footPoseListener;
  Eigen::Vector3d _footPosition;
  Eigen::Quaterniond _footQuaternion;
  Eigen::Matrix3d _footRotationMatrix;

  // Publisher declaration
  ros::Publisher _pubLegJointStates;

  //! boolean variables

  bool _flagFootPoseConnected;
  bool _stop;

  std::mutex _mutex;
  static legRobot *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  legRobot(ros::NodeHandle &n_1, double frequency,
                      legRobot::Leg_Name leg_id_);

  ~legRobot();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void publishFootJointStates();
  void readFootPose();
  void performInverseKinematics();

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __leg_robot_H__