#ifndef __sharedControlGrasp_H__
#define __sharedControlGrasp_H__

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/kdl.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>

#include "custom_msgs/FootInputMsg_v5.h"
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include "vibrator.h"
#include "smoothSignals.h"
#include "PIDd.h"
#include "MatLP_Filterd.h"
#include <custom_msgs_gripper/SharedGrasping.h>

const uint8_t NB_AXIS_POSITIONING = 4;

using namespace std;
using namespace Eigen;

#define TOOL_AXES                                                                   \
  ListofToolAxes(tool_pitch, "tool_pitch")    \
  ListofToolAxes(tool_roll, "tool_roll")              \
  ListofToolAxes(tool_yaw, "tool_yaw")    \
  ListofToolAxes(tool_insertion, "tool_insertion")  \
  ListofToolAxes(tool_wrist_pitch, "tool_wrist_pitch")  \
  ListofToolAxes(tool_wrist_yaw, "tool_wrist_yaw")  \
  ListofToolAxes(tool_wrist_open_angle, "tool_wrist_open_angle")  \
  ListofToolAxes(tool_wrist_open_angle_mimic, "tool_wrist_open_angle_mimic")  \
  ListofToolAxes(NB_TOOL_AXIS_FULL, "total_tool_joints") 
#define ListofToolAxes(enumeration, names) enumeration,
enum Tool_Axis : size_t { TOOL_AXES };
#undef ListofToolAxes
extern const char *Tool_Axis_Names[];

#define NB_TOOL_AXIS_RED (NB_TOOL_AXIS_FULL - 4)

const float SCALE_GAINS_LINEAR_POSITION  = 1.0f;
const float SCALE_GAINS_ANGULAR_POSITION = 1e-4f * RAD_TO_DEG;

class sharedControlGrasp {

private:
  
  static sharedControlGrasp *me;
  
  enum ToolID { UNKNOWN = 0, RIGHT_TOOL = 1, LEFT_TOOL = 2};

  //enum Action_State {A_POSITIONING, A_GRASPING, NB_ACTIONS};
  enum Action_State {A_POSITIONING_OPEN, A_GRASPING, A_POSITIONING_CLOSE, NB_ACTIONS};

  // Target_Status _myStatus;
  
  Action_State _aState;
  Action_State _aStateNext;

  ToolID _myID;
  // TrackMode _myTrackMode; 
  // Eigen and Geometry

  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointSpeed;
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointPosition;


  double _graspingAngle;
  double _graspingAngleSpeed;

  double _hapticAxisFilterPos;
  double _hapticAxisFilterGrasp;
  
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointPosition;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointPositionOffset;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointVelocity;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointEffort;
  

  Eigen::Matrix<double,NB_AXIS_POSITIONING,1> _thresholds;

  double _vibrationGrasping;
  double _impedanceGrasping;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _hapticTorques;

  double _precisionPos, _precisionAng,_precisionGrasp;

  
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kpPosition;
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kiPosition;
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kdPosition;


  LP_Filterd _thresholdFilter;
  LP_Filterd _kpPositionFilter[NB_AXIS_POSITIONING];
  LP_Filterd _kiPositionFilter[NB_AXIS_POSITIONING];
  LP_Filterd _kdPositionFilter[NB_AXIS_POSITIONING];

  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlRef;
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlIn;
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlInPrev;
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlOut;

  PIDd* _pidPosition[NB_AXIS_POSITIONING];
  PIDd* _pidGrasping;

  double _graspCtrlRef;
  double _graspCtrlIn;
  double _graspCtrlInPrev;
  double _graspCtrlOut;
  double _myThreshold;

  double _kpGrasping;
  double _kiGrasping;
  double _kdGrasping;

  std::mutex _mutex;

  // ROS
  
  // ros variables
  ros::NodeHandle _n;
  ros::Rate _loopRate;
  float _dt;
  
  ros::Publisher _pubSharedGrasp;
  custom_msgs_gripper::SharedGrasping _msgSharedGrasp;

  
  ros::Publisher _pubFootInput;


  custom_msgs::FootInputMsg_v5 _msgFootInput;

  ros::Subscriber _subToolJointStates;
  ros::Subscriber _subPlatformJointStates;

  vibrator* _myVibrator;
  smoothSignals* _mySmoothSignals;
  
  //! boolean variables
  bool  _flagPlatformJointsConnected;
  bool  _flagToolJointsConnected;
  bool _flagHapticGrasping;
  bool _flagSharedGrasping;

  bool  _stop;


  // METHODS
public:
  sharedControlGrasp(ros::NodeHandle &n_1, double frequency, std::string toolName_);

  ~sharedControlGrasp();

  bool init();
  void run();

private:
  //! ROS METHODS

  void readToolState(const sensor_msgs::JointState::ConstPtr &msg);
  
  void readPlatformState(const sensor_msgs::JointState::ConstPtr &msg);

  void estimateActionState();
  void doSharedControl();

  void computesharedControlGraspPose();

  void publishFootInput();
  void publishSharedGrasp();

  

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __sharedControlGrasp_H__
