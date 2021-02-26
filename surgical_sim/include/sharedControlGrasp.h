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

#include "custom_msgs/FootInputMsg.h"
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
#include <custom_msgs_gripper/SharedGraspingMsg.h>
#include <custom_msgs_gripper/GripperOutputMsg.h>

#include <custom_msgs/SurgicalTaskStateMsg.h>

const uint8_t NB_AXIS_POSITIONING = 4;

using namespace std;
using namespace Eigen;

#define TOOL_AXES                                                                   \
  ListofToolAxes(tool_pitch, "tool_joint1_pitch")    \
  ListofToolAxes(tool_roll, "tool_joint2_roll")              \
  ListofToolAxes(tool_yaw, "tool_joint3_yaw")    \
  ListofToolAxes(tool_insertion, "tool_joint4_insertion")  \
  ListofToolAxes(tool_wrist_open_angle, "tool_joint5_wrist_open_angle")  \
  ListofToolAxes(tool_wrist_open_angle_mimic, "tool_joint5_wrist_open_angle_mimic")  \
  ListofToolAxes(NB_TOOL_AXIS, "total_tool_joints") 
#define ListofToolAxes(enumeration, names) enumeration,
enum Tool_Axis : size_t { TOOL_AXES };
#undef ListofToolAxes
extern const char *Tool_Axis_Names[];

#define NB_TOOL_AXIS_FULL NB_TOOL_AXIS
#define NB_TOOL_AXIS_RED (NB_TOOL_AXIS_FULL - 4)

const float SCALE_GAINS_LINEAR_POSITION  = 1.0f;
const float SCALE_GAINS_ANGULAR_POSITION = 1e-4f * RAD_TO_DEG;

class sharedControlGrasp {

private:
  
  static sharedControlGrasp *me;
  
  enum ToolID { UNKNOWN = 0, RIGHT_TOOL = 1, LEFT_TOOL = 2};
  enum SimType {KINEMATIC_SIM, DYNAMIC_SIM};
  //enum Action_State {A_POSITIONING, A_GRASPING, NB_ACTIONS};
  enum Action_State {A_POSITIONING_OPEN, A_GRASPING, A_HOLDING_GRASP, A_POSITIONING_CLOSE, A_FETCHING_OLD_GRASP, A_RELEASE_GRASP, NB_ACTIONS};

  // Target_Status _myStatus;
  
  Action_State _aState;
  Action_State _aStateNext;
  SimType _mySimType;
  ToolID _myID;
  // TrackMode _myTrackMode; 
  // Eigen and Geometry

  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointSpeed;
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointPosition;

  double _graspingAnglePrev;
  double _graspingAngle;
  double _graspingAngleBeforeHolding;
  double _graspingAngleSpeed;

  double _hapticAxisFilterPos;
  double _hapticAxisFilterGrasp;
  
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointPosition;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointPositionOffset;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointVelocity;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointEffort;
  

  Eigen::Matrix<double,NB_AXIS_POSITIONING,1> _thresholds;

  double _vibrationGrasping;
  double _vibInput;
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
  double _dt;
  
  ros::Publisher _pubSharedGrasp;
  custom_msgs_gripper::SharedGraspingMsg _msgSharedGrasp;

  
  ros::Publisher _pubFootInput;


  custom_msgs::FootInputMsg _msgFootInput;
  Eigen::Matrix<double, 6, 1> _wrenchGrasperRobot;
  
  float _realGripperSpeed;
  float _realGripperErrorPos;
  float _realGripperPosition;

  ros::Subscriber _subToolJointStates;
  ros::Subscriber _subPlatformJointStates;
  ros::Subscriber _subSurgicalTaskStates;
  ros::Subscriber _subGripperOutput;

  vibrator* _myVibrator;
  smoothSignals* _mySmoothSignalsPos;
  smoothSignals* _mySmoothSignalsGrasp;
  ros::Time _startTimeForKeepingGrasp;
  ros::Time _startTimeForReleasingGrasp;
  ros::Time _startTimeForChangingGainsToZero;


  
  //! boolean variables
  volatile bool _flagSurgicalTaskStateReceived;
  volatile bool _flagGripperOutputMsgReceived;
  volatile bool  _flagPlatformJointsConnected;
  volatile bool  _flagToolJointsConnected;

  bool _flagHapticGrasping;
  bool _flagSharedGrasping;
  bool _flagGraspingStarted;
  bool _flagHoldingGraspStarted;
  bool _flagPositioningStarted;

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
  void readSurgicalTaskState(const custom_msgs::SurgicalTaskStateMsgConstPtr& msg);
  void readGripperOutput(const custom_msgs_gripper::GripperOutputMsgConstPtr& msg);

  void estimateActionState();
  void estimateActionTransition();
  void doSharedControl();

  void computesharedControlGraspPose();

  void publishFootInput();
  void publishSharedGrasp();

  

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __sharedControlGrasp_H__
