#ifndef __surgicalTool_H__
#define __surgicalTool_H__

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/articulatedbodyinertia.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kdl.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"

#include <std_msgs/Float64MultiArray.h>

// #include <custom_msgs/FootInputMsg_v3.h>
// #include <custom_msgs/FootOutputMsg_v3.h>
// #include <custom_msgs/setControllerSrv.h>
// #include <custom_msgs/setStateSrv_v2.h>
// #include "../../5_axis_platform/lib/platform/src/definitions_pid.h"
// #include "../../5_axis_platform/lib/platform/src/definitions_ros.h"
// #include "../../5_axis_platform/lib/platform/src/definitions_security.h"

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include "MatLP_Filterd.h"
#include "LP_Filterd.h"
#include <geometry_msgs/PoseStamped.h>
#include <custom_msgs_gripper/SharedGraspingMsg.h>

#define LEG_AXES \
  ListofLegAxes(hip_adduction, "hip_adduction")    \
  ListofLegAxes(hip_extension, "hip_extension")    \
  ListofLegAxes(hip_roll, "hip_roll")              \
  ListofLegAxes(knee_extension, "knee_extension")  \
  ListofLegAxes(ankle_pitch, "ankle_pitch")  \
  ListofLegAxes(ankle_roll, "ankle_roll")  \
  ListofLegAxes(ankle_yaw, "ankle_yaw")  \
  ListofLegAxes(NB_LEG_AXIS, "total_leg_joints") 
#define ListofLegAxes(enumeration, names) enumeration,
enum Leg_Axis : size_t { LEG_AXES };
#undef ListofLegAxes
extern const char *Leg_Axis_Names[];



const uint8_t NB_AXIS_POSITIONING = 4;

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


using namespace std;
using namespace Eigen;

class surgicalTool {

public:
  enum Tool_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};


private:
  enum Control_Input { LEG_INPUT = 1, PLATFORM_INPUT = 2 };
  
  Control_Input _myInput;
  
  Tool_Name _tool_id;

  // internal variables

  Eigen::Matrix<double,NB_TOOL_AXIS_RED,1> _toolJointsPosFiltered;
  Eigen::Matrix<double,NB_TOOL_AXIS_FULL,1> _toolJointsAll;
  Eigen::Matrix<double,NB_TOOL_AXIS_FULL,1> _toolJointsAllPrev;
  Eigen::Matrix<double,NB_TOOL_AXIS_FULL,1> _toolJointsAllSpeed;
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL, 1> _toolJointsAllOffset;
  KDL::JntArray* _toolJoints;
  KDL::JntArray* _toolJointsFull;
  KDL::JntArray* _toolJointsInit;
  KDL::JntArray* _toolJointLims[NB_LIMS];
  KDL::JntArray* _toolJointLimsAll[NB_LIMS];
  KDL::JntArray* _legJointLims[NB_LIMS];
  KDL::JntArray* _platformJointLims[NB_LIMS];
  KDL::JntArray* _platformJointLimsDelta;

  Eigen::Matrix<double,NB_AXIS_WRENCH,1> _supportWrenchEigen;
  Eigen::Matrix<double,NB_AXIS_POSITIONING,1> _thresholds;
  
  urdf::Model _myModel;
  urdf::Model _legModel;
  urdf::Model _platformModel;
  KDL::Tree _myTree;
  std::vector<KDL::Segment> _mySegments;
  std::vector<KDL::Segment> _mySegmentsTip;
  std::vector<KDL::Frame> _myFrames;
  std::vector<KDL::Frame> _myFrames_tip;
  // KDL::ChainDynParam*  _myChainDyn;
  KDL::Chain _myToolBaseToWristChain;
  KDL::Chain _myToolWristToTipChain;
  KDL::Chain _myToolBaseToTipChain;
  KDL::Jacobian _myJacobian;
  KDL::ChainFkSolverPos_recursive* _myFKSolver_tip;
  KDL::ChainFkSolverPos_recursive* _myFKSolver_wrist;
  KDL::ChainIkSolverVel_wdls* _myVelIKSolver_tip;
  KDL::ChainIkSolverVel_wdls* _myVelIKSolver_wrist;
  KDL::ChainIkSolverPos_NR_JL* _myPosIkSolver_tip;
  KDL::ChainIkSolverPos_NR_JL* _myPosIkSolver_wrist;
  KDL::ChainJntToJacSolver* _myJacSolver_tip;
  KDL::ChainJntToJacSolver* _myJacSolver_wrist;
  // Eigen::JacobiSVD<MatrixXd> _mySVD;
  

  Eigen::Matrix<double,NB_TOOL_AXIS_RED,NB_TOOL_AXIS_RED> _weightedJointSpaceMassMatrix;
  Eigen::Matrix<double,NB_AXIS_WRENCH,NB_AXIS_WRENCH> _weightedTaskSpaceMassMatrix;
  
  bool _mySolutionFound;
  bool _flagSharedGrasp;
  bool _flagToolJointLimitsOffsetCalculated;
  bool _flagLegJointLimitsOffsetCalculated;
  bool _flagPlatformJointLimitsOffsetCalculated;
  
  
  Eigen::Vector4d _hAxisFilterPosValue;
  double _hAxisFilterGraspValue;

  // ros variables
  ros::NodeHandle _n;
  ros::Rate _loopRate;
  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  KDL::Frame _desiredTargetFrame;
  KDL::Frame _footTipPosFrame;
  KDL::Frame _footTipPosFrameInit;

  Eigen::Vector3d _footTipPosition;
  Eigen::Quaterniond _footTipQuaternion;
  Eigen::Matrix3d _footTipRotationMatrix;
   
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _legJoints;
  Eigen::Matrix<double, NB_LEG_AXIS, 1> _legJointsOffset;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformJoints;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformVelocities;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformEfforts;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformJointsOffset;

  // Publisher declaration
  //ros::Publisher _pubToolJointStates;
  ros::Publisher _pubToolJointCommands;
  ros::Publisher _pubToolTipPose;
  ros::Subscriber _subLegJointStates;
  ros::Subscriber _subPlatformJointStates;
  ros::Subscriber _subSharedGrasp;

  // Messages
  std_msgs::Float64MultiArray _msgJointCommands;
  sensor_msgs::JointState _msgJointStates;
  geometry_msgs::PoseStamped _msgToolTipPose;
  //! boolean variables

  bool _flagFootTipPoseConnected;
  bool _flagLegJointsConnected;
  bool _flagPlatformJointsConnected;

  bool _stop;

  std::mutex _mutex;
  static surgicalTool *me;
  MatLP_Filterd* _hAxisFilterPos;
  LP_Filterd* _hAxisFilterGrasp;
  //! Dynamic Reconfigures

  // METHODS
public:
  surgicalTool(ros::NodeHandle &n_1, double frequency,
                      surgicalTool::Tool_Name tool_id_, urdf::Model model_);

  ~surgicalTool();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  //void publishToolJointStates();
  void publishToolJointCommands();
  void publishToolTipPose();
  void readFootTipBasePose();
  void performInverseKinematics();
  void computeWithLegJoints7DoF();
  void computeWithLegJoints4DoF();
  void computeWithPlatformJoints4DoF();
  void computeWithPlatformTask4DoF();
  void performChainForwardKinematics();
  void calculateDesiredFrame();
  void readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg);
  void readLegJoints(const sensor_msgs::JointState::ConstPtr &msg);
  void readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg);
  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __surgicalTool_H__
