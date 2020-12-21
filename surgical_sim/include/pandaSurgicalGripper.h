#ifndef __pandaSurgicalGripper_H__
#define __pandaSurgicalGripper_H__

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
//#include <custom_msgs_gripper/SharedGraspingMsg.h>

// #define LEG_AXES \
//   ListofLegAxes(hip_adduction, "hip_adduction")    \
//   ListofLegAxes(hip_extension, "hip_extension")    \
//   ListofLegAxes(hip_roll, "hip_roll")              \
//   ListofLegAxes(knee_extension, "knee_extension")  \
//   ListofLegAxes(ankle_pitch, "ankle_pitch")  \
//   ListofLegAxes(ankle_roll, "ankle_roll")  \
//   ListofLegAxes(ankle_yaw, "ankle_yaw")  \
//   ListofLegAxes(NB_LEG_AXIS, "total_leg_joints") 
// #define ListofLegAxes(enumeration, names) enumeration,
// enum Leg_Axis : size_t { LEG_AXES };
// #undef ListofLegAxes
// extern const char *Leg_Axis_Names[];



#define PANDA_TOOL_AXES                                                                   \
  ListofPandaToolAxes(panda_joint1, "panda_joint1")    \
  ListofPandaToolAxes(panda_joint2, "panda_joint2")              \
  ListofPandaToolAxes(panda_joint3, "panda_joint3")    \
  ListofPandaToolAxes(panda_joint4, "panda_joint4")  \
  ListofPandaToolAxes(panda_joint5, "panda_joint5")  \
  ListofPandaToolAxes(panda_joint6, "panda_joint6")  \
  ListofPandaToolAxes(panda_joint7, "panda_joint7")  \
  ListofPandaToolAxes(panda_tool_wrist_open_angle, "panda_tool_wrist_open_angle")  \
  ListofPandaToolAxes(panda_tool_wrist_open_angle_mimic, "panda_tool_wrist_open_angle_mimic")  \
  ListofPandaToolAxes(NB_PANDA_TOOL_AXIS_FULL, "total_panda_tool_joints") 
#define ListofPandaToolAxes(enumeration, names) enumeration,
enum Tool_Axis : size_t { PANDA_TOOL_AXES };
#undef ListofPandaToolAxes
extern const char *Panda_Tool_Axis_Names[];

#define NB_PANDA_TOOL_AXIS_RED (NB_PANDA_TOOL_AXIS_FULL - 2)

const uint8_t NB_AXIS_POSITIONING = NB_PANDA_TOOL_AXIS_RED;

using namespace std;
using namespace Eigen;

class pandaSurgicalGripper {

public:
  enum Panda_Tool_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};


private:
  enum Control_Input {PLATFORM_INPUT = 2}; // LEG_INPUT = 1, };
  
  Control_Input _myInput;
  
  Panda_Tool_Name _pandaTool_id;

  // internal variables

  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_RED,1> _pandaToolJointsPosFiltered;
  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_FULL,1> _pandaToolJointsAll;
  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_FULL,1> _pandaToolJointsGet;
  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_FULL,1> _pandaToolJointsAllPrev;
  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_FULL,1> _pandaToolJointsAllSpeed;
  Eigen::Matrix<double, NB_PANDA_TOOL_AXIS_FULL, 1> _pandaToolJointsAllOffset;
  KDL::JntArray* _pandaToolJoints;
  KDL::JntArray* _pandaToolJointsPrev;
  KDL::JntArray* _pandaToolJointsFull;
  KDL::JntArray* _pandaToolJointsInit;
  KDL::JntArray* _pandaToolJointLims[NB_LIMS];
  KDL::JntArray* _pandaToolJointLimsAll[NB_LIMS];
  // KDL::JntArray* _legJointLims[NB_LIMS];
  KDL::JntArray* _platformJointLims[NB_LIMS];
  KDL::JntArray* _platformJointLimsDelta;

  Eigen::Matrix<double,NB_AXIS_WRENCH,1> _supportWrenchEigen;
  
  urdf::Model _myModel;
  // urdf::Model _legModel;
  urdf::Model _platformModel;
  KDL::Tree _myTree;
  std::vector<KDL::Segment> _mySegments;
  std::vector<KDL::Segment> _mySegmentsTip;
  std::vector<KDL::Frame> _myFrames;
  std::vector<KDL::Frame> _myFrames_tip;
  // KDL::ChainDynParam*  _myChainDyn;
  KDL::Chain _myPandaBaseToWristChain;
  KDL::Chain _myPandaWristToTipChain;
  KDL::Chain _myPandaBaseToTipChain;
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
  

  Eigen::Matrix<double,NB_PANDA_TOOL_AXIS_RED,NB_PANDA_TOOL_AXIS_RED> _weightedJointSpaceMassMatrix;
  Eigen::Matrix<double,NB_AXIS_WRENCH,NB_AXIS_WRENCH> _weightedTaskSpaceMassMatrix;
  
  
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _hAxisFilterPosValue;
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
  KDL::Frame _trocarBasePoseFrame;
  KDL::Frame _worldPandaLink0Frame;
    
  // Eigen::Matrix<double, NB_LEG_AXIS, 1> _legJoints;
  // Eigen::Matrix<double, NB_LEG_AXIS, 1> _legJointsOffset;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformJoints;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformVelocities;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformEfforts;
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _platformJointsOffset;

  // Publisher declaration
  // ros::Publisher _pubToolJointStates;
  ros::Publisher _pubToolJointCommands;
  ros::Publisher _pubToolTipPose;
  // ros::Subscriber _subLegJointStates;
  
  ros::Subscriber _subPlatformJointStates;
  ros::Subscriber _subPandaToolJointStates;
  //ros::Subscriber _subSharedGrasp;

  
  // Messages
  std_msgs::Float64MultiArray _msgJointCommands;
  //sensor_msgs::JointState _msgJointStates;
  //  geometry_msgs::PoseStamped _msgToolTipPose;
  //! boolean variables
  
  bool _flagRosControl;
  bool _mySolutionFound;
  bool _flagToolJointLimitsOffsetCalculated;
  bool _flagPlatformJointLimitsOffsetCalculated;
  bool _flagTrocarBasePoseRetrieved;
  bool _flagPandaLink0FrameRetrieved;
  bool _flagPantaToolJointInitRetrieved;
  volatile bool _flagPantaToolJointsRetrieved;


  // bool _flagLegJointLimitsOffsetCalculated;
  // bool _flagSharedGrasp;

    // bool _flagLegJointsConnected;
  bool _flagPlatformJointsConnected;

  bool _stop;

  std::mutex _mutex;
  static pandaSurgicalGripper *me;
  MatLP_Filterd* _hAxisFilterPos;
  LP_Filterd* _hAxisFilterGrasp;
  //! Dynamic Reconfigures

  // METHODS
public:
  pandaSurgicalGripper(ros::NodeHandle &n_1, double frequency,
                      pandaSurgicalGripper::Panda_Tool_Name tool_id_, urdf::Model model_);

  ~pandaSurgicalGripper();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  // void publishPandaToolJointStates();
  void publishPandaToolJointCommands();
  void publishPandaToolTipPose();
  
  void readPandaLink0Frame();
  void readTrocarBasePose();

  
  void performInverseKinematicsWithPlatform();
  void performChainForwardKinematics();
  void calculateDesiredFrame();

  // void readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg);
  void readPandaToolJoints(const sensor_msgs::JointState::ConstPtr &msg);
  // void readLegJoints(const sensor_msgs::JointState::ConstPtr &msg);
  void readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg);
  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __pandaSurgicalGripper_H__
