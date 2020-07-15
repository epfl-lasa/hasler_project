#ifndef __leg_robot_H__
#define __leg_robot_H__

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


#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/WrenchStamped.h"
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
  
  KDL::JntArray* _legJoints;
  KDL::JntArray*_legJointsInit;
  KDL::JntArray* _legJointLims[NB_LIMS];
  KDL::JntArray* _gravityTorques;

  // ros variables
  urdf::Model _myModel;
  KDL::Tree _myTree;
  std::vector<KDL::Segment> _mySegments;
  std::vector<KDL::Frame> _myFrames;
  KDL::ChainDynParam*  _myChainDyn;
  KDL::Chain _myFootBaseChain;
  KDL::Vector _cogLeg;
  KDL::ChainFkSolverPos_recursive* _myFKSolver;
  KDL::ChainIkSolverVel_pinv* _myVelIKSolver;
  KDL::ChainIkSolverPos_NR_JL* _myPosIkSolver;

  bool _mySolutionFound;

  ros::NodeHandle _n;
  ros::Rate _loopRate;
  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  KDL::Frame _footPosFrame;
  KDL::FrameVel _footVelFrame;
  Eigen::Vector3d _footPosition;
  Eigen::Vector3d _footEuler;
  Eigen::Quaterniond _footQuaternion;
  Eigen::Matrix3d _footRotationMatrix;
  Eigen::Vector3d _netCoG;
  Eigen::Matrix<double, NB_AXIS_WRENCH,1> _wrenchGravityFootBase;
  Eigen::Vector3d _gravityVector;
  
  // Publisher declaration
  ros::Publisher _pubLegJointStates;
  ros::Publisher _pubNetCoG;
  ros::Publisher _pubFootBaseWrench; //! To the foot variable synchornizer


  // Messages

  sensor_msgs::JointState _msgJointStates;
  geometry_msgs::PointStamped _msgNetCoG;
  geometry_msgs::WrenchStamped _msgFootBaseWrench;

  //! boolean variables

  bool _flagFootPoseConnected;
  bool _stop;

  std::mutex _mutex;
  static legRobot *me;

  //! Dynamic Reconfigures

  // METHODS
public:
  legRobot(ros::NodeHandle &n_1, double frequency,
                      legRobot::Leg_Name leg_id_, urdf::Model model_);

  ~legRobot();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void publishLegJointStates();
  void readFootBasePose();
  void performInverseKinematics();
  void processAngles(Eigen::MatrixXd ikSolutions);
  void computeGravityTorque(); //! effort in each leg joint
  void computeFootBaseGravityWrench();
  void publishNetCoG();
  void publishFootBaseGravityWrench();

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __leg_robot_H__