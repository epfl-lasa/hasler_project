#ifndef __pandaController_H__
#define __pandaController_H__

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

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include "MatLP_Filterd.h"
#include "LP_Filterd.h"
#include <geometry_msgs/PoseStamped.h>


#define PANDA_AXES                               \
  ListofPandaAxes(panda_joint1, "panda_joint1")   \
  ListofPandaAxes(panda_joint2, "panda_joint2")   \
  ListofPandaAxes(panda_joint3, "panda_joint3")   \
  ListofPandaAxes(panda_joint4, "panda_joint4")  \
  ListofPandaAxes(panda_joint5, "panda_joint5")  \
  ListofPandaAxes(panda_joint6, "panda_joint6")  \
  ListofPandaAxes(panda_joint7, "panda_joint7")  \
  ListofPandaAxes(NB_PANDA_AXIS, "total_panda_joints") 
#define ListofPandaAxes(enumeration, names) enumeration,
enum Tool_Axis : size_t { PANDA_AXES };
#undef ListofPandaAxes
extern const char *Panda_Axis_Names[];

using namespace std;
using namespace Eigen;

class pandaController {

public:
  enum Panda_Name { UNKNOWN = 0, RIGHT = 1, LEFT = 2};

private:

  Panda_Name _panda_id;

  // internal variables

  KDL::JntArray* _pandaJoints;
  KDL::JntArray* _pandaJointsGet;
  KDL::JntArray* _pandaJointsInit;
  KDL::JntArray* _pandaJointLims[NB_LIMS];

  urdf::Model _myModel;
  KDL::Tree _myTree;
  std::vector<KDL::Segment> _mySegments;
  std::vector<KDL::Frame> _myFrames;
  // KDL::ChainDynParam*  _myChainDyn;
  KDL::Chain _myPandaBaseToWristChain;
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
  

  // ros variables
  ros::NodeHandle _n;
  ros::Rate _loopRate;
  float _dt;

  //! subscribers and publishers declaration
  // Subscribers declarations
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  
  KDL::Frame _desiredTargetFrame;
  KDL::Frame _worldPandaLink0Frame;
    
  // Publisher declaration
  ros::Publisher _pubPandaJointCommands;
  
  ros::Subscriber _subPandaJointStates;
  
  // Messages
  std_msgs::Float64MultiArray _msgJointCommands;
  //! boolean variables
  
  bool _mySolutionFound;
  bool _flagToolFrameRetrieved;
  bool _flagPandaLink0FrameRetrieved;
  volatile bool _flagPandaJointsRetrieved;
  volatile bool _flagPandaJointInitRetrieved;

  bool _stop;

  std::mutex _mutex;
  static pandaController *me;
  //! Dynamic Reconfigures

  // METHODS
public:
  pandaController(ros::NodeHandle &n_1, double frequency,
                      pandaController::Panda_Name tool_id_, urdf::Model model_);

  ~pandaController();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  void publishPandaJointCommands();
  
  void readToolBaseFrame();
  void readPandaLink0Frame();
  
  void performInverseKinematics();
  void performChainForwardKinematics();
  void calculateDesiredFrame();

  void readPandaJoints(const sensor_msgs::JointState::ConstPtr &msg);
  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __pandaController_H__
