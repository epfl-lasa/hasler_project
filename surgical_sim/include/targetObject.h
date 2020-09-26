#ifndef __targetObject_H__
#define __targetObject_H__

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
#include "PIDd.h"


const uint8_t NB_AXIS_POSITIONING = 4;

using namespace std;
using namespace Eigen;

#define TOOL_NAMES \
  ListofTools(RIGHT_TOOL, "right") \
  ListofTools(LEFT_TOOL, "left")   \
  ListofTools(ALL_TOOLS,"~")
#define ListofTools(enumeration, names) enumeration,
enum TrackMode : size_t { TOOL_NAMES };
#undef ListofTools
#define NB_TOOLS ALL_TOOLS


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

class targetObject {

private:
  
  static targetObject *me;

  enum Marker_Color {NONE, RED, YELLOW, CYAN};
  enum Target_Status {TARGET_NOT_REACHED, TARGET_REACHED_AND_GRASPED, TARGET_CHANGED};
  enum Action_State {A_POSITIONING, A_GRASPING, NB_ACTIONS};

  Target_Status _myStatus;
  
  Action_State _aState[NB_TOOLS];
  Action_State _aStateNext[NB_TOOLS];

  TrackMode _myTrackMode; 
  // Eigen and Geometry

  Eigen::Vector3d    _toolTipPosition[NB_TOOLS];
  Eigen::Vector3d    _toolTipPositionPrev[NB_TOOLS];
  Eigen::Quaterniond _toolTipQuaternion[NB_TOOLS];
  Eigen::Quaterniond _toolTipQuaternionPrev[NB_TOOLS];
  Eigen::Matrix3d    _toolTipRotationMatrix[NB_TOOLS];
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _devToolJointStates[NB_TOOLS];
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointStates[NB_TOOLS];
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointStates_prev[NB_TOOLS];
  geometry_msgs::Wrench _footBaseWorldForce[NB_TOOLS];

  LP_Filterd _hapticAxisFilter;
 
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointStates[NB_TOOLS];
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointStates_prev[NB_TOOLS];
  
  Eigen::Vector3d    _trocarPosition[NB_TOOLS];
  Eigen::Quaterniond _trocarQuaternion[NB_TOOLS];
  Eigen::Matrix3d    _trocarRotationMatrix[NB_TOOLS];

  Eigen::Vector3d    _myPosition;
  Eigen::Quaterniond _myQuaternion;
  Eigen::Matrix3d    _myRotationMatrix;

  Eigen::Vector3d _maxLimsTarget;

  double _errorPenetration[NB_TOOLS];
  double _vibrationGrasping[NB_TOOLS];
  double _impedanceGrasping[NB_TOOLS];
  double _errorPenetration_prev[NB_TOOLS];
  double _devErrorPenetration[NB_TOOLS];

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _hapticTorques[NB_TOOLS];

  double _precisionPos[NB_TOOLS], _precisionAng[NB_TOOLS];

  
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kpPosition[NB_TOOLS];
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kiPosition[NB_TOOLS];
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _kdPosition[NB_TOOLS];

  LP_Filterd _kpPositionFilter[NB_TOOLS][NB_AXIS_POSITIONING];
  LP_Filterd _kiPositionFilter[NB_TOOLS][NB_AXIS_POSITIONING];
  LP_Filterd _kdPositionFilter[NB_TOOLS][NB_AXIS_POSITIONING];

  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlRef[NB_TOOLS];
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlIn[NB_TOOLS];
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlInPrev[NB_TOOLS];
  Eigen::Matrix<double, NB_AXIS_POSITIONING,1> _posCtrlOut[NB_TOOLS];

  PIDd* _pidPosition[NB_TOOLS][NB_AXIS_POSITIONING];
  PIDd* _pidGrasping[NB_TOOLS];

  double _graspCtrlRef[NB_TOOLS];
  double _graspCtrlIn[NB_TOOLS];
  double _graspCtrlInPrev[NB_TOOLS];
  double _graspCtrlOut[NB_TOOLS];

  double _kpGrasping[NB_TOOLS];
  double _kiGrasping[NB_TOOLS];
  double _kdGrasping[NB_TOOLS];

  // std variables

  std::ofstream _myFile;
  std::string _myName;
  std::mutex _mutex;


   //! Variables for Logging    
    std::string _subjectID;
		std::ofstream _statsOutputFile;   
  
  // ROS
  
  // urdf
  urdf::Model _myModel;
  // ros variables
  ros::NodeHandle _n;
  ros::Rate _loopRate;
  ros::Time _startDelayForCorrection;
  ros::Time _startingTime;
  float _dt;
  //! topics, actions, services, listeners and broadcasters
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  tf2_ros::TransformBroadcaster* _tfBroadcaster;
  geometry_msgs::TransformStamped _msgTargetObjectTransform;
  

  
  ros::Publisher _pubTargetReachedSphere;
  ros::Publisher _pubFootInput[NB_TOOLS];


  custom_msgs::FootInputMsg_v5 _msgFootInput[NB_TOOLS];

  visualization_msgs::Marker _msgTargetReachedSphere;


  ros::Subscriber _subToolJointStates[NB_TOOLS];
  ros::Subscriber _subPlatformJointStates[NB_TOOLS];
  ros::Subscriber _subForceFootRestWorld[NB_TOOLS];

  vibrator* _myVibrator[NB_TOOLS];
  
  //KDL 

     KDL::Tree _myTree;
  // KDL::Frame _toolTipFrame;
  // KDL::Frame _trocarFrame;
  // KDL::Frame _myFrame;
  
  //! boolean variables
  bool _flagFootBaseForceConnected[NB_TOOLS];
  bool  _flagPlatformJointsConnected[NB_TOOLS];
  bool  _flagToolJointsConnected[NB_TOOLS];
  bool  _flagToolTipTFConnected[NB_TOOLS];
  bool  _flagTrocarTFConnected[NB_TOOLS];
  bool  _flagTargetReached[NB_TOOLS];
  bool  _flagTargetGrasped[NB_TOOLS];
  bool  _flagRecordingStarted;
  bool _flagHapticGrasping;
  bool _flagSharedGrasping;

  bool  _stop;


  // METHODS
public:
  targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_);

  ~targetObject();

  bool init();
  void run();

private:
  //! ROS METHODS

  // bool allSubscribersOK();
  std::vector<std::pair<std::string,std::vector<float>>> 
  readTargetPointsCSV(std::string filename);

  std::vector<std::pair<std::string, std::vector<float>>>
  _targetsXYZ;

  int NB_TARGETS;
  int _nTarget, _xTarget;

  void readToolState(const sensor_msgs::JointState::ConstPtr &msg,unsigned int n_);
  
  void readPlatformState(const sensor_msgs::JointState::ConstPtr &msg,unsigned int n_);

  void readForceFootRestWorld(const geometry_msgs::WrenchStamped::ConstPtr &msg,unsigned int n_);
  void readTFTool(unsigned int n_);
  void estimateActionState(unsigned int n_);
  void readTFTrocar(unsigned int n_);
  void writeTFTargetObject();
  void doSharedControl(unsigned int n_);

  void computeTargetObjectPose(unsigned int n_);
  void evaluateTarget(unsigned int n_);

  void publishTargetReachedSphere(int32_t action_, Marker_Color color_, double delay_);
  void publishFootInput(int n_);
  void recordStatistics();

  

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __targetObject_H__
