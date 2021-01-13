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

#include <gazebo_msgs/LinkStates.h>   //! to read the current state of the base link of the target sphere


#include <gazebo_msgs/SpawnModel.h> //! to re-spawn the the target sphere
#include <gazebo_msgs/DeleteModel.h>

#include "Eigen/Eigen"
#include <signal.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <boost/shared_ptr.hpp>

#include "custom_msgs/FootInputMsg_v5.h"
#include "custom_msgs/FootOutputMsg_v3.h"
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

const uint8_t NB_AXIS_POSITIONING = 4;

using namespace std;
using namespace Eigen;

#define TOOL_NAMES \
  ListofTools(RIGHT_TOOL, "right") \
  ListofTools(LEFT_TOOL, "left")   \
  ListofTools(ALL_TOOLS,"~")
#define ListofTools(enumeration, names) enumeration,
enum TrackID : size_t { TOOL_NAMES };
#undef ListofTools


#define TOOL_AXES                                                                   \
  ListofToolAxes(tool_pitch, "tool_pitch")    \
  ListofToolAxes(tool_roll, "tool_roll")              \
  ListofToolAxes(tool_yaw, "tool_yaw")    \
  ListofToolAxes(tool_insertion, "tool_insertion")  \
  ListofToolAxes(tool_wrist_open_angle, "tool_wrist_open_angle")  \
  ListofToolAxes(tool_wrist_open_angle_mimic, "tool_wrist_open_angle_mimic")  \
  ListofToolAxes(NB_TOOL_AXIS_FULL, "total_tool_joints") 
#define ListofToolAxes(enumeration, names) enumeration,
enum Tool_Axis : size_t { TOOL_AXES };
#undef ListofToolAxes
extern const char *Tool_Axis_Names[];

#define NB_TOOL_AXIS_RED (NB_TOOL_AXIS_FULL - 2)
const float SCALE_GAINS_LINEAR_POSITION  = 1.0f;
const float SCALE_GAINS_ANGULAR_POSITION = 1e-4f * RAD_TO_DEG;

class targetObject {

private:
  
  static targetObject *me;

  enum Marker_Color {NONE, RED, YELLOW, CYAN, WHITE};
  enum Target_Status {TARGET_NOT_REACHED, TARGET_REACHED, TARGET_GRASPED, TARGET_CHANGED};
  enum Action_State {A_POSITIONING, A_GRASPING, NB_ACTIONS};

  Target_Status _myStatus;
  
  Action_State _aState;

  TrackID _myTrackID; 
  // Eigen and Geometry

  Eigen::Vector3d    _toolTipPosition;
  Eigen::Vector3d    _toolTipPositionPrev;
  Eigen::Vector3d    _toolTipSpeed;
  Eigen::Vector3d    _toolTipSpeedPrev;
  Eigen::Vector3d    _toolTipAcc;
  Eigen::Vector3d    _toolTipAccPrev;

  Eigen::Vector3d    _toolTipJerk;
  Eigen::Vector3d    _toolTipPositionWRTTorso;
  Eigen::Quaterniond _toolTipQuaternion;
  Eigen::Quaterniond _toolTipQuaternionWRTTorso;
  Eigen::Quaterniond _toolTipQuaternionPrev;
  Eigen::Matrix3d    _toolTipRotationMatrix;
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointSpeed;
  Eigen::Matrix<double, NB_TOOL_AXIS_FULL,1> _toolJointPosition;
  geometry_msgs::Wrench _footBaseWorldForce;

  double _hapticAxisFilterPos;
  double _hapticAxisFilterGrasp;
  
  MatLP_Filterd* _toolTipPositionFilter;
  MatLP_Filterd* _toolTipSpeedFilter;
  MatLP_Filterd* _toolTipAccFilter;
  MatLP_Filterd* _toolTipJerkFilter;

   
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointPosition;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointVelocity;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointEffortD;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointEffortRef;
  Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointEffortM;
  // Eigen::Matrix<double, NB_PLATFORM_AXIS,1> _platformJointStates_prev;
  
  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _legGravityCompTorques;

  Eigen::Vector3d    _trocarPosition;
  Eigen::Quaterniond _trocarQuaternion;
  Eigen::Matrix3d    _trocarRotationMatrix;


  Eigen::Vector3d    _myPositionSpawn;
  Eigen::Quaterniond _myQuaternionSpawn;
  Eigen::Matrix3d    _myRotationMatrixSpawn;

  Eigen::Vector3d    _myPositionCurrent;
  Eigen::Quaterniond _myQuaternionCurrent;
  Eigen::Matrix3d    _myRotationMatrixCurrent;
  Eigen::Vector3d _maxLimsTarget;

  double _vibrationGrasping;
  double _impedanceGrasping;

  Eigen::Matrix<double, NB_PLATFORM_AXIS, 1> _hapticTorques;

  double _precisionPos, _precisionAng,_precisionGrasp;
  
  double _myThreshold;

  double _myRandomAngle;
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
  std::string _myModelXml;
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
  geometry_msgs::TransformStamped _msgtargetObjectTransform;
  
  custom_msgs_gripper::SharedGraspingMsg _msgSharedGrasp;

  gazebo_msgs::SpawnModel _srvGzSpawnModel;
  gazebo_msgs::DeleteModel _srvGzDeleteModel;
  
  ros::Publisher _pubSharedGrasp;
  ros::Publisher _pubTargetReachedSphere;
  ros::Publisher _pubRvizTargetMarker;
  ros::Publisher _pubFootInput;
  ros::ServiceClient _clientSpawnNewTargetAtPos;
  ros::ServiceClient _clientDeleteOldTarget;


  custom_msgs::FootInputMsg_v5 _msgFootInput;

  visualization_msgs::Marker _msgTargetReachedSphere;
  visualization_msgs::Marker _msgRvizTarget;

  gazebo_msgs::LinkStates _msgGazeboLinkStates;


  ros::Subscriber _subGazeboLinkStates;
  ros::Subscriber _subSharedGrasp;
  ros::Subscriber _subLegGravityCompTorques;
  ros::Subscriber _subToolTipPose;
  ros::Subscriber _subToolJointStates;
  ros::Subscriber _subPlatformJointStates;
  ros::Subscriber _subFootPlatform;
  ros::Subscriber _subForceFootRestWorld;
  ros::Subscriber _subPlatformROSInput;
  ros::Subscriber _subUnbiasedJointTorques;

  vibrator* _myVibrator;
  smoothSignals* _mySmoothSignals;
  
  //KDL 

     KDL::Tree _myTree;
  // KDL::Frame _toolTipFrame;
  // KDL::Frame _trocarFrame;
  // KDL::Frame _myFrame;
  
  //! boolean variables
  bool  _flagFootBaseForceConnected;
  bool  _flagLegGravityTorquesConnected;
  bool  _flagPlatformJointsConnected;
  bool  _flagToolJointsConnected;
  bool  _flagToolTipTFConnected;
  bool  _flagTrocarTFConnected;
  bool  _flagTargetReached;
  bool  _flagTargetReachedOpen;
  bool  _flagTargetGrasped;
  bool  _flagTargetSpawned;
  bool  _flagRecordingStarted;

  bool  _stop;

  volatile bool _flagGazeboLinkStateRead;


  // METHODS
public:
  targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_);

  ~targetObject();

  bool init();
  void run();

private:
  //! ROS METHODS

  bool allSubscribersOK();
  std::vector<std::pair<std::string,std::vector<float>>> 
  readTargetPointsCSV(std::string filename);

  std::vector<std::pair<std::string, std::vector<float>>>
  _targetsXYZ;

  int NB_TARGETS;
  int _nTarget, _xTarget;


  void readGazeboLinkStates(const gazebo_msgs::LinkStates::ConstPtr &msg);

  void readToolState(const sensor_msgs::JointState::ConstPtr &msg);
  
  void readPlatformState(const sensor_msgs::JointState::ConstPtr &msg);

  void readLegGravityCompTorques(const custom_msgs::FootInputMsg_v5::ConstPtr &msg);

  void readFIOutput(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg);

  void readForceFootRestWorld(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  void readUnbiasedJointTorques(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg);
  
  void readGtraTorques(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg);
  
  void readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg);
  
  void computeToolTipDerivatives();

  void readTFTool();
  void readTFTrocar();
  void writeTFtargetObject();

  void computetargetObjectPose();
  void evaluateTarget();

  void publishTargetReachedSphere(int32_t action_, Marker_Color color_, double delay_);
  void publishMarkerTargetRviz(int32_t action_, Marker_Color color_, double delay_);
  void recordStatistics();
  
  
  void getGazeboTargetCurrentPos();
  
  bool gazeboDeleteModel();
  bool gazeboSpawnModel();



  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __targetObject_H__

