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

extern const char *Tools_Names[];



class targetObject {

private:
  
  static targetObject *me;

  TrackMode _myTrackMode; 
  // Eigen and Geometry

  Eigen::Vector3d    _toolTipPosition[NB_TOOLS];
  Eigen::Quaterniond _toolTipQuaternion[NB_TOOLS];
  Eigen::Matrix3d    _toolTipRotationMatrix[NB_TOOLS];

  Eigen::Vector3d    _trocarPosition[NB_TOOLS];
  Eigen::Quaterniond _trocarQuaternion[NB_TOOLS];
  Eigen::Matrix3d    _trocarRotationMatrix[NB_TOOLS];

  Eigen::Vector3d    _myPosition;
  Eigen::Quaterniond _myQuaternion;
  Eigen::Matrix3d    _myRotationMatrix;

  // std variables

  std::ofstream _myFile;
  std::string _myName;
  std::mutex _mutex;
  
  // ROS
  
  // urdf
  urdf::Model _myModel;
  // ros variables
  ros::NodeHandle _n;
  ros::Rate _loopRate;
  ros::Time _startDelayForCorrection;
  float _dt;
  //! topics, actions, services, listeners and broadcasters
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  tf2_ros::TransformBroadcaster* _tfBroadcaster;
  geometry_msgs::TransformStamped _msgTargetObjectTransform;
 
  
  //KDL 

     KDL::Tree _myTree;
  // KDL::Frame _toolTipFrame;
  // KDL::Frame _trocarFrame;
  // KDL::Frame _myFrame;
  
  //! boolean variables

  bool  _flagToolTipTFConnected[NB_TOOLS];
  bool  _flagTrocarTFConnected[NB_TOOLS];
  bool  _flagTargetReached[NB_TOOLS];

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
  int _nTarget;

  void readTFTool(unsigned int n_);
  void readTFTrocar(unsigned int n_);
  void writeTFTargetObject();

  void computeTargetObjectPose();
  void generateNextTarget();

  //! OTHER METHODS
  static void stopNode(int sig);
};
#endif // __targetObject_H__