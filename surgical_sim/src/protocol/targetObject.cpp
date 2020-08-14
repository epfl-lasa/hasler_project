#include "Utils_math.h"
#include "targetObject.h"
#include "tf_conversions/tf_eigen.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


#define ListofTools(enumeration, names) names,
char const *Tools_Names[]{TOOL_NAMES};
#undef ListofTools

targetObject *targetObject::me = NULL;

targetObject::targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_)
    : _n(n_1), _loopRate(frequency), _dt(1.0f / frequency),
    _myModel(model_), _myName(name_){

   me = this;
  _stop = false;

  
  NB_TARGETS=0;
  _nTarget=0;
  for (unsigned int tool=0; tool<NB_TOOLS; tool++)
  {
    _toolTipPosition[tool].setZero();
    _trocarPosition[tool].setZero();
    
    _toolTipQuaternion[tool].setIdentity();
    _trocarQuaternion[tool].setIdentity();
   
    _toolTipRotationMatrix[tool].setIdentity();
    _trocarRotationMatrix[tool].setIdentity();
  
    _flagTargetReached[tool] = false;
    _flagTrocarTFConnected[tool] = false;
    _flagToolTipTFConnected[tool] = false;
  }

  _myPosition.setZero();
  _myQuaternion.setIdentity();
  _myRotationMatrix.setIdentity();
  
  std::string trackingMode;
  _startDelayForCorrection = ros::Time::now();
  
  if (!_n.getParam("trackingMode", trackingMode))
  { 
      ROS_ERROR("No indicaton of tracking mode (right, left, both) was done"); 
  }

  _myTrackMode = RIGHT_TOOL;
  
  if (trackingMode == "right") {
    _myTrackMode = RIGHT_TOOL;
  }
  
  if (trackingMode=="left")
  {
    _myTrackMode = LEFT_TOOL;
  }

  if (trackingMode == "all") {
    _myTrackMode = ALL_TOOLS;
  }



  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("Failed to construct kdl tree");
    _stop = true;
  }
    KDL::Vector grav_vector(0.0, 0.0, (double)GRAVITY);
    _tfListener = new tf2_ros::TransformListener(_tfBuffer);
    _tfBroadcaster = new tf2_ros::TransformBroadcaster;

    _targetsXYZ = readTargetPointsCSV("targets.csv");
}

targetObject::~targetObject() { me->_n.shutdown(); }

bool targetObject::init() //! Initialization of the node. Its datatype
                          //! (bool) reflect the success in
                          //! initialization

{
  // Subscriber definitions
  signal(SIGINT, targetObject::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("The target spawner is about to start ");
    return true;
  }
  else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void targetObject::stopNode(int sig) { me->_stop = true; }

void targetObject::run() {

  while (!_stop) {
    readTFTool(_myTrackMode);
    readTFTrocar(_myTrackMode);
    if (true) {
      generateNextTarget();
      computeTargetObjectPose();
      writeTFTargetObject();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("The target spawner stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void targetObject::readTFTool(unsigned int n_) {

  if (n_==ALL_TOOLS)
  {
    for (unsigned int i = 0; i < NB_TOOLS; i++) {
      readTFTool(i);
    }
  }
  else
  {
    static int count = 0;
    std::string original_frame;
    std::string destination_frame;

    destination_frame = std::string(Tools_Names[n_]) + "/tool_tip_link_ee";
    original_frame = "torso_link";

    geometry_msgs::TransformStamped toolTipTransform_;

    try 
    {
      toolTipTransform_ = _tfBuffer.lookupTransform(
          original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

      if (_flagToolTipTFConnected[n_]) {

        tf::vectorMsgToEigen(toolTipTransform_.transform.translation,
                             _toolTipPosition[n_]);
        tf::quaternionMsgToEigen(toolTipTransform_.transform.rotation,
                                 _toolTipQuaternion[n_]);
        _toolTipRotationMatrix[n_] =
            _toolTipQuaternion[n_].normalized().toRotationMatrix();
      }
      _flagToolTipTFConnected[n_] = true;

    } 
    catch (tf2::TransformException ex) 
    {
      if (count > 2) {
        ROS_ERROR("%s", ex.what());
        count = 0;
      } else {
        count++;
      }
      ros::Duration(1.0).sleep();
    }
  }
}

void targetObject::readTFTrocar(unsigned int n_) {

  if (n_ == ALL_TOOLS) {
    for (unsigned int i = 0; i < NB_TOOLS; i++) {
      readTFTrocar(i);
    }
  } else {
    static int count = 0;
    std::string original_frame;
    std::string destination_frame;

    destination_frame = std::string(Tools_Names[n_]) + "/trocar_link";
    original_frame = "torso_link";

    geometry_msgs::TransformStamped trocarTransform_;

    try {
      trocarTransform_ = _tfBuffer.lookupTransform(
          original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

      if (_flagTrocarTFConnected[n_]) {

        tf::vectorMsgToEigen(trocarTransform_.transform.translation,
                             _trocarPosition[n_]);
        tf::quaternionMsgToEigen(trocarTransform_.transform.rotation,
                                 _trocarQuaternion[n_]);
        _trocarRotationMatrix[n_] =
            _trocarQuaternion[n_].normalized().toRotationMatrix();
      }
      _flagTrocarTFConnected[n_] = true;

    } catch (tf2::TransformException ex) {
      if (count > 2) {
        ROS_ERROR("%s", ex.what());
        count = 0;
      } else {
        count++;
      }
      ros::Duration(1.0).sleep();
    }
  }
}



void targetObject::computeTargetObjectPose(){

 _myPosition << _targetsXYZ[0].second[_nTarget], _targetsXYZ[2].second[_nTarget], -_targetsXYZ[1].second[_nTarget];
//  cout<<_myPosition.transpose()<<endl; 
  Eigen::Vector3d targetTrocarDistance = _trocarPosition[RIGHT_TOOL]-_myPosition;
  
  if (targetTrocarDistance.norm() > FLT_EPSILON) {
    _myRotationMatrix = Utils_math<double>::rodriguesRotation(
    Eigen::Vector3d(0.0, 0.0, 1.0), targetTrocarDistance);
  }
  _myQuaternion = Eigen::Quaternion<double>(_myRotationMatrix);
}


void targetObject::generateNextTarget()
{
  double posError = (_myPosition - _toolTipPosition[RIGHT_TOOL]).norm();
  double rotError = 1.0-cos(_myQuaternion.angularDistance(_toolTipQuaternion[RIGHT_TOOL]));
  //std::cout << rotError << endl;
  if (posError < 0.020 && rotError < (1-cos(10.0*DEG_TO_RAD))) {
    if (!_flagTargetReached[RIGHT_TOOL])
    {
      _startDelayForCorrection = ros::Time::now(); 
      _flagTargetReached[RIGHT_TOOL]=true;
    }
  }
  
  if (_flagTargetReached[RIGHT_TOOL])
  {
    if ((ros::Time::now() - _startDelayForCorrection).toSec() > 5.0)
    { 
    _nTarget = int(rand() % NB_TARGETS-1); // Generate new target randomly
    _flagTargetReached[RIGHT_TOOL] =false;
    _startDelayForCorrection=ros::Time::now();
    ROS_INFO("New target generated!");
    }
  }

}

void targetObject::writeTFTargetObject(){
  _msgTargetObjectTransform.child_frame_id="target_object_link";
  _msgTargetObjectTransform.header.frame_id="torso_link";
  _msgTargetObjectTransform.header.stamp = ros::Time::now();

  tf::vectorEigenToMsg(_myPosition,_msgTargetObjectTransform.transform.translation);
  tf::quaternionEigenToMsg(_myQuaternion,_msgTargetObjectTransform.transform.rotation); 

  _tfBroadcaster->sendTransform(_msgTargetObjectTransform);
}

std::vector<std::pair<std::string, std::vector<float>>> targetObject::readTargetPointsCSV(std::string filename)
 {
    // Reads a CSV file into a vector of <string, vector<float>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(ros::package::getPath("surgical_sim") + "/data/" + filename);

    // Make sure the file is open
    if (!myFile.is_open())
      throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    float val;

    // Read the column names
    if (myFile.good()) {
      // Extract the first line in the file
      std::getline(myFile, line);

      // Create a stringstream from line
      std::stringstream ss(line);

      // Extract each column name
      while (std::getline(ss, colname, ',')) {

        // Initialize and add <colname, float vector> pairs to result
        result.push_back({colname, std::vector<float>{}});
      }
    }

    // Read data, line by line
    while (std::getline(myFile, line)) {
      // Create a stringstream of the current line
      std::stringstream ss(line);

      // Keep track of the current column index
      int colIdx = 0;

      // Extract each integer
      while (ss >> val) {

        // Add the current integer to the 'colIdx' column's values vector
        result.at(colIdx).second.push_back(val);

        // If the next token is a comma, ignore it and move on
        if (ss.peek() == ',')
          ss.ignore();

        // Increment the column index
        colIdx++;
      }
      NB_TARGETS++;
    }

    // Close file
    myFile.close();

    return result;
}


