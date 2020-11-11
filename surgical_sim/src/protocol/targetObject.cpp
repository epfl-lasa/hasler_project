#include "Utils_math.h"
#include "targetObject.h"
#include "tf_conversions/tf_eigen.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include "boost/date_time/gregorian/gregorian.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
// #include "boost/date_time/gregorian_types.hpp"

const float conversion_factor[] = {1.0, 1.0, DEG_TO_RAD, DEG_TO_RAD, DEG_TO_RAD};

using namespace boost::gregorian;
#define ListofToolAxes(enumeration, names) names,
char const *Tool_Axis_Names[]{TOOL_AXES};
#undef ListofToolAxes

#define ListofTools(enumeration, names) names,
char const *Tools_Names[]{TOOL_NAMES};
#undef ListofTools

char const *Tools_Names2[]{"Right","Left"};

#define DELAY_SEC 3.0



  // _desiredTargetFrame.p.data[0] = Utils_math<double>::map( _platformJoints(p_x),
  //                                                         -_platformJointLimsDelta->data(p_x), _platformJointLimsDelta->data(p_x), 
  //                                                         -0.15,0.15);



  // _desiredTargetFrame.p.data[1] = Utils_math<double>::map( _platformJoints(p_y),
  //                                                         -_platformJointLimsDelta->data(p_y), _platformJointLimsDelta->data(p_y), 
  //                                                         -0.15,0.15);
  
  // _desiredTargetFrame.p.data[2] = -0.12 + Utils_math<double>::map( _platformJoints(p_pitch),
  //                                                         -_platformJointLimsDelta->data(p_pitch), 0.5*_platformJointLimsDelta->data(p_pitch), 
  //                                                         -0.15,0.15) ;

  //  _toolJoints->data(tool_yaw) = -Utils_math<double>::map( (_platformJoints(p_yaw) - _platformJointsOffset(p_yaw)) , 
  //                                 -35*DEG_TO_RAD, 35*DEG_TO_RAD, 
  //                                 _toolJointLimsAll[L_MIN]->data(tool_yaw), _toolJointLimsAll[L_MAX]->data(tool_yaw));


  //  _toolJointsAll(tool_wrist_open_angle) = Utils_math<double>::map( (-_platformJoints(p_roll) - _platformJointsOffset(p_roll)),
  //                                         -0.5*_platformJointLimsDelta->data(p_roll), 0.5*_platformJointLimsDelta->data(p_roll), 
  //                                          _toolJointLimsAll[L_MIN]->data(tool_wrist_open_angle), _toolJointLimsAll[L_MAX]->data(tool_wrist_open_angle));



const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const float SAMPLING_TIME = 10; // 100Hz

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_roll,p_yaw};

const double scaleFoot[] = {Axis_Limits[0]/0.15,Axis_Limits[1]/0.15,(1.5*Axis_Limits[2])/(2*0.15),35.0*DEG_TO_RAD/(1.5*M_PI),20.5*DEG_TO_RAD/30*DEG_TO_RAD}; // X, Y, Z, YAW, ROLL;


targetObject *targetObject::me = NULL;

  double magnitude_vib = 2.0;
  double decayRate_vib = 3.0;
  double frequency_vib = 32.6;

targetObject::targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_)
    : _n(n_1), _loopRate(frequency), _dt(1.0f / frequency),
    _myModel(model_), _myName(name_){

   me = this;
  _stop = false;

   
  Eigen::Vector3d alphaPosition;
  alphaPosition.setConstant(0.6);

  Eigen::Vector3d alphaSpeed;
  alphaSpeed.setConstant(0.95);

  Eigen::Vector3d alphaAcc;
  alphaAcc.setConstant(0.95);

  Eigen::Vector3d alphaJerk;
  alphaJerk.setConstant(0.95);


  for (size_t i = 0; i < NB_TOOLS; i++)
  {
  _toolTipPositionFilter[i] = new MatLP_Filterd(alphaPosition);
  _toolTipSpeedFilter[i] = new MatLP_Filterd(alphaSpeed);
  _toolTipAccFilter[i] = new MatLP_Filterd (alphaAcc);
  _toolTipJerkFilter[i]= new MatLP_Filterd (alphaJerk);
  _myThreshold[i]=0.0;
  _vibrationGrasping[i] = 0.0;
  _impedanceGrasping[i] = 0.0;
  _hapticTorques[i].setZero();
  _precisionAng[i]=0.0;
  _precisionPos[i]=0.0;  
  _precisionGrasp[i]=0.0; 
  _hapticAxisFilterPos[i]=1.0; 
  }

  _myRandomAngle=0.0;

  NB_TARGETS=0;
  
  _xTarget=0;  _nTarget=0;

  
  for (unsigned int tool=0; tool<NB_TOOLS; tool++)
  {
    _toolTipPosition[tool].setZero();
    _toolTipPositionWRTTorso[tool].setZero();
    _toolTipPositionPrev[tool].setZero();
    _trocarPosition[tool].setZero();

    _toolJointPosition[tool].setZero();  
    _toolJointSpeed[tool].setZero();  
    _platformJointPosition[tool].setZero();
    _platformJointVelocity[tool].setZero();
    _platformJointEffortD[tool].setZero();
    _platformJointEffortRef[tool].setZero();
    _legGravityCompTorques[tool].setZero();
    _platformJointEffortM[tool].setZero();
    // _platformJointStates[tool].setZero();
    // _platformJointStates_prev[tool].setZero(); 
    _toolTipQuaternion[tool].setIdentity();
    _toolTipQuaternionWRTTorso[tool].setIdentity();
    _toolTipQuaternionPrev[tool].setIdentity();
    _trocarQuaternion[tool].setIdentity();
   
    _toolTipRotationMatrix[tool].setIdentity();
    _trocarRotationMatrix[tool].setIdentity();

    _flagTargetReached[tool] = false;
    _flagTargetReachedOpen[tool] = false;
    _flagTargetGrasped[tool] = false;
    _flagTrocarTFConnected[tool] = false;
    _flagToolTipTFConnected[tool] = false;
    _flagFootBaseForceConnected[tool]=false;
    _flagToolJointsConnected[tool]=false;
    _flagLegGravityTorquesConnected[tool]=false;
    _aState[tool]=A_POSITIONING;
  }


    _flagRecordingStarted=false;

  _myPosition.setZero();
  _myQuaternion.setIdentity();
  _myRotationMatrix.setIdentity();
  _maxLimsTarget.setZero();
  
  std::string trackingMode;
  _startDelayForCorrection = ros::Time::now();
  _startingTime = ros::Time::now();
  
  if (!_n.getParam("trackingMode", trackingMode))
  { 
      ROS_ERROR("No indicaton of tracking mode (right, left, both) was done"); 
  }

   
  if (!_n.getParam("subjectID", _subjectID))
  { 
      ROS_ERROR("No indicaton of the subject ID (e.g. sXX) was done"); 
      _subjectID = "s0";
  }

    _myTrackMode = RIGHT_TOOL;
  

  if (trackingMode.compare("right") == 0) {
      _myTrackMode = RIGHT_TOOL;
    } else if (trackingMode.compare("left") == 0) {
      _myTrackMode = LEFT_TOOL;
    } else if (trackingMode.compare("all") == 0) {
      _myTrackMode = ALL_TOOLS;
    } else {
      ROS_ERROR("You didn't enter a tracking Mode");
      _stop=true;
    }


  _myStatus = TARGET_NOT_REACHED;

  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("Failed to construct kdl tree");
    _stop = true;
  }
    KDL::Vector grav_vector(0.0, 0.0, GRAVITY);

    _tfListener = new tf2_ros::TransformListener(_tfBuffer);
    _tfBroadcaster = new tf2_ros::TransformBroadcaster;

    _targetsXYZ = readTargetPointsCSV("targets.csv");
    for (unsigned int i=0; i<NB_CART_AXIS; i++)
    {
      for (unsigned int j=0; j<NB_TARGETS; j++)
      {
        _maxLimsTarget(i) = fmax(_maxLimsTarget(i),abs(_targetsXYZ[i].second[j]));
      }
    }
   // cout<<_maxLimsTarget.transpose()<<endl;
}

targetObject::~targetObject() { me->_n.shutdown(); }

bool targetObject::init() //! Initialization of the node. Its datatype
                          //! (bool) reflect the success in
                          //! initialization

{

  _pubTargetReachedSphere = _n.advertise<visualization_msgs::Marker>("target_reached_sphere", 0);
  

  switch (_myTrackMode)
  {
  case ALL_TOOLS:
    
    for (unsigned int i=0; i<NB_TOOLS; i++)
    {
      _hapticTorques[i].setZero();
      _pubFootInput[i] = _n.advertise<custom_msgs::FootInputMsg_v5>("/"+std::string(Tools_Names[i])+"/target_fi_publisher/foot_input",0);
      _subSharedGrasp[i] = _n.subscribe<custom_msgs_gripper::SharedGrasping>(
                  "/"+std::string(Tools_Names[i])+"/sharedGrasping", 1,boost::bind(&targetObject::readSharedGrasp, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subLegGravityCompTorques[i] = _n.subscribe<custom_msgs::FootInputMsg_v5>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&targetObject::readLegGravityCompTorques, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subForceFootRestWorld[i] = _n.subscribe<geometry_msgs::WrenchStamped>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/force_foot_rest_world", 1,boost::bind(&targetObject::readForceFootRestWorld, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subUnbiasedJointTorques[i] = _n.subscribe<custom_msgs::FootOutputMsg_v3>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/torques_modified", 1,boost::bind(&targetObject::readUnbiasedJointTorques, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
      _subToolJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/tool_joint_state_publisher/joint_states"
      , 1, boost::bind(&targetObject::readToolState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay()); 
       _subFootPlatform[i] = _n.subscribe<custom_msgs::FootOutputMsg_v3>( "/FI_Output/"+std::string(Tools_Names2[i])
       , 1, boost::bind(&targetObject::readFIOutput, this, _1, i),
       ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());                  
      _subPlatformJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/platform_joint_publisher/joint_states"
      , 1, boost::bind(&targetObject::readPlatformState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());                  
                     
    }
    break;
  
  default:
    unsigned int i = 0;
    _hapticTorques[i].setZero();
    _pubFootInput[i] = _n.advertise<custom_msgs::FootInputMsg_v5>("/"+std::string(Tools_Names[i])+"/target_fi_publisher/foot_input",0);
    _pubSharedGrasp[i] = _n.advertise<custom_msgs_gripper::SharedGrasping>("/"+std::string(Tools_Names[i])+"/sharedGrasping",0);
    _subSharedGrasp[i] = _n.subscribe<custom_msgs_gripper::SharedGrasping>(
                  "/"+std::string(Tools_Names[i])+"/sharedGrasping", 1,boost::bind(&targetObject::readSharedGrasp, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subLegGravityCompTorques[i] = _n.subscribe<custom_msgs::FootInputMsg_v5>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&targetObject::readLegGravityCompTorques, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subForceFootRestWorld[i] = _n.subscribe<geometry_msgs::WrenchStamped>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/force_foot_rest_world", 1,boost::bind(&targetObject::readForceFootRestWorld, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subUnbiasedJointTorques[i] = _n.subscribe<custom_msgs::FootOutputMsg_v3>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/torques_modified", 1,boost::bind(&targetObject::readUnbiasedJointTorques, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
     _subToolJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/tool_joint_state_publisher/joint_states"
      , 1, boost::bind(&targetObject::readToolState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());    
      _subFootPlatform[i] = _n.subscribe<custom_msgs::FootOutputMsg_v3>( "/FI_Output/"+std::string(Tools_Names2[i])
       , 1, boost::bind(&targetObject::readFIOutput, this, _1, i),
       ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());     
     _subPlatformJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/platform_joint_publisher/joint_states"
      , 1, boost::bind(&targetObject::readPlatformState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }
 

  //boost::posix_time::ptime thistime = ros::Time::now().toBoost();
  boost::posix_time::ptime thistime = boost::posix_time::second_clock::local_time();
  
  std::string datefilename = to_simple_string(thistime);
  std::replace( datefilename.begin(), datefilename.end(), ':', '_');
  std::replace( datefilename.begin(), datefilename.end(), '.', '_');
  std::replace( datefilename.begin(), datefilename.end(), ',', '_');
  std::replace( datefilename.begin(), datefilename.end(), ' ', '_');
  std::replace( datefilename.begin(), datefilename.end(), '-', '_');

 // cout<<_statsfilename<<endl;
 if (_subjectID!="none")
  {
    _statsOutputFile.open(ros::package::getPath(std::string("surgical_sim")) + "/data/log/"+ _subjectID + "_" + datefilename + ".txt");
  }

  // Subscriber definitions
  signal(SIGINT, targetObject::stopNode);

  if (_n.ok()) {
    srand(ros::Time::now().toNSec());
    ros::spinOnce();
    ROS_INFO("The target spawner is about to start ");
    return true;
  }
  else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void targetObject::stopNode(int sig) { me->_stop = true;  }

void targetObject::run() {

  while (!_stop) {
    for (size_t i = 0; i < NB_TOOLS; i++)
    {
      _hapticTorques[i].setZero();
    }
       
    readTFTool(_myTrackMode);
    computeToolTipDerivatives(_myTrackMode);
    readTFTrocar(_myTrackMode); 

    evaluateTarget(_myTrackMode);
    computetargetObjectPose(_myTrackMode);
    writeTFtargetObject();
    recordStatistics();
    ros::spinOnce();
    _loopRate.sleep();
  }
  me->_statsOutputFile.close();
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
        _toolTipPositionPrev[n_] = _toolTipPosition[n_];
        tf::vectorMsgToEigen(toolTipTransform_.transform.translation,
                             _toolTipPosition[n_]);
        _toolTipQuaternionPrev[n_] = _toolTipQuaternion[n_];
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
      //ros::Duration(1.0).sleep();
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
      // cout<<_trocarRotationMatrix[n_]<<endl;
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


void targetObject::computetargetObjectPose(unsigned int n_){
  if (_targetsXYZ[0].second[_xTarget]!=0.0f)
  {
      _myPosition << _targetsXYZ[0].second[_xTarget], _targetsXYZ[2].second[_xTarget], -_targetsXYZ[1].second[_xTarget]+0.01;

  //  cout<<_myPosition.transpose()<<endl; 
    Eigen::Vector3d targetTrocarDistance = _trocarPosition[n_]-_myPosition;
    
    if (targetTrocarDistance.norm() > FLT_EPSILON) {
      _myRotationMatrix = Utils_math<double>::rodriguesRotation(
      Eigen::Vector3d(0.0, 0.0, 1.0), targetTrocarDistance);
    }
    _myRotationMatrix =  _myRotationMatrix * AngleAxis<double>(_myRandomAngle, Vector3d::UnitZ()).toRotationMatrix();
    _myQuaternion = Eigen::Quaternion<double>(_myRotationMatrix);
  }
  else
  {
    _xTarget++;
    computetargetObjectPose(_myTrackMode);
    if (_xTarget>=NB_TARGETS)
    {
      ROS_ERROR("Error generating target");
      _stop = true;
    }    
  }
}


void targetObject::evaluateTarget(unsigned int n_)
{  
  _precisionGrasp[n_] = 17.0*DEG_TO_RAD - _toolJointPosition[n_](tool_wrist_open_angle);
  _precisionPos[n_] = (_myPosition - _toolTipPosition[n_]).norm();
  _precisionAng[n_] = _myQuaternion.angularDistance(_toolTipQuaternion[n_]);
  double errorAng = 1.0-cos(_precisionAng[n_]);
  if (_precisionPos[n_] < 0.007 && errorAng < (1-cos(7.0*DEG_TO_RAD)))
  {
     _flagTargetReached[n_]=true;
     _myStatus=TARGET_REACHED;
     if (_flagTargetReachedOpen[n_] || (_toolJointPosition[n_](tool_wrist_open_angle)>20.0*DEG_TO_RAD))
     {
      _flagTargetReachedOpen[n_]=true; 
      publishTargetReachedSphere(visualization_msgs::Marker::ADD, CYAN,0.0);
        
      if (_toolJointPosition[n_](tool_wrist_open_angle)<=17.0*DEG_TO_RAD)
      {
          publishTargetReachedSphere(visualization_msgs::Marker::ADD, YELLOW,0.0);
          if (!_flagTargetGrasped[n_])
          {
            _startDelayForCorrection = ros::Time::now(); 
            std::cout<<"grasped"<<std::endl;
          }
          _flagTargetGrasped[n_]=true;
          _myStatus=TARGET_GRASPED;
      }
      else
      {
        _startDelayForCorrection = ros::Time::now(); 
        _flagTargetGrasped[n_]=false;
      }
     }
  }
  else
  {
    _startDelayForCorrection = ros::Time::now(); 
    _flagTargetReached[n_]=false; 
    _flagTargetReachedOpen[n_]=false;
    _flagTargetGrasped[n_]=false; 
    _myStatus=TARGET_NOT_REACHED;
     publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
  }

  if (_myStatus==TARGET_GRASPED && _nTarget < NB_TARGETS)
  {
    if ((ros::Time::now() - _startDelayForCorrection).toSec() >= DELAY_SEC)
    { 
    _nTarget++;
    int newTargetNum = _xTarget;
    while (_xTarget==newTargetNum)
    {
       newTargetNum = rand() % (NB_TARGETS-1);
      std::cout<<newTargetNum<<std::endl;
      if (_xTarget!=newTargetNum)
        {
          break;
        }
    }
    _xTarget = newTargetNum; // Generate new target randomly
    double randomAngleNew = _myRandomAngle;
    while(_myRandomAngle == randomAngleNew)
    {
      randomAngleNew = Utils_math<double>::bound(- 1.5 * M_PI + ((rand()%(NB_TARGETS-1)) * 3.0 * M_PI)/((NB_TARGETS-1)), -1.5 * M_PI, 1.5 * M_PI ) ;
       if (_myRandomAngle!=randomAngleNew)
        {
          break;
        }
    }
    _myRandomAngle = randomAngleNew;
    _flagTargetReached[n_] =false;
    _startDelayForCorrection=ros::Time::now();
    ROS_INFO("New target generated!. # %i",_nTarget);
    _myStatus = TARGET_CHANGED;
    publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
    }
  }

  if (_nTarget>=NB_TARGETS)
  {
    ROS_INFO("Protocol finished");
    publishTargetReachedSphere(visualization_msgs::Marker::ADD, RED,0.0);
    _stop=true;
  }

}


void targetObject::writeTFtargetObject(){
  _msgtargetObjectTransform.child_frame_id="target_object_link";
  _msgtargetObjectTransform.header.frame_id="torso_link";
  _msgtargetObjectTransform.header.stamp = ros::Time::now();

  tf::vectorEigenToMsg(_myPosition,_msgtargetObjectTransform.transform.translation);
  tf::quaternionEigenToMsg(_myQuaternion,_msgtargetObjectTransform.transform.rotation); 

  _tfBroadcaster->sendTransform(_msgtargetObjectTransform);
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


void targetObject::publishTargetReachedSphere(int32_t action_,Marker_Color color_, double delay_){
  
  _msgTargetReachedSphere.header.frame_id = "target_object_link";
  _msgTargetReachedSphere.header.stamp = ros::Time::now();
  _msgTargetReachedSphere.ns = "";
  _msgTargetReachedSphere.id = 0;
  _msgTargetReachedSphere.type = visualization_msgs::Marker::SPHERE;
  _msgTargetReachedSphere.action = action_;
  _msgTargetReachedSphere.pose.position.x = 0.0;
  _msgTargetReachedSphere.pose.position.y = 0.0;
  _msgTargetReachedSphere.pose.position.z = 0.0;
  _msgTargetReachedSphere.pose.orientation.x = 0.0;
  _msgTargetReachedSphere.pose.orientation.y = 0.0;
  _msgTargetReachedSphere.pose.orientation.z = 0.0;
  _msgTargetReachedSphere.pose.orientation.w = 1;
  _msgTargetReachedSphere.scale.x = 0.05;
  _msgTargetReachedSphere.scale.y = 0.05;
  _msgTargetReachedSphere.scale.z = 0.05;
  _msgTargetReachedSphere.color.a = 0.2; // Don't forget to set the alpha!
  switch (color_)
  {
  case YELLOW:
  {
    _msgTargetReachedSphere.color.r = 1;
    _msgTargetReachedSphere.color.g = 1;
    _msgTargetReachedSphere.color.b = 0;  
    break;
  }
  case RED:
  {
    _msgTargetReachedSphere.color.r = 1;
    _msgTargetReachedSphere.color.g = 0;
    _msgTargetReachedSphere.color.b = 0;  
    break;
  }
  case CYAN:
  {
    _msgTargetReachedSphere.color.r = 0;
    _msgTargetReachedSphere.color.g = 1;
    _msgTargetReachedSphere.color.b = 1;  
    break;
  }
  default:
  {
    break;
  }
  }
  _msgTargetReachedSphere.lifetime= ros::Duration(delay_);
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubTargetReachedSphere.publish(_msgTargetReachedSphere);
  // _mutex.unlock();
}

void targetObject::recordStatistics(){

  // ros::Duration deltaTime = ros::Time::now() - _startingTime;
  // double precisionPOS_on = _myStatus != TARGET_NOT_REACHED ? _precisionPos[RIGHT_TOOL] : 0.0;
  // double precisionANG_on = _myStatus != TARGET_NOT_REACHED ? _precisionAng[RIGHT_TOOL] * RAD_TO_DEG : 0.0;
  
  if (_subjectID != std::string("none"))
	{
	ros::Duration deltaTime = ros::Time::now() - _startingTime;
  double precisionPOS_on = _flagTargetReached[RIGHT_TOOL] ? _precisionPos[RIGHT_TOOL] : 0.0;
  double precisionANG_on = _flagTargetReached[RIGHT_TOOL] ? _precisionAng[RIGHT_TOOL] * RAD_TO_DEG : 0.0;
  double precisionGRASP_on = _flagTargetGrasped[RIGHT_TOOL] ? _precisionGrasp[RIGHT_TOOL] * RAD_TO_DEG : 0.0;
  
    if (_flagRecordingStarted)
    {
		_statsOutputFile << deltaTime<< " "
					<< _nTarget<< " "
          << _myStatus<< " "
          << _myPosition.transpose()<<" "
          << _myQuaternion.x()<<" "
          << _myQuaternion.y()<<" "
          << _myQuaternion.z()<<" "
          << _myQuaternion.w()<<" "
          << _toolJointPosition[RIGHT_TOOL](tool_wrist_open_angle)<<" "
          << _toolJointSpeed[RIGHT_TOOL](tool_wrist_open_angle)<<" "
          << _toolTipPosition[RIGHT_TOOL].transpose() << " "
          << _toolTipSpeed[RIGHT_TOOL].transpose() << " "
          << _toolTipAcc[RIGHT_TOOL].transpose() << " "
          << _toolTipJerk[RIGHT_TOOL].transpose() << " "
          << _toolTipQuaternion[RIGHT_TOOL].x()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].y()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].z()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].w()<<" "
          << _toolJointPosition[RIGHT_TOOL](tool_yaw)<<" "
          <<  precisionPOS_on<<" "
          <<  precisionANG_on<<" "
          <<  precisionGRASP_on<<" "
          << _footBaseWorldForce->force.x<<" "
          << _footBaseWorldForce->force.y<<" "
          << _footBaseWorldForce->force.z<<" "
          << _footBaseWorldForce->torque.x<<" "
          << _footBaseWorldForce->torque.y<<" "
          << _footBaseWorldForce->torque.z<<" "
          << _platformJointPosition[RIGHT_TOOL].transpose()<<" "
          << _platformJointVelocity[RIGHT_TOOL].transpose()<<" "
          << _platformJointEffortD[RIGHT_TOOL].transpose()<<" "
          << _platformJointEffortM[RIGHT_TOOL].transpose()<<" "
          << _platformJointEffortRef[RIGHT_TOOL].transpose()<<" "
          << _legGravityCompTorques[RIGHT_TOOL].transpose()<<" "
          << _hapticTorques[RIGHT_TOOL].transpose()<<" "
          << _hapticAxisFilterPos[RIGHT_TOOL]<<" "
          << _hapticAxisFilterGrasp[RIGHT_TOOL]<<" "
          << _myThreshold[RIGHT_TOOL]<<" "
          << std::endl;
    }
    else
    {
      _statsOutputFile << "t"<< " "
					<< "nT"<< " "
          << "stat"<< " "
          << "posTX"<<" "
          << "posTY"<<" "
          << "posTZ"<<" "
          << "quatTX"<<" "
          << "quatTY"<<" "
          << "quatTZ"<<" "
          << "quatTW"<<" "
          << "graspAngle"<<" "
          << "graspAngSpeed"<<" "
          << "posTipX" << " "
          << "posTipY" << " "
          << "posTipZ" << " "
          << "speedTipX" << " "
          << "speedTipY" << " "
          << "speedTipZ" << " "
          << "accTipX" << " "
          << "accTipY" << " "
          << "accTipZ" << " "
          << "jerkTipX" << " "
          << "jerkTipY" << " "
          << "jerkTipZ" << " "
          << "quatTipX"<<" "
          << "quatTipY"<<" "
          << "quatTipZ"<<" "
          << "quatTipW"<<" "
          << "rotTipZ"<<" "
          << "precL"<<" "
          << "precR"<<" "
          << "precG"<<" "
          << "wFX" <<" "
          << "wFY"<<" "
          << "wFZ"<<" "
          << "wTX"<<" "
          << "wTY"<<" "
          << "wTZ"<<" "
          << "posJY"<<" "
          << "posJX"<<" "
          << "posJPITCH"<<" "
          << "posJROLL"<<" "
          << "posJYAW"<<" "
          << "velJY"<<" "
          << "velJX"<<" "
          << "velJPITCH"<<" "
          << "velJROLL"<<" "
          << "velJYAW"<<" "
          << "effortDJY"<<" "
          << "effortDJX"<<" "
          << "effortDJPITCH"<<" "
          << "effortDJROLL"<<" "
          << "effortDJYAW"<<" "
          << "effortMJY"<<" "
          << "effortMJX"<<" "
          << "effortMJPITCH"<<" "
          << "effortMJROLL"<<" "
          << "effortMJYAW"<<" "
          << "effortRefJY"<<" "
          << "effortRefJX"<<" "
          << "effortRefJPITCH"<<" "
          << "effortRefJROLL"<<" "
          << "effortRefJYAW"<<" "
          << "effortLGJY"<<" "
          << "effortLGJX"<<" "
          << "effortLGJPITCH"<<" "
          << "effortLGJROLL"<<" "
          << "effortLGJYAW"<<" "
          << "hapticTX"<<" "
          << "hapticTY"<<" "
          << "hapticTPITCH"<<" "
          << "hapticTROLL"<<" "
          << "hapticTYAW"<<" "
          << "hapticAxisFilterPos"<<" "
          << "hapticAxisFilterGrasp"<<" "
          << "graspingThreshold"<<" "
          << std::endl;
          _flagRecordingStarted=true;
    }
	}
}

  void targetObject::readToolState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_) {

  for (unsigned int i = 0; i < NB_TOOL_AXIS_FULL; i++) {
    me->_toolJointPosition[n_](i) = msg->position[i];
    me->_toolJointSpeed[n_](i) = msg->velocity[i];
  }

  if (!_flagToolJointsConnected[n_]) {
    ROS_INFO("Joints received from tool %i", n_);
  }
  _flagToolJointsConnected[n_] = true;
}

  void targetObject::readPlatformState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev[n_](i) = me->_platformJointStates[n_](i);
    me->_platformJointPosition[n_](i) = msg->position[i];
    me->_platformJointVelocity[n_](i) = msg->velocity[i];
    me->_platformJointEffortD[n_](i) = msg->effort[i];
  }

  if (!_flagPlatformJointsConnected[n_]) {
    ROS_INFO("Joints received from platform %i", n_);
  }
  _flagPlatformJointsConnected[n_] = true;
  }

  void targetObject::readLegGravityCompTorques(const custom_msgs::FootInputMsg_v5::ConstPtr &msg, unsigned int n_) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev[n_](i) = me->_platformJointStates[n_](i);
    me->_legGravityCompTorques[n_](i) = msg->ros_effort[i];
  }

  if (!_flagLegGravityTorquesConnected[n_]) {
    ROS_INFO("Platform  %i Input Connected", n_);
  }
  _flagLegGravityTorquesConnected[n_] = true;
  }

  void targetObject::readFIOutput(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg,unsigned int n_) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev[n_](i) = me->_platformJointStates[n_](i);
    me->_platformJointPosition[n_](i) = msg->platform_position[Axis_Mod[i]] * conversion_factor[i];
    me->_platformJointVelocity[n_](i) = msg->platform_speed[Axis_Mod[i]] * conversion_factor[i];
    me->_platformJointEffortD[n_](i) = msg->platform_effortD[Axis_Mod[i]];
    me->_platformJointEffortRef[n_](i) = msg->platform_effortRef[Axis_Mod[i]];
    // me->_platformJointEffortM[n_](i) = msg->platform_effortM[Axis_Mod[i]];
  }

  if (!_flagPlatformJointsConnected[n_]) {
    ROS_INFO("Joints received from platform %i", n_);
  }
  _flagPlatformJointsConnected[n_] = true;
}



void targetObject::computeToolTipDerivatives(unsigned int n_){
   if (n_==ALL_TOOLS)
  {
    for (unsigned int i=0; i<ALL_TOOLS; i++)
    {
      computeToolTipDerivatives(i);
    }
  }
  else
  {
    double freq = 1.0/_dt;
    _toolTipSpeedPrev[n_] = _toolTipSpeed[n_];
    _toolTipSpeed[n_]= _toolTipSpeedFilter[n_]->update((_toolTipPosition[n_] - _toolTipPositionPrev[n_]) * freq);
    _toolTipAccPrev[n_] = _toolTipAcc[n_];
    _toolTipAcc[n_]= _toolTipAccFilter[n_]->update((_toolTipSpeed[n_] - _toolTipSpeedPrev[n_]) * freq);
    _toolTipJerk[n_]= _toolTipJerkFilter[n_]->update((_toolTipAcc[n_] - _toolTipAccPrev[n_]) * freq);
  }
}


void targetObject::readForceFootRestWorld(const geometry_msgs::WrenchStamped::ConstPtr &msg, unsigned int n_){
    _footBaseWorldForce[n_] = msg->wrench;
    if (!_flagFootBaseForceConnected[n_])
	 {
		ROS_INFO("Reading forces in the foot %i base w.r.t. world",n_);
    _flagFootBaseForceConnected[n_] = true;
	 }
}

void targetObject::readUnbiasedJointTorques(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg, unsigned int n_){
    for (size_t i = 0; i < NB_AXIS; i++)
    {
      _platformJointEffortM[n_](i) = msg->platform_effortM[i];
    }    
}

void targetObject::readSharedGrasp(const custom_msgs_gripper::SharedGrasping::ConstPtr &msg, unsigned int n_)
{
    _hapticAxisFilterPos[n_] = msg->sGrasp_hFilters[p_x];  
    _hapticAxisFilterGrasp[n_] = msg->sGrasp_hFilters[p_roll];
    _myThreshold[n_] = msg->sGrasp_threshold;   
    _aState[n_] = (targetObject::Action_State) msg->sGrasp_aState;
    for (size_t i = 0; i < NB_AXIS; i++)
    {
      _hapticTorques[n_](i) = msg->sGrasp_hapticTorques[i];
    }
}
  
  
  
  
  
  
  
  
  
  
    
