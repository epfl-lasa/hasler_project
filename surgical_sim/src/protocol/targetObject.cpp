#include "Utils_math.h"
#include "targetObject.h"
#include "tf_conversions/tf_eigen.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//#include "boost/date_time/gregorian/gregorian.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
// #include "boost/date_time/gregorian_types.hpp"



using namespace boost::gregorian;
#define ListofToolAxes(enumeration, names) names,
char const *Tool_Axis_Names[]{TOOL_AXES};
#undef ListofToolAxes

#define ListofTools(enumeration, names) names,
char const *Tools_Names[]{TOOL_NAMES};
#undef ListofTools

#define DELAY_SEC 3.0

const float SAMPLING_TIME = 3000; // 0.5 s

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};

targetObject *targetObject::me = NULL;

targetObject::targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_)
    : _n(n_1), _loopRate(frequency), _dt(1.0f / frequency),
    _myModel(model_), _myName(name_){

   me = this;
  _stop = false;

  double magnitude_vib = 240.0;
  double decayRate_vib = 60.0;
  double frequency_vib = 32.6;

  _devErrorPenetration=0.0;
  _vibrationGrasping = 0.0;
  _impedanceGrasping = 0.0;
  for (size_t i = 0; i < NB_TOOLS; i++)
  {
    _hapticTorques[i].setZero();
  }
  
  _errorPenetration=0.0;
  _errorPenetration_prev=0.0;
  _precisionAng=0.0;
  _precisionPos=0.0;  
  NB_TARGETS=0;
  
  _xTarget=0;  _nTarget=0;

  
  for (unsigned int tool=0; tool<NB_TOOLS; tool++)
  {
    _toolTipPosition[tool].setZero();
    _toolTipPositionPrev[tool].setZero();
    _trocarPosition[tool].setZero();

    _toolJointStates[tool].setZero();  
    _toolJointStates_prev[tool].setZero(); 
    _platformJointStates[tool].setZero();
    _platformJointStates_prev[tool].setZero(); 
    _toolTipQuaternion[tool].setIdentity();
    _toolTipQuaternionPrev[tool].setIdentity();
    _trocarQuaternion[tool].setIdentity();
   
    _toolTipRotationMatrix[tool].setIdentity();
    _trocarRotationMatrix[tool].setIdentity();
  
    _flagTargetReached[tool] = false;
    _flagTargetReachedAndGrasped[tool] = false;
    _flagTrocarTFConnected[tool] = false;
    _flagToolTipTFConnected[tool] = false;
    _flagFootBaseForceConnected[tool]=false;
    _flagToolJointsConnected[tool]=false;

    _kpPosition[tool].setZero();
    _kiPosition[tool].setZero();
    _kdPosition[tool].setZero();
    
    _posCtrlRef[tool].setZero();
    _posCtrlIn[tool].setZero();
    _posCtrlInPrev[tool].setZero();
    _posCtrlOut[tool].setZero();

    _kpGrasping[tool]=0.0;
    _kiGrasping[tool]=0.0;
    _kdGrasping[tool]=0.0;

    _graspCtrlRef[tool]=0.0;
    _graspCtrlIn[tool]=0.0;
    _graspCtrlOut[tool]=0.0;

    _aStateNext[tool]=A_POSITIONING;
    _aState[tool]=A_GRASPING;


   for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
   {
      _pidPosition[tool][i] = new PIDd(&_posCtrlIn[tool](i), &_posCtrlOut[tool](i), &_posCtrlRef[tool](i),
      _kpPosition[tool](i), _kiPosition[tool](i), _kdPosition[tool](i), P_ON_E , DIRECT, 0.5);
      _pidPosition[tool][i]->setMode(AUTOMATIC);
      _pidPosition[tool][i]->setSampleTime(SAMPLING_TIME);
   }
    
    _pidGrasping[tool] = new PIDd(&_graspCtrlIn[tool], &_graspCtrlOut[tool], &_graspCtrlRef[tool],
    _kpGrasping[tool], _kiGrasping[tool], _kdGrasping[tool], P_ON_E , DIRECT, 0.5);
    _pidGrasping[tool]->setMode(AUTOMATIC);
    _pidGrasping[tool]->setSampleTime(SAMPLING_TIME);

    _pidPosition[tool][Axis_Mod[p_x]]->setOutputLimits(-15.0,15.0);
    _pidPosition[tool][Axis_Mod[p_y]]->setOutputLimits(-15.0,15.0);
    _pidPosition[tool][Axis_Mod[p_pitch]]->setOutputLimits(-2.0,2.0);
    _pidPosition[tool][Axis_Mod[p_yaw]]->setOutputLimits(-2.0,2.0);
    
    _pidGrasping[tool]->setOutputLimits(-0.5,0.5);
  }



    _flagRecordingStarted=false;
    _flagHapticGrasping=false;
    _flagSharedGrasping=false;

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

   if (!_n.getParam("hapticGraspOn", _flagHapticGrasping))
  { 
      ROS_ERROR("No indicaton of the subject ID (e.g. sXX) was done"); 
      _flagHapticGrasping = false;
  }
  
     if (!_n.getParam("sharedGraspOn", _flagSharedGrasping))
  { 
      ROS_ERROR("No indicaton of the subject ID (e.g. sXX) was done"); 
      _flagSharedGrasping = false;
  }
  

    
  if (!_n.getParam("magnitudeVibration", magnitude_vib))
  { 
      ROS_ERROR("No indication of the magnitude of the vibration was given"); 
  }

  if (!_n.getParam("decayRateVibration", decayRate_vib))
  { 
      ROS_ERROR("No indication of the decayRate of the vibration was given"); 
  }

  if (!_n.getParam("frequencyVibration", frequency_vib))
  { 
      ROS_ERROR("No indication of the frequency of the vibration was given"); 
  }

  _myVibrator = new vibrator(&_devErrorPenetration, &_vibrationGrasping,-1.0 * magnitude_vib,decayRate_vib,frequency_vib,0.0);


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


  _myStatus = TARGET_NOT_REACHED;

  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("Failed to construct kdl tree");
    _stop = true;
  }
    KDL::Vector grav_vector(0.0, 0.0, (double)GRAVITY);
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
    cout<<_maxLimsTarget.transpose()<<endl;
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
      _subForceFootRestWorld[i] = _n.subscribe<geometry_msgs::WrenchStamped>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/force_foot_rest_world", 1,boost::bind(&targetObject::readForceFootRestWorld, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subToolJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/tool_joint_state_publisher/joint_states"
      , 10, boost::bind(&targetObject::readToolState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay()); 
      _subPlatformJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/platform_joint_publisher/joint_states"
      , 10, boost::bind(&targetObject::readPlatformState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());                  
    }
    break;
  
  default:
    unsigned int i = 0;
    _hapticTorques[i].setZero();
    _pubFootInput[i] = _n.advertise<custom_msgs::FootInputMsg_v5>("/"+std::string(Tools_Names[i])+"/target_fi_publisher/foot_input",0);
    _subForceFootRestWorld[i] = _n.subscribe<geometry_msgs::WrenchStamped>(
                  "/"+std::string(Tools_Names[i])+"/force_sensor_modifier/force_foot_rest_world", 1,boost::bind(&targetObject::readForceFootRestWorld, this, _1, i),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
     _subToolJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/tool_joint_state_publisher/joint_states"
      , 1, boost::bind(&targetObject::readToolState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());    
     _subPlatformJointStates[i] = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tools_Names[i])+"/platform_joint_publisher/joint_states"
      , 1, boost::bind(&targetObject::readPlatformState, this, _1, i),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());             
    break;
  }
 

  boost::posix_time::ptime thistime = ros::Time::now().toBoost();
  
  std::string datefilename = to_iso_extended_string(thistime);
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
    readTFTrocar(_myTrackMode);
    if (true) {
      estimateActionState(_myTrackMode);
      if (_flagSharedGrasping)
      {
        doSharedControl(_myTrackMode);
      }
      evaluateTarget(_myTrackMode);
      computeTargetObjectPose(_myTrackMode);
      writeTFTargetObject();
      recordStatistics();
      _myVibrator->run(ros::Time::now());
      publishFootInput(_myTrackMode);
    }
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


void targetObject::computeTargetObjectPose(unsigned int n_){
  if (_targetsXYZ[0].second[_xTarget]!=0.0f)
  {
      _myPosition << _targetsXYZ[0].second[_xTarget], _targetsXYZ[2].second[_xTarget], -_targetsXYZ[1].second[_xTarget]+0.01;

  //  cout<<_myPosition.transpose()<<endl; 
    Eigen::Vector3d targetTrocarDistance = _trocarPosition[n_]-_myPosition;
    
    if (targetTrocarDistance.norm() > FLT_EPSILON) {
      _myRotationMatrix = Utils_math<double>::rodriguesRotation(
      Eigen::Vector3d(0.0, 0.0, 1.0), targetTrocarDistance);
    }
    _myQuaternion = Eigen::Quaternion<double>(_myRotationMatrix);
  }
  else
  {
    _xTarget++;
    computeTargetObjectPose(_myTrackMode);
    if (_xTarget==NB_TARGETS)
    {
      ROS_ERROR("Error generating target");
      _stop = true;
    }    
  }
}


void targetObject::evaluateTarget(unsigned int n_)
{
  static int count =0; // temporal varible 
  _precisionPos = (_myPosition - _toolTipPosition[n_]).norm();
  _precisionAng = _myQuaternion.angularDistance(_toolTipQuaternion[n_]);
  double errorAng = 1.0-cos(_precisionAng);
  //std::cout << rotError << endl;
  
  bool grasped_target = _toolJointStates[n_](tool_wrist_open_angle)<17.0*DEG_TO_RAD;
  _errorPenetration_prev = _errorPenetration;
  _errorPenetration = 17.0 * DEG_TO_RAD - _toolJointStates[n_](tool_wrist_open_angle);
  _devErrorPenetration = (_errorPenetration - _errorPenetration_prev) * _dt;
  
  

  // if (count != 0)
  // {
  //   if (_myStatus == TARGET_GRASPED){
  //     _myVibrator->start();
  //     //cout<<"target_grasped"<<endl;
  //         //cout<<_devErrorPenetration<<endl;
  //       _impedanceGrasping =   Utils_math<double>::bound(3.0 * _errorPenetration - 1.0 * _devErrorPenetration,-2.0f,2.0f); 
          
  //     }
  //   else
  //   {
  //     _impedanceGrasping = 0.0;
  //     if (_toolJointStates[n_](tool_wrist_open_angle)>25*DEG_TO_RAD)
  //     {
  //       // if (_devErrorPenetration<=0)
  //       // {
  //         _myVibrator->reset();
  //       // }
  //     }

  //   }

  //   if (_flagHapticGrasping)  
  //   {
  //     _hapticTorques[n_](p_roll) = Utils_math<double>::bound(-(_vibrationGrasping + _impedanceGrasping) + _graspCtrlOut[n_],-2.0, 2.0);
  //   }
  //   else
  //   {
  //     _hapticTorques[n_](p_roll) = Utils_math<double>::bound(_graspCtrlOut[n_],-2.0, 2.0);
  //   }
  // }


  // else{
  //       cout<<"first grasp"<<endl;
  //     }
  // count++;



  if (_precisionPos < 0.01 && errorAng < (1-cos(5.0*DEG_TO_RAD))) {
    if (!_flagTargetReached[n_])
    {
      _flagTargetReached[n_]=true;
      publishTargetReachedSphere(visualization_msgs::Marker::ADD, CYAN,0.0);
      _myStatus = TARGET_REACHED;
    }
    if (_flagTargetReached && grasped_target)
    {
      if (!_flagTargetReachedAndGrasped[n_])
      {
        _startDelayForCorrection = ros::Time::now(); 
        _flagTargetReachedAndGrasped[n_]=true;
        publishTargetReachedSphere(visualization_msgs::Marker::ADD, YELLOW,3.0);
        _myStatus = TARGET_GRASPED;
      }
    }
    else
    {
      if (_flagTargetReachedAndGrasped[n_])
      {
        publishTargetReachedSphere(visualization_msgs::Marker::DELETE,NONE,0.0);
        _flagTargetReachedAndGrasped[n_]=false;
      }
       _myStatus = TARGET_REACHED;
    }
    
  }
  else
  {
    if (_flagTargetReached)
    {
      publishTargetReachedSphere(visualization_msgs::Marker::DELETE,NONE,0.0);
      _flagTargetReached[n_]=false;
    }
    _myStatus = TARGET_NOT_REACHED;
  }
  
  
  if (_myStatus==TARGET_GRASPED && _nTarget < NB_TARGETS)
  {
    if ((ros::Time::now() - _startDelayForCorrection).toSec() > DELAY_SEC)
    { 
    _nTarget++;
    _xTarget = int(rand() % NB_TARGETS-1); // Generate new target randomly
    _flagTargetReached[n_] =false;
    _startDelayForCorrection=ros::Time::now();
    ROS_INFO("New target generated!. # %i",_nTarget);
    _myStatus = TARGET_CHANGED;
    }
  }

  if (_nTarget>=NB_TARGETS)
  {
    ROS_INFO("Protocol finished");
    publishTargetReachedSphere(visualization_msgs::Marker::ADD, RED,0.0);
    _stop=true;
  }

}
void targetObject::estimateActionState(unsigned int n_)
  
  
{

  if ( (30.0*DEG_TO_RAD - _toolJointStates[n_](tool_wrist_open_angle)) < 5.0*DEG_TO_RAD)
  {
    _aStateNext[n_] = A_POSITIONING;
    //cout<<"positioning"<<endl;
  }
  else
  {
    _aStateNext[n_] = A_GRASPING;
    //cout<<"grasping"<<endl;
  }
  
  if (_aState[n_]!=_aStateNext[n_])
  {
    _posCtrlRef[n_] = _posCtrlIn[n_];
    _aState[n_] = _aStateNext[n_];
  }

};

void targetObject::doSharedControl(unsigned int n_)
{

  switch (_aState[n_])
  {
  case A_POSITIONING:
    {
      _pidPosition[n_][Axis_Mod[p_x]]->reset();
      _pidPosition[n_][Axis_Mod[p_y]]->reset();
      _pidPosition[n_][Axis_Mod[p_pitch]]->reset();
      _pidPosition[n_][Axis_Mod[p_yaw]]->reset();

      _kpPosition[n_](Axis_Mod[p_x]) = 0.0f * SCALE_GAINS_LINEAR_POSITION;
      _kpPosition[n_](Axis_Mod[p_y]) = 0.0f * SCALE_GAINS_LINEAR_POSITION;
      
      _kpPosition[n_](Axis_Mod[p_pitch]) = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      _kpPosition[n_](Axis_Mod[p_yaw]) = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      
      _kdPosition[n_](Axis_Mod[p_x]) = 0.0f * SCALE_GAINS_LINEAR_POSITION;
      _kdPosition[n_](Axis_Mod[p_y]) = 0.0f * SCALE_GAINS_LINEAR_POSITION;

      _kdPosition[n_](Axis_Mod[p_pitch]) = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      _kdPosition[n_](Axis_Mod[p_yaw]) = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      
      _kpGrasping[n_] = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      _kdGrasping[n_] = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      break;
    }
  
  case A_GRASPING:
    {
      _pidGrasping[n_]->reset();
      _kpPosition[n_](Axis_Mod[p_x]) = 1000.0 * SCALE_GAINS_LINEAR_POSITION;
      _kpPosition[n_](Axis_Mod[p_y]) = 1000.0 * SCALE_GAINS_LINEAR_POSITION;
      
      _kpPosition[n_](Axis_Mod[p_pitch]) = 1000.0 * SCALE_GAINS_ANGULAR_POSITION;
      _kpPosition[n_](Axis_Mod[p_yaw]) = 1000.0 * SCALE_GAINS_ANGULAR_POSITION;
      
      _kdPosition[n_](Axis_Mod[p_x]) = 3000.0f * SCALE_GAINS_LINEAR_POSITION;
      _kdPosition[n_](Axis_Mod[p_y]) = 2000.0f * SCALE_GAINS_LINEAR_POSITION;

      _kdPosition[n_](Axis_Mod[p_pitch]) = 2000.0f * SCALE_GAINS_ANGULAR_POSITION;
      _kdPosition[n_](Axis_Mod[p_yaw]) = 6000.0f * SCALE_GAINS_ANGULAR_POSITION;
      
      _kpGrasping[n_] = 0.0f * SCALE_GAINS_ANGULAR_POSITION; 
      break;
    }
  }
  _posCtrlInPrev[n_] =  _posCtrlIn[n_];
  for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
    {
      
      _posCtrlIn[n_](i)=_platformJointStates[n_](Axis_Mod[i]);
            
      _pidPosition[n_][i]->setTunings(_kpPosition[n_](i),_kiPosition[n_](i),_kdPosition[n_](i));
      _pidPosition[n_][i]->compute(ros::Time::now());
        // _posCtrlOut[n_](p_x) = Utils_math<double>::bound(_kpPosition[n_](p_x) * (_posCtrlRef[n_](p_x) - _posCtrlIn[n_](p_x)) - _kdPosition[n_](p_x) * (_posCtrlIn[n_](p_x) - _posCtrlInPrev[n_](p_x)) *_dt ,-5.0,5.0);    
      _hapticTorques[n_](Axis_Pos[i])=_posCtrlOut[n_](i);    

    }

    //cout<<_posCtrlRef[n_](p_x)<<endl;
    

    
    _graspCtrlIn[n_] = _toolJointStates[n_](p_roll);
    _pidGrasping[n_]->setTunings(_kpGrasping[n_],_kiGrasping[n_],_kdGrasping[n_]);
    //_pidGrasping[n_]->compute(ros::Time::now()); 
    //_hapticTorques[n_](p_roll)=_graspCtrlOut[n_]; -> Added in the detect grasp condition in evaluate target routine
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

  ros::Duration deltaTime = ros::Time::now() - _startingTime;
  double precisionPOS_on = _myStatus != TARGET_NOT_REACHED ? _precisionPos : 0.0;
  double precisionANG_on = _myStatus != TARGET_NOT_REACHED ? _precisionAng * RAD_TO_DEG : 0.0;
  
  if (_subjectID != std::string("none"))
	{
	ros::Duration deltaTime = ros::Time::now() - _startingTime;
  double precisionPOS_on = _myStatus != TARGET_NOT_REACHED ? _precisionPos : 0.0;
  double precisionANG_on = _myStatus != TARGET_NOT_REACHED ? _precisionAng * RAD_TO_DEG : 0.0;
  
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
          << _toolTipPosition[RIGHT_TOOL].transpose() << " "
          << _toolTipQuaternion[RIGHT_TOOL].x()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].y()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].z()<<" "
          << _toolTipQuaternion[RIGHT_TOOL].w()<<" "
          << precisionPOS_on<<" "
          << precisionANG_on<<" "
          << _footBaseWorldForce->force.x<<" "
          << _footBaseWorldForce->force.y<<" "
          << _footBaseWorldForce->force.z<<" "
          << _footBaseWorldForce->torque.x<<" "
          << _footBaseWorldForce->torque.y<<" "
          << _footBaseWorldForce->torque.z<<" "
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
          << "posTipX" << " "
          << "posTipY" << " "
          << "posTipZ" << " "
          << "quatTipX"<<" "
          << "quatTipY"<<" "
          << "quatTipZ"<<" "
          << "quatTipW"<<" "
          << "precL"<<" "
          << "precR"<<" "
          << "wFX" <<" "
          << "wFY"<<" "
          << "wFZ"<<" "
          << "wTX"<<" "
          << "wTY"<<" "
          << "wTZ"<<" "
          << std::endl;
          _flagRecordingStarted=true;
    }
	}
}

  void targetObject::readToolState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_) {

  for (unsigned int i = 0; i < NB_TOOL_AXIS_FULL; i++) {
    me->_toolJointStates_prev[n_](i) = me->_toolJointStates[n_](i);
    me->_toolJointStates[n_](i) = msg->position[i];
  }

  if (!_flagToolJointsConnected[n_]) {
    ROS_INFO("Joints received from tool %i", n_);
  }
  _flagToolJointsConnected[n_] = true;
}

  void targetObject::readPlatformState(const sensor_msgs::JointState::ConstPtr &msg, unsigned int n_) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    me->_platformJointStates_prev[n_](i) = me->_platformJointStates[n_](i);
    me->_platformJointStates[n_](i) = msg->position[i];
  }

  if (!_flagPlatformJointsConnected[n_]) {
    ROS_INFO("Joints received from platform %i", n_);
  }
  _flagPlatformJointsConnected[n_] = true;
}


  void targetObject::readForceFootRestWorld(const geometry_msgs::WrenchStamped::ConstPtr &msg, unsigned int n_){
    _footBaseWorldForce[n_] = msg->wrench;
    if (!_flagFootBaseForceConnected[n_])
	 {
		ROS_INFO("Reading forces in the foot %i base w.r.t. world",n_);
    _flagFootBaseForceConnected[n_] = true;
	 }
}

void targetObject::publishFootInput(int n_)
{
  if (n_==ALL_TOOLS)
  {
    for (unsigned int i=0; i<ALL_TOOLS; i++)
    {
      publishFootInput(i);
    }
  }
  else
  {
    _msgFootInput[n_].ros_effort.fill(0.0f);
    _msgFootInput[n_].ros_position.fill(0.0f);
    _msgFootInput[n_].ros_forceSensor.fill(0.0f);
    _msgFootInput[n_].ros_speed.fill(0.0f);
    _msgFootInput[n_].ros_filterAxisForce.fill(1.0f); 

    for (unsigned int i=0; i<NB_PLATFORM_AXIS; i++)
    {
      _msgFootInput[n_].ros_effort[i] =  _hapticTorques[n_](i);
    }
    if (_flagSharedGrasping)
    {
      if (_aState[n_] == A_GRASPING)
      {
        _msgFootInput[n_].ros_filterAxisForce.fill(0.0f); 
        _msgFootInput[n_].ros_filterAxisForce[p_roll] = 1.0f;
      }
      if (_aState[n_] == A_POSITIONING)
      {
        _msgFootInput[n_].ros_filterAxisForce.fill(1.0f); 
        //_msgFootInput[n_].ros_filterAxisForce[p_roll] = 0.0f;
      }
    }
    _pubFootInput[n_].publish(_msgFootInput[n_]);
  }
  
}
  