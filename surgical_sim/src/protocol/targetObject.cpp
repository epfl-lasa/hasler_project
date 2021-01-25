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
char const *Tool_Names[]{TOOL_NAMES};
#undef ListofTools

char const *Tool_Names2[]{"None","Right","Left"};

#define DELAY_SEC 1 // Its so low because of the gazebo timeout to spawn (~3s)


const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const float SAMPLING_TIME = 10; // 100Hz

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_roll,p_yaw};

const double scaleFoot[] = {Axis_Limits[0]/0.15,Axis_Limits[1]/0.15,(1.5*Axis_Limits[2])/(2*0.15),35.0*DEG_TO_RAD/(1.5*M_PI),20.5*DEG_TO_RAD/30*DEG_TO_RAD}; // X, Y, Z, YAW, ROLL;

const double LimitsCart[NB_CART_AXIS][NB_TOOLS][NB_LIMS] = {{{-0.15,0.05},{0.05,0.15}},  //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]
                                                                        {{-0.15,0.15},{-0.15,0.15}}, //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]
                                                                        {{-0.26,-0.06},{-0.26,-0.06}} }; //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]



targetObject *targetObject::me = NULL;

  double magnitude_vib = 2.0;
  double decayRate_vib = 3.0;
  double frequency_vib = 32.6;

targetObject::targetObject(ros::NodeHandle &n_1, double frequency, urdf::Model model_, std::string name_)
    : _n(n_1), _loopRate(frequency), _dt(1.0f / frequency),
    _myModel(model_), _myName(name_){

   me = this;
  _stop = false;

  _msgAllTransforms.resize(NB_TF_LIST);
  Eigen::Vector3d alphaPosition;
  alphaPosition.setConstant(0.6);

  Eigen::Vector3d alphaSpeed;
  alphaSpeed.setConstant(0.95);

  Eigen::Vector3d alphaAcc;
  alphaAcc.setConstant(0.95);

  Eigen::Vector3d alphaJerk;
  alphaJerk.setConstant(0.95);

  _toolTipPositionFilter = new MatLP_Filterd(alphaPosition);
  _toolTipSpeedFilter = new MatLP_Filterd(alphaSpeed);
  _toolTipAccFilter = new MatLP_Filterd (alphaAcc);
  _toolTipJerkFilter= new MatLP_Filterd (alphaJerk);
  _myThreshold=0.0;
  _vibrationGrasping = 0.0;
  _impedanceGrasping = 0.0;
  _hapticTorques.setZero();
  _precisionAng=0.0;
  _precisionPos=0.0;  
  _precisionGrasp=0.0; 
  _hapticAxisFilterPos=1.0; 
 

  _myRandomAngle=0.0;

  NB_TARGETS=0;
  
  _xTarget=0;  _nTarget=0; _xTargetPrev=_xTarget;

  
    
    _toolTipPosition.setZero();
    _targetAimPosition.setZero();
    _toolTipPositionWRTTorso.setZero();
    _toolTipPositionPrev.setZero();
    _trocarPosition.setZero();

    _toolJointPosition.setZero();  
    _toolJointSpeed.setZero();  
    _platformJointPosition.setZero();
    _platformJointVelocity.setZero();
    _platformJointEffortD.setZero();
    _platformJointEffortRef.setZero();
    _legGravityCompTorques.setZero();
    _platformJointEffortM.setZero();
    // _platformJointStates.setZero();
    // _platformJointStates_prev.setZero(); 
    _toolTipQuaternion.setIdentity();
    _targetAimQuaternion.setIdentity();
    _toolTipQuaternionWRTTorso.setIdentity();
    _toolTipQuaternionPrev.setIdentity();
    _trocarQuaternion.setIdentity();
   
    _toolTipRotationMatrix.setIdentity();
    _trocarRotationMatrix.setIdentity();
    
    _flagGazeboLinkStateRead = false;
    _flagTargetReached = false;
    // _flagTargetReachedOpen = false;
    _flagTargetGrasped = false;
    _flagTargetSpawned = false;
    _flagTrocarTFConnected = false;
    _flagToolTipTFConnected = false;
    _flagFootBaseForceConnected=false;
    _flagToolJointsConnected=false;
    _flagLegGravityTorquesConnected=false;
    _aState=A_POSITIONING;
  


    _flagRecordingStarted=false;

  _myPositionSpawn.setZero();
  _myQuaternionSpawn.setIdentity();
  _myRotationMatrixSpawn.setIdentity();
  _myPositionCurrent.setZero();
  _myQuaternionCurrent.setIdentity();
  _myRotationMatrixCurrent.setIdentity();
  _maxLimsTarget.setZero();
  
  std::string trackingMode;
  _startDelayForCorrection = ros::Time::now();
  _startingTime = ros::Time::now();
  


  for (size_t i = 0; i < NB_CART_AXIS; i++)
  {
    for (size_t j = 0; j < NB_LIMS; j++)
    {
      _cartesianLimsTools(i,j) = LimitsCart[i][_myTrackID-1][j];
    }
  }

   std::vector<double> cartLimsX;
   std::vector<double> cartLimsY;
   std::vector<double> cartLimsZ;
  
    cartLimsX.resize(NB_LIMS);     cartLimsY.resize(NB_LIMS);    cartLimsZ.resize(NB_LIMS);


    if (!_n.getParam("/"+std::string(Tool_Names[_myTrackID])+"_tool/"+"cartLimsX", cartLimsX))
    { 
        ROS_ERROR("[%s target]: No cartLimsX param", Tool_Names[_myTrackID]); 
    }

     if (!_n.getParam("/"+std::string(Tool_Names[_myTrackID])+"_tool/"+"cartLimsY", cartLimsY))
    { 
        ROS_ERROR("[%s target]: No cartLimsY param", Tool_Names[_myTrackID]); 
    }

     if (!_n.getParam("/"+std::string(Tool_Names[_myTrackID])+"_tool/"+"cartLimsZ", cartLimsZ))
    { 
        ROS_ERROR("[%s target]: No cartLimsZ param", Tool_Names[_myTrackID]); 
    }
      
      
      for (size_t i = 0; i < NB_LIMS; i++)
      {
        _cartesianLimsTools(CART_X,i) = cartLimsX[i];
        _cartesianLimsTools(CART_Y,i) = cartLimsY[i];
        _cartesianLimsTools(CART_Z,i) = cartLimsZ[i];
      }
  

  if (!_n.getParam("trackingMode", trackingMode))
  { 
        ROS_ERROR("[%s target]: No indicaton of tracking mode (right, left, both) was done", Tool_Names[_myTrackID]); 
  }

   if (!_n.getParam("targetName", _myName))
  { 
      ROS_ERROR("[%s target]: No indicaton of target name (e.g. 1, etc) was done", Tool_Names[_myTrackID]); 
      _myName="t1";
  }

   
  if (!_n.getParam("subjectID", _subjectID))
  { 
      ROS_ERROR("[%s target]: No indicaton of the subject ID (e.g. sXX) was done", Tool_Names[_myTrackID]); 
      _subjectID = "s0";
  }

  
  

  if (trackingMode.compare("right") == 0) {
      _myTrackID = RIGHT_TOOL;
    } else if (trackingMode.compare("left") == 0) {
       _myTrackID = LEFT_TOOL;
    } else {
      ROS_ERROR("[%s target]: You didn't enter a tracking id: left or right", Tool_Names[_myTrackID]);
      _stop=true;
    }


  for (size_t i = 0; i < NB_TARGET_STATUS; i++)
  {
    _myStatus[i]=0;
  }
  


  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("[%s target]: Failed to construct kdl tree", Tool_Names[_myTrackID]);
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

  if (!_n.getParam("/"+std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target/robot_description",_myModelXml)) {
    ROS_ERROR("[%s target]: Failed to get model xml file", Tool_Names[_myTrackID]);
  }
  else
  {
    ROS_INFO("[%s target]: Successfully got xml file", Tool_Names[_myTrackID]);
  }

_myVirtualJointLims(CART_X,L_MIN) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_x_joint")->limits->lower;
_myVirtualJointLims(CART_X,L_MAX) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_x_joint")->limits->upper;
_myVirtualJointLims(CART_Y,L_MIN) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_y_joint")->limits->lower;
_myVirtualJointLims(CART_Y,L_MAX) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_y_joint")->limits->upper;
_myVirtualJointLims(CART_Z,L_MIN) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_z_joint")->limits->lower;
_myVirtualJointLims(CART_Z,L_MAX) = _myModel.getJoint(std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_z_joint")->limits->upper;
}

targetObject::~targetObject() { me->_n.shutdown(); }

bool targetObject::init() //! Initialization of the node. Its datatype
                          //! (bool) reflect the success in
                          //! initialization

{

  _hapticTorques.setZero();
  
  _pubTargetReachedSphere = _n.advertise<visualization_msgs::Marker>("target_reached_sphere", 0);
  _pubRvizTargetMarker = _n.advertise<visualization_msgs::Marker>("marker_target_rviz", 0);

  _pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v5>("/"+std::string(Tool_Names[_myTrackID])+"/target_fi_publisher/foot_input",0);
  _pubSharedGrasp = _n.advertise<custom_msgs_gripper::SharedGraspingMsg>("/"+std::string(Tool_Names[_myTrackID])+"/sharedGrasping",0);

  _subGazeboLinkStates = _n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1,boost::bind(&targetObject::readGazeboLinkStates, this, _1),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subSharedGrasp = _n.subscribe<custom_msgs_gripper::SharedGraspingMsg>("/"+std::string(Tool_Names[_myTrackID])+"/sharedGrasping", 1,boost::bind(&targetObject::readSharedGrasp, this, _1),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  
  _subLegGravityCompTorques = _n.subscribe<custom_msgs::FootInputMsg_v5>(
                  "/"+std::string(Tool_Names[_myTrackID])+"/force_sensor_modifier/leg_comp_platform_effort", 1,boost::bind(&targetObject::readLegGravityCompTorques, this, _1),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  
  
  _subForceFootRestWorld = _n.subscribe<geometry_msgs::WrenchStamped>(
                  "/"+std::string(Tool_Names[_myTrackID])+"/force_sensor_modifier/force_foot_rest_world", 1,boost::bind(&targetObject::readForceFootRestWorld, this, _1),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  _subUnbiasedJointTorques = _n.subscribe<custom_msgs::FootOutputMsg_v3>(
                  "/"+std::string(Tool_Names[_myTrackID])+"/force_sensor_modifier/torques_modified", 1,boost::bind(&targetObject::readUnbiasedJointTorques, this, _1),
                  ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
  _subToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myTrackID])+"_tool/joint_states"
      , 1, boost::bind(&targetObject::readToolState, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());    
  _subFootPlatform = _n.subscribe<custom_msgs::FootOutputMsg_v3>( "/FI_Output/"+std::string(Tool_Names2[_myTrackID])
       , 1, boost::bind(&targetObject::readFIOutput, this, _1),
       ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());     
  _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myTrackID])+"/platform_joint_publisher/joint_states"
      , 1, boost::bind(&targetObject::readPlatformState, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  

  _clientSpawnNewTargetAtPos = _n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  _clientDeleteOldTarget = _n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  //boost::posix_time::ptime thistime = ros::Time::now().toBoost();
  boost::posix_time::ptime thistime = boost::posix_time::second_clock::local_time();
  
  std::string datefilename = to_simple_string(thistime);
  std::replace( datefilename.begin(), datefilename.end(), ':', '_');
  std::replace( datefilename.begin(), datefilename.end(), '.', '_');
  std::replace( datefilename.begin(), datefilename.end(), ',', '_');
  std::replace( datefilename.begin(), datefilename.end(), ' ', '_');
  std::replace( datefilename.begin(), datefilename.end(), '-', '_');

 // cout<<_statsfilename<<endl;
 if (_subjectID.compare("none")!=0)
  {
    _statsOutputFile.open(ros::package::getPath(std::string("surgical_sim")) + "/data/log/"+ _subjectID + "_" + datefilename + ".txt");
  }

  // Subscriber definitions
  signal(SIGINT, targetObject::stopNode);

  if (_n.ok()) {
    srand(time(NULL));
    ros::spinOnce();
    ROS_INFO("[%s target]: The target spawner is about to start ", Tool_Names[_myTrackID]);
    return true;
  }
  else {
    ROS_ERROR("[%s target]: The ros node has a problem.", Tool_Names[_myTrackID]);
    return false;
  }
}

void targetObject::stopNode(int sig) { me->_stop = true;  }

void targetObject::run() {

  while (!_stop) {
    _hapticTorques.setZero();
       
    readTFTool();
    computeToolTipDerivatives();
    readTFTrocar(); 
    if (_flagTrocarTFConnected)
    {   
      if (!_flagTargetSpawned)
      {
        computetargetObjectPose();  
      }
      else
      {
        if (_flagToolJointsConnected && _flagGazeboLinkStateRead)
        { 
          getGazeboTargetCurrentPos();
          writeTFtargetObject();
          publishMarkerTargetRviz(visualization_msgs::Marker::ADD, WHITE,0.0);
          writeTFtargetAim();
          _tfBroadcaster->sendTransform(_msgAllTransforms);
          evaluateTarget();
          ROS_INFO_ONCE("[%s target]: It is possible to evaluate the target now",Tool_Names[_myTrackID]);
        }
        else
        {
          ROS_WARN("[%s target]: Not possible to evaluate target, possible the tool is not present in the scenario", Tool_Names[_myTrackID]);
        }
      }
      recordStatistics();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }
  me->_statsOutputFile.close();
  ROS_INFO("[%s target]: The target spawner stopped", Tool_Names[_myTrackID]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void targetObject::readTFTool() {

  static int count = 0;
  std::string original_frame;
  std::string destination_frame;

    destination_frame = std::string(Tool_Names[_myTrackID]) + "_tool_tip_link_ee";
    original_frame = "torso_link";

    geometry_msgs::TransformStamped toolTipTransform_;

  try 
  {
    toolTipTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

      if (!_flagToolTipTFConnected) {
        _toolTipPositionPrev = _toolTipPosition;
        tf::vectorMsgToEigen(toolTipTransform_.transform.translation,
                             _toolTipPosition);
        _toolTipQuaternionPrev = _toolTipQuaternion;
        tf::quaternionMsgToEigen(toolTipTransform_.transform.rotation,
                                 _toolTipQuaternion);
        _toolTipRotationMatrix =
            _toolTipQuaternion.normalized().toRotationMatrix();
      }
      _flagToolTipTFConnected = true;

    } 
    catch (tf2::TransformException ex) 
    {
      if (count > 2) {
        ROS_ERROR("[%s target]: %s", Tool_Names[_myTrackID], ex.what());
        count = 0;
      } else {
        count++;
      }
      //ros::Duration(1.0).sleep();
    }
}

void targetObject::readTFTrocar() {


    static int count = 0;
    std::string original_frame;
    std::string destination_frame;

    destination_frame = std::string(Tool_Names[_myTrackID]) + "_tool_base_link";
    original_frame = "torso_link";

    geometry_msgs::TransformStamped trocarTransform_;

    try {
      trocarTransform_ = _tfBuffer.lookupTransform(
          original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

      if (!_flagTrocarTFConnected) {

        tf::vectorMsgToEigen(trocarTransform_.transform.translation,
                             _trocarPosition);
        tf::quaternionMsgToEigen(trocarTransform_.transform.rotation,
                                 _trocarQuaternion);
        _trocarRotationMatrix =
            _trocarQuaternion.normalized().toRotationMatrix();
      }
      // cout<<_trocarRotationMatrix<<endl;
      _flagTrocarTFConnected = true;

    } catch (tf2::TransformException ex) {
      if (count > 2) {
        ROS_ERROR("[%s target]: %s", ex.what(), Tool_Names[_myTrackID]);
        count = 0;
      } else {
        count++;
      }
      ros::Duration(1.0).sleep();
    }
  
}


void targetObject::computetargetObjectPose(){
    if(_flagTrocarTFConnected)
    {
      while (_xTargetPrev==_xTarget)
      {
          generateRandomTarget(&_xTarget,&_myRandomAngle);
      }
      // if( Utils_math<double>::isInRange(-_trocarPosition(CART_X) + _targetsXYZ[CART_X].second[_xTarget], _cartesianLimsTools(CART_X,L_MIN) - 1.5*_myVirtualJointLims(CART_X,L_MIN), _cartesianLimsTools(CART_X,L_MAX)- 1.5*_myVirtualJointLims(CART_X,L_MAX)) 
      //     && Utils_math<double>::isInRange(-_trocarPosition(CART_Y) + _targetsXYZ[CART_Y].second[_xTarget], _cartesianLimsTools(CART_Y,L_MIN)- 1.5*_myVirtualJointLims(CART_Y,L_MIN), _cartesianLimsTools(CART_Y,L_MAX)- 1.5*_myVirtualJointLims(CART_Y,L_MAX))    
      //   ) 
      if(true)
      {
        _myPositionSpawn << _targetsXYZ[CART_X].second[_xTarget], _targetsXYZ[CART_Y].second[_xTarget], _targetsXYZ[CART_Z].second[_xTarget] + 0.025;
        // cout<<"Next Position: "<<_myPositionSpawn.transpose()<<endl; 
        Eigen::Vector3d targetTrocarDistance = _trocarPosition-_myPositionSpawn;
        // cout<<"targetTrocarDistanceSpawn: "<<targetTrocarDistance.transpose()<<endl; 
        _myRotationMatrixSpawn.setIdentity();
        if (targetTrocarDistance.norm() > FLT_EPSILON) 
        {
          _myRotationMatrixSpawn = Utils_math<double>::rodriguesRotation(Eigen::Vector3d(0.0, 0.0, 1.0), targetTrocarDistance);
        }
        _myRotationMatrixSpawn =  _myRotationMatrixSpawn * AngleAxis<double>(_myRandomAngle, Vector3d::UnitZ()).toRotationMatrix();
        _myQuaternionSpawn = Eigen::Quaternion<double>(_myRotationMatrixSpawn);
        if (gazeboDeleteModel())
        {
          _flagTargetSpawned = gazeboSpawnModel();
        if (_flagTargetSpawned)
          {
            _nTarget++;
            ROS_INFO("[%s target]: New target generated!. # %i",Tool_Names[_myTrackID],_nTarget);
            _myStatus[TARGET_CHANGED]=0;
            _xTargetPrev=_xTarget;
            if (_nTarget>=NB_TARGETS)
            {
              ROS_INFO("[%s target]: Protocol finished", Tool_Names[_myTrackID]);
              publishTargetReachedSphere(visualization_msgs::Marker::ADD, RED,0.0);
              _stop=true;
            }
          }
        }
      else
      {
        _flagTargetSpawned = false;
      }
        // Generate Aim Frame
        
        _targetAimPosition = _myPositionSpawn + Eigen::Vector3d(0.0,0.0,0.7*_myVirtualJointLims(CART_Z,L_MAX));
        double randomAngleAim = _myRandomAngle;
        generateRandomTarget(NULL,&randomAngleAim); // Generate an aim orientation different than that in which we spawned the target
        _targetAimQuaternion = _myQuaternionSpawn * AngleAxis<double>(randomAngleAim, Vector3d::UnitZ()).toRotationMatrix();
    } else {
      
      ROS_INFO_ONCE("[%s target]: Looking for a target inside the workspace of the tool...", Tool_Names[_myTrackID] );
      _flagTargetSpawned = false;    
    }
  }
}


void targetObject::evaluateTarget()
{  
  _precisionPos = (_myPositionCurrent - _targetAimPosition).norm();
  _precisionAng = _myQuaternionCurrent.angularDistance(_targetAimQuaternion);
  double errorAng = 1.0-cos(_precisionAng);
  // std::cout<<"Precision in angle:" << errorAng<<std::endl;
  if (_precisionPos < 0.007)
  {
    _myStatus[TARGET_REACHED]=true;
   
  } else {
    _flagTargetReached=false; 
    // _flagTargetReachedOpen=false;
     _myStatus[TARGET_REACHED]=false;
  }
  if (errorAng < (1-cos(7.0*DEG_TO_RAD)))
  {
    _myStatus[TARGET_ALIGNED]=true;
    
  } else{
    _myStatus[TARGET_ALIGNED]=false;
  }  

  if ( _myStatus[TARGET_REACHED] && !_myStatus[TARGET_ALIGNED])
  {
     if(!_flagTargetReached)
    {
       _flagTargetReached=true;
      //  _flagTargetReachedOpen=true; 
       ROS_INFO_ONCE("[%s target]: YOU MANAGED THE POSITIONING", Tool_Names[_myTrackID]);   
    }
    publishTargetReachedSphere(visualization_msgs::Marker::ADD, CYAN,0.0);
  }

  if ( _myStatus[TARGET_REACHED] && _myStatus[TARGET_ALIGNED])
  {
    if (!_flagTargetGrasped)
    {
          _startDelayForCorrection = ros::Time::now(); 
          // cout<<"startDelay: "<<_startDelayForCorrection.toNSec()<<endl;
          ROS_INFO_ONCE("[%s target]: YOU MANAGED THE ALIGNMENT", Tool_Names[_myTrackID]);
          _flagTargetGrasped=true;
    }  
    publishTargetReachedSphere(visualization_msgs::Marker::ADD, YELLOW,0.0);
  }

  if ( !_myStatus[TARGET_REACHED] && !_myStatus[TARGET_ALIGNED])
  {
    _startDelayForCorrection = ros::Time::now(); 
    publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
    _flagTargetGrasped=false;
  }



  if (_myStatus[TARGET_REACHED] && _myStatus[TARGET_ALIGNED] && _nTarget < NB_TARGETS) {
    
    if ((ros::Time::now() - _startDelayForCorrection).toSec() > DELAY_SEC){ 
      // generateRandomTarget(&_xTarget, &_myRandomAngle);
      // _startDelayForCorrection=ros::Time::now();
      _myStatus[TARGET_CHANGED]=true;
      _myStatus[TARGET_REACHED]=false;
      _myStatus[TARGET_ALIGNED]=false;
      _flagTargetSpawned=false;
      publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
    }
  }
}


/*
void targetObject::evaluateTarget()
{  
  _precisionGrasp = 15.0*DEG_TO_RAD - _toolJointPosition(tool_wrist_open_angle);
  _precisionPos = (_myPositionCurrent - _toolTipPosition).norm();
  // std::cout<<"Precision in position:" << _precisionPos<<std::endl;
  _precisionAng = _myQuaternionCurrent.angularDistance(_toolTipQuaternion);
  double errorAng = 1.0-cos(_precisionAng);
  // std::cout<<"Precision in angle:" << errorAng<<std::endl;
  if (_precisionPos < 0.007 && errorAng < (1-cos(7.0*DEG_TO_RAD)))
  {
     _flagTargetReached=true;
     _myStatus=TARGET_REACHED;
    //  if (_flagTargetReachedOpen || (_toolJointPosition(tool_wrist_open_angle)>20.0*DEG_TO_RAD))
     {
      // _flagTargetReachedOpen=true; 
      publishTargetReachedSphere(visualization_msgs::Marker::ADD, CYAN,0.0);
        
      if (_toolJointPosition(tool_wrist_open_angle)<=17.0*DEG_TO_RAD)
      {
          publishTargetReachedSphere(visualization_msgs::Marker::ADD, YELLOW,0.0);
          if (!_flagTargetGrasped)
          {
            _startDelayForCorrection = ros::Time::now(); 
            std::cout<<"grasped"<<std::endl;
          }
          _flagTargetGrasped=true;
          _myStatus=TARGET_ALIGNED;
      }
      else
      {
        _startDelayForCorrection = ros::Time::now(); 
        _flagTargetGrasped=false;
      }
     }
  }
  else
  {
    _startDelayForCorrection = ros::Time::now(); 
    _flagTargetReached=false; 
    // _flagTargetReachedOpen=false;
    _flagTargetGrasped=false; 
    _myStatus=TARGET_NOT_REACHED;
     publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
  }

  if (_myStatus==TARGET_ALIGNED && _nTarget < NB_TARGETS)
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
    _flagTargetReached =false;
    _startDelayForCorrection=ros::Time::now();
    ROS_INFO("[%s target]: New target generated!. # %i",_nTarget, Tool_Names[_myTrackID]);
    _myStatus = TARGET_CHANGED;
    _flagTargetSpawned=false;
    publishTargetReachedSphere(visualization_msgs::Marker::DELETE, NONE,0.0);
    }
  }

  if (_nTarget>=NB_TARGETS)
  {
    ROS_INFO("[%s target]: Protocol finished", Tool_Names[_myTrackID]);
    publishTargetReachedSphere(visualization_msgs::Marker::ADD, RED,0.0);
    _stop=true;
  }

}
*/


void targetObject::writeTFtargetObject(){
  _msgAllTransforms[TF_TARGET_OBJECT].child_frame_id=std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_main_link";
  _msgAllTransforms[TF_TARGET_OBJECT].header.frame_id="torso_link";
  _msgAllTransforms[TF_TARGET_OBJECT].header.stamp = ros::Time::now();

  tf::vectorEigenToMsg(_myPositionCurrent,_msgAllTransforms[TF_TARGET_OBJECT].transform.translation);
  tf::quaternionEigenToMsg(_myQuaternionCurrent,_msgAllTransforms[TF_TARGET_OBJECT].transform.rotation); 

  // _msgAllTransforms[TF_TARGET_OBJECT] = _msgtargetObjectTransform;
  
}

void targetObject::writeTFtargetAim(){
  _msgAllTransforms[TF_TARGET_AIM].child_frame_id=std::string(Tool_Names[_myTrackID])+"_"+_myName+"_target_aim";
  _msgAllTransforms[TF_TARGET_AIM].header.frame_id="torso_link";
  _msgAllTransforms[TF_TARGET_AIM].header.stamp = ros::Time::now();

  tf::vectorEigenToMsg(_targetAimPosition,_msgAllTransforms[TF_TARGET_AIM].transform.translation);
  tf::quaternionEigenToMsg(_targetAimQuaternion,_msgAllTransforms[TF_TARGET_AIM].transform.rotation); 

  // _msgAllTransforms[TF_TARGET_AIM] = _msgtargetAimTransform;
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
  
  _msgTargetReachedSphere.header.frame_id = std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target_main_link";
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

void targetObject::publishMarkerTargetRviz (int32_t action_,Marker_Color color_, double delay_){
  


  _msgRvizTarget.header.frame_id = std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target_main_link";
  _msgRvizTarget.header.stamp = ros::Time::now();
  _msgRvizTarget.ns = "";
  _msgRvizTarget.id = 1;
  _msgRvizTarget.type = visualization_msgs::Marker::MESH_RESOURCE;
  _msgRvizTarget.mesh_resource = "package://surgical_sim/models/meshes/stl/target/target_translated.STL";
  _msgRvizTarget.action = action_;
  _msgRvizTarget.pose.position.x = 0.0;
  _msgRvizTarget.pose.position.y = 0.0;
  _msgRvizTarget.pose.position.z = 0.0;
  _msgRvizTarget.pose.orientation.x = 0.0;
  _msgRvizTarget.pose.orientation.y = 0.0;
  _msgRvizTarget.pose.orientation.z = 0.0;
  _msgRvizTarget.pose.orientation.w = 1;
  _msgRvizTarget.scale.x = 1.00;
  _msgRvizTarget.scale.y = 1.00;
  _msgRvizTarget.scale.z = 1.00;
  _msgRvizTarget.color.a = 1.0; // Don't forget to set the alpha!
  switch (color_)
  {
  case WHITE:
  {
    _msgRvizTarget.color.r = 1;
    _msgRvizTarget.color.g = 1;
    _msgRvizTarget.color.b = 1;  
    break;
  }  
  case YELLOW:
  {
    _msgRvizTarget.color.r = 1;
    _msgRvizTarget.color.g = 1;
    _msgRvizTarget.color.b = 0;  
    break;
  }
  case RED:
  {
    _msgRvizTarget.color.r = 1;
    _msgRvizTarget.color.g = 0;
    _msgRvizTarget.color.b = 0;  
    break;
  }
  case CYAN:
  {
    _msgRvizTarget.color.r = 0;
    _msgRvizTarget.color.g = 1;
    _msgRvizTarget.color.b = 1;  
    break;
  }
  default:
  {
    break;
  }
  }
  _msgRvizTarget.lifetime= ros::Duration(delay_);
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubTargetReachedSphere.publish(_msgRvizTarget);
  // _mutex.unlock();
}

void targetObject::recordStatistics(){

  // ros::Duration deltaTime = ros::Time::now() - _startingTime;
  // double precisionPOS_on = _myStatus != TARGET_NOT_REACHED ? _precisionPos : 0.0;
  // double precisionANG_on = _myStatus != TARGET_NOT_REACHED ? _precisionAng * RAD_TO_DEG : 0.0;
  
  if (_subjectID.compare("none")!=0)
	{
	ros::Duration deltaTime = ros::Time::now() - _startingTime;
  double precisionPOS_on = _flagTargetReached ? _precisionPos : 0.0;
  double precisionANG_on = _flagTargetReached ? _precisionAng * RAD_TO_DEG : 0.0;
  double precisionGRASP_on = _flagTargetGrasped ? _precisionGrasp * RAD_TO_DEG : 0.0;
  
    if (_flagRecordingStarted)
    {
		_statsOutputFile << deltaTime<< " "
					<< _nTarget<< " "
          << _myStatus[TARGET_REACHED]<< " "
          << _myStatus[TARGET_ALIGNED]<< " "
          << _myStatus[TARGET_CHANGED]<< " "
          << _myPositionSpawn.transpose()<<" "
          << _myQuaternionSpawn.x()<<" "
          << _myQuaternionSpawn.y()<<" "
          << _myQuaternionSpawn.z()<<" "
          << _myQuaternionSpawn.w()<<" "
          << _myPositionCurrent.transpose()<<" "
          << _myQuaternionCurrent.x()<<" "
          << _myQuaternionCurrent.y()<<" "
          << _myQuaternionCurrent.z()<<" "
          << _myQuaternionCurrent.w()<<" "
          << _toolJointPosition(tool_wrist_open_angle)<<" "
          << _toolJointSpeed(tool_wrist_open_angle)<<" "
          << _toolTipPosition.transpose() << " "
          << _toolTipSpeed.transpose() << " "
          << _toolTipAcc.transpose() << " "
          << _toolTipJerk.transpose() << " "
          << _toolTipQuaternion.x()<<" "
          << _toolTipQuaternion.y()<<" "
          << _toolTipQuaternion.z()<<" "
          << _toolTipQuaternion.w()<<" "
          << _toolJointPosition(tool_yaw)<<" "
          <<  precisionPOS_on<<" "
          <<  precisionANG_on<<" "
          <<  precisionGRASP_on<<" "
          << _footBaseWorldForce.force.x<<" "
          << _footBaseWorldForce.force.y<<" "
          << _footBaseWorldForce.force.z<<" "
          << _footBaseWorldForce.torque.x<<" "
          << _footBaseWorldForce.torque.y<<" "
          << _footBaseWorldForce.torque.z<<" "
          << _platformJointPosition.transpose()<<" "
          << _platformJointVelocity.transpose()<<" "
          << _platformJointEffortD.transpose()<<" "
          << _platformJointEffortM.transpose()<<" "
          << _platformJointEffortRef.transpose()<<" "
          << _legGravityCompTorques.transpose()<<" "
          << _hapticTorques.transpose()<<" "
          << _hapticAxisFilterPos<<" "
          << _hapticAxisFilterGrasp<<" "
          << _myThreshold<<" "
          << std::endl;
    }
    else
    {
      _statsOutputFile << "t"<< " "
					<< "nT"<< " "
          << "statReached"<< " "
          << "statGrasped"<< " "
          << "statChanged"<< " "
          << "posTXSpawn"<<" "
          << "posTYSpawn"<<" "
          << "posTZSpawn"<<" "
          << "quatTXSpawn"<<" "
          << "quatTYSpawn"<<" "
          << "quatTZSpawn"<<" "
          << "quatTWSpawn"<<" "
          << "posTXCurrent"<<" "
          << "posTYCurrent"<<" "
          << "posTZCurrent"<<" "
          << "quatTXCurrent"<<" "
          << "quatTYCurrent"<<" "
          << "quatTZCurrent"<<" "
          << "quatTWCurrent"<<" "
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

  void targetObject::readToolState(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_TOOL_AXIS_FULL; i++) {
    me->_toolJointPosition(i) = msg->position[i];
    me->_toolJointSpeed(i) = msg->velocity[i];
  }

  if (!_flagToolJointsConnected) {
    ROS_INFO("[%s target]: Joints received from tool", Tool_Names[_myTrackID]);
  }
  _flagToolJointsConnected = true;
}

  void targetObject::readPlatformState(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev(i) = me->_platformJointStates(i);
    me->_platformJointPosition(i) = msg->position[i];
    me->_platformJointVelocity(i) = msg->velocity[i];
    me->_platformJointEffortD(i) = msg->effort[i];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("[%s target]: Joints received from platform", Tool_Names[_myTrackID]);
  }
  _flagPlatformJointsConnected = true;
  }

  void targetObject::readLegGravityCompTorques(const custom_msgs::FootInputMsg_v5::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev(i) = me->_platformJointStates(i);
    me->_legGravityCompTorques(i) = msg->ros_effort[i];
  }

  if (!_flagLegGravityTorquesConnected) {
    ROS_INFO("[%s target]: Platform Input Connected", Tool_Names[_myTrackID]);
  }
  _flagLegGravityTorquesConnected = true;
  }

  void targetObject::readFIOutput(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev(i) = me->_platformJointStates(i);
    me->_platformJointPosition(i) = msg->platform_position[Axis_Mod[i]] * conversion_factor[i];
    me->_platformJointVelocity(i) = msg->platform_speed[Axis_Mod[i]] * conversion_factor[i];
    me->_platformJointEffortD(i) = msg->platform_effortD[Axis_Mod[i]];
    me->_platformJointEffortRef(i) = msg->platform_effortRef[Axis_Mod[i]];
    // me->_platformJointEffortM(i) = msg->platform_effortM[Axis_Mod[i]];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("[%s target]: Joints received from platform", Tool_Names[_myTrackID]);
  }
  _flagPlatformJointsConnected = true;
}



void targetObject::computeToolTipDerivatives(){
    double freq = 1.0/_dt;
    _toolTipSpeedPrev = _toolTipSpeed;
    _toolTipSpeed= _toolTipSpeedFilter->update((_toolTipPosition - _toolTipPositionPrev) * freq);
    _toolTipAccPrev = _toolTipAcc;
    _toolTipAcc= _toolTipAccFilter->update((_toolTipSpeed - _toolTipSpeedPrev) * freq);
    _toolTipJerk= _toolTipJerkFilter->update((_toolTipAcc - _toolTipAccPrev) * freq);
}


void targetObject::readForceFootRestWorld(const geometry_msgs::WrenchStamped::ConstPtr &msg){
    _footBaseWorldForce = msg->wrench;
  if (!_flagFootBaseForceConnected)
	 {
		ROS_INFO("[%s target]: Reading forces in the foot base w.r.t. world", Tool_Names[_myTrackID]);
    _flagFootBaseForceConnected = true;
	 }
}

void targetObject::readUnbiasedJointTorques(const custom_msgs::FootOutputMsg_v3::ConstPtr &msg){
    for (size_t i = 0; i < NB_AXIS; i++)
    {
      _platformJointEffortM(i) = msg->platform_effortM[i];
    }    
}

void targetObject::readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg)
{
    _hapticAxisFilterPos = msg->sGrasp_hFilters[p_x];  
    _hapticAxisFilterGrasp = msg->sGrasp_hFilters[p_roll];
    _myThreshold = msg->sGrasp_threshold;   
    _aState = (targetObject::Action_State) msg->sGrasp_aState;
    for (size_t i = 0; i < NB_AXIS; i++)
    {
      _hapticTorques(i) = msg->sGrasp_hapticTorques[i];
    }
}
  
  
void targetObject::readGazeboLinkStates(const gazebo_msgs::LinkStates::ConstPtr &msg){
  
  _msgGazeboLinkStates = *msg;
  _flagGazeboLinkStateRead=true;
}

void targetObject::getGazeboTargetCurrentPos()
{
  Eigen::Vector3d torsoPosition; 
  Eigen::Quaterniond torsoQuaternion; 
  Eigen::Vector3d targetWorldPosition;
  Eigen::Quaterniond targetWorldQuaternion; 

  torsoPosition.setZero(); targetWorldPosition.setZero(); torsoQuaternion.setIdentity(); targetWorldQuaternion.setIdentity();

  if (_flagGazeboLinkStateRead)
  {
    for (size_t i = 0; i < _msgGazeboLinkStates.name.size() ; i++)
    {
      if (_msgGazeboLinkStates.name[i].compare("body::torso_link")==0)
      {
        tf::pointMsgToEigen(_msgGazeboLinkStates.pose[i].position,
                              torsoPosition);
        tf::quaternionMsgToEigen(_msgGazeboLinkStates.pose[i].orientation,
                              torsoQuaternion);
        //std::cout<<"torso position"<<torsoPosition.transpose()<<std::endl;
      }
      else if (_msgGazeboLinkStates.name[i].compare(std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target::"+std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target_main_link")==0)
      {
        tf::pointMsgToEigen(_msgGazeboLinkStates.pose[i].position,
                              targetWorldPosition);
        _myPositionCurrent=torsoQuaternion.inverse()._transformVector(targetWorldPosition - torsoPosition);
       //std::cout<<"target position"<<_myPositionCurrent.transpose()<<std::endl;

        tf::quaternionMsgToEigen(_msgGazeboLinkStates.pose[i].orientation,
                                   targetWorldQuaternion);
        _myQuaternionCurrent = torsoQuaternion.inverse()*targetWorldQuaternion ;
        _myRotationMatrixCurrent = _myQuaternionCurrent.normalized().toRotationMatrix();
      }
    }
  }
}

bool targetObject::gazeboSpawnModel(){


  _srvGzSpawnModel.request.model_xml = _myModelXml.c_str();
  // _srvGzSpawnModel.request.model_xml = std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target/robot_description";
  _srvGzSpawnModel.request.model_name =  std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target";
  // _srvGzSpawnModel.request.link_state.link_name = std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target::"+std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target_main_link";
  _srvGzSpawnModel.request.robot_namespace = "/" + std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target";
  _srvGzSpawnModel.request.reference_frame = "body::torso_link";
  geometry_msgs::Point positionToSpawn;
  std::cout<<"Spawning Position: "<<_myPositionSpawn.transpose()<<std::endl;
  tf::pointEigenToMsg(_myPositionSpawn,positionToSpawn);
  _srvGzSpawnModel.request.initial_pose.position = positionToSpawn;
  geometry_msgs::Quaternion quaternionToSpawn;
  tf::quaternionEigenToMsg(_myQuaternionSpawn,quaternionToSpawn);
  _srvGzSpawnModel.request.initial_pose.orientation = quaternionToSpawn;

  if (_clientSpawnNewTargetAtPos.call(_srvGzSpawnModel))
  {
    ROS_INFO("[%s target]: Target (re-)spawned to a new position", Tool_Names[_myTrackID]);
    return true;
  }
  else
  {
    ROS_ERROR("[%s target]: Error in (re-)spawning the target to a new position", Tool_Names[_myTrackID]);
    return false;
  }
}

bool targetObject::gazeboDeleteModel(){
  // try
  // {
    _srvGzDeleteModel.request.model_name = std::string(Tool_Names[_myTrackID])+"_" + _myName +"_target";

    if (_clientDeleteOldTarget.call(_srvGzDeleteModel))
    {
      ROS_INFO("[%s target]: Target ready to change position", Tool_Names[_myTrackID]);
      return true;
    }
    else
    {
      ROS_ERROR("[%s target]: Error in changing target position", Tool_Names[_myTrackID]);
      return false;
    }
  // }

  // catch(const std::exception& ex)
  // {
  //   if (_xTarget>0)
  //   {
  //     ROS_ERROR("[%s target]: %s", ex.what(, Tool_Names[_myTrackID]));
  //   }

  // }

}

void targetObject::generateRandomTarget(int* xTarget_,double* randomAngle_)
{
  if (xTarget_!=NULL)
  {
    int newTargetNum = *xTarget_;
      while (*xTarget_==newTargetNum)
      {
         newTargetNum = rand() % (NB_TARGETS-1);
        //std::cout<<newTargetNum<<std::endl;
        if (*xTarget_!=newTargetNum)
          {
            break;
          }
      }
      *xTarget_ = newTargetNum; // Generate new target randomly
  }
  if (randomAngle_!=NULL)
  {
    double randomAngleNew = *randomAngle_;
    while(*randomAngle_ == randomAngleNew)
    {
      randomAngleNew = Utils_math<double>::bound(- 1.5 * M_PI + ((rand()%(NB_TARGETS-1)) * 3.0 * M_PI)/((NB_TARGETS-1)), -1.5 * M_PI, 1.5 * M_PI ) ;
       if (*randomAngle_!=randomAngleNew)
        {
          break;
        }
    }
    *randomAngle_ = randomAngleNew;
  }
}
  
  
  
  
  
  
    
