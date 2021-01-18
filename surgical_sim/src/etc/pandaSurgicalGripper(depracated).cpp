#include "pandaSurgicalGripper.h"
#include <stdio.h>
#include <stdlib.h>
#include "tf_conversions/tf_eigen.h"
#include "Utils_math.h"
#include <iostream>

#define ListofPandaToolAxes(enumeration, names) names,
char const *Panda_Tool_Axis_Names[]{PANDA_TOOL_AXES};
#undef ListofPandaToolAxes

// #define ListofLegAxes(enumeration, names) names,
// char const *Leg_Axis_Names[]{LEG_AXES};
// #undef ListofLegAxes

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{PLATFORM_AXES};
#undef ListofPlatformAxes


char const *Panda_Tool_Names[]{"none", "right", "left"};
// char const *Leg_Names[]{"none", "right", "left"};
char const *Platform_Names[]{"none", "right", "left"};

pandaSurgicalGripper *pandaSurgicalGripper::me = NULL;



#define MIN_THRESHOLD 5.0
#define MAX_THRESHOLD 15.0
#define STD 0.7

const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};



pandaSurgicalGripper::pandaSurgicalGripper(ros::NodeHandle &n_1, double frequency,
                   pandaSurgicalGripper::Panda_Tool_Name tool_id, urdf::Model model_)
    : _n(n_1), _pandaTool_id(tool_id), _loopRate(frequency), _dt(1.0f / frequency),
      _myModel(model_) {
   me = this;
  _stop = false;

  _myInput = PLATFORM_INPUT;
  std::string inputName_;

  if (!_n.getParam("controlInput", inputName_))
    { ROS_ERROR("No controlInput param"); }

  if (inputName_ == "platform")
  {
    ROS_INFO("Control input of the tool changed to platform");
    _myInput=PLATFORM_INPUT;
  }
  //std::cout<<_myInput<<endl;
  _pandaToolJoints = new KDL::JntArray(NB_PANDA_TOOL_AXIS_RED);
  _pandaToolJoints->data.setZero();
  _pandaToolJointsPrev = new KDL::JntArray(NB_PANDA_TOOL_AXIS_RED);
  _pandaToolJointsPrev->data.setZero();
  _pandaToolJointsFull = new KDL::JntArray(NB_PANDA_TOOL_AXIS_FULL);
  _pandaToolJointsFull->data.setZero();
  _pandaToolJointsInit = new KDL::JntArray(NB_PANDA_TOOL_AXIS_RED);
  _pandaToolJointsInit->data.setZero();

  // _legJointLims[L_MIN] = new KDL::JntArray(NB_LEG_AXIS);
  // _legJointLims[L_MIN]->data.setZero();
  // _legJointLims[L_MAX] = new KDL::JntArray(NB_LEG_AXIS);
  // _legJointLims[L_MAX]->data.setZero();

  _platformJointLims[L_MIN] = new KDL::JntArray(NB_PLATFORM_AXIS);
  _platformJointLims[L_MIN]->data.setZero();
  _platformJointLims[L_MAX] = new KDL::JntArray(NB_PLATFORM_AXIS);
  _platformJointLims[L_MAX]->data.setZero();
  _platformJointLimsDelta = new KDL::JntArray(NB_PLATFORM_AXIS);

  _pandaToolJointLims[L_MIN] = new KDL::JntArray(NB_PANDA_TOOL_AXIS_RED);
  _pandaToolJointLims[L_MIN]->data.setZero();
  _pandaToolJointLims[L_MAX] = new KDL::JntArray(NB_PANDA_TOOL_AXIS_RED);
  _pandaToolJointLims[L_MAX]->data.setZero();
  _pandaToolJointLimsAll[L_MIN] = new KDL::JntArray(NB_PANDA_TOOL_AXIS_FULL);
  _pandaToolJointLimsAll[L_MIN]->data.setZero();
  _pandaToolJointLimsAll[L_MAX] = new KDL::JntArray(NB_PANDA_TOOL_AXIS_FULL);
  _pandaToolJointLimsAll[L_MAX]->data.setZero();

   //_weightedJointSpaceMassMatrix.setIdentity();
  _weightedTaskSpaceMassMatrix.setIdentity();
  _myJacobian.resize(NB_PANDA_TOOL_AXIS_RED);

  _desiredTargetFrame.M.Identity();
  _desiredTargetFrame.p.Zero();
  _trocarBasePoseFrame.M.Identity();
  _worldPandaLink0Frame.M.Identity();
  _trocarBasePoseFrame.p.Zero();
  _worldPandaLink0Frame.p.Zero();

  // _flagSharedGrasp=false;
  _flagTrocarBasePoseRetrieved =  false;
  _flagPandaLink0FrameRetrieved =  false;
  _flagPantaToolJointInitRetrieved = false;
  _flagPantaToolJointsRetrieved = false;
  // _flagLegJointsConnected = false;
  _flagPlatformJointsConnected = false;
  _flagRosControl=false;

  _flagToolJointLimitsOffsetCalculated = false;
  
  // _flagLegJointLimitsOffsetCalculated = false;
  
  _flagPlatformJointLimitsOffsetCalculated = false;
  _pandaToolJointsAll.setZero(); 
  _pandaToolJointsGet.setZero();
  _pandaToolJointsPosFiltered.setZero();
  _pandaToolJointsAllPrev.setZero();
  _pandaToolJointsAllSpeed.setZero();
  
  _pandaToolJointsAllOffset.setZero();
  
   _hAxisFilterPosValue.setZero();
   _hAxisFilterGraspValue=0.0;

   _hAxisFilterPos = new MatLP_Filterd(_hAxisFilterPosValue);
   _hAxisFilterPos->setAlphas(_hAxisFilterPosValue);

   _hAxisFilterGrasp = new LP_Filterd(_hAxisFilterGraspValue);
   _hAxisFilterGrasp->setAlpha(_hAxisFilterGraspValue);

  // _legJoints.setZero();
  // _legJointsOffset.setZero();

  _platformJoints.setZero();
  _platformJointsOffset.setZero();
  _platformVelocities.setZero();
  _platformEfforts.setZero();

  _msgJointCommands.data.resize(NB_PANDA_TOOL_AXIS_FULL);

  // if (!_n.getParam("sharedGraspOn", _flagSharedGrasp))
  // { 
  //     ROS_ERROR("No sharedGrasp param"); 
  // }

  // if (_flagSharedGrasp) cout<<"flag shared grasp enabled"<<endl;

  if (!_n.getParam("rosControl", _flagRosControl))
  { 
      ROS_ERROR("No rosControl param"); 
  }

  if (_flagRosControl) cout<<"flag ros control for the tool enabled"<<endl;

  // if (_myInput==LEG_INPUT)
  // {
  //   if (!_legModel.initParam("/"+std::string(Panda_Tool_Names[_pandaTool_id]) + "_leg/robot_description")) {
  //       ROS_ERROR("Failed to parse urdf of the leg");
  //   }
  // for (unsigned int i = 0; i<NB_LEG_AXIS; i++)
  // {
  //   _legJointLims[L_MIN]->data(i) = _legModel.getJoint("/"+std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + Leg_Axis_Names[i])->limits->lower;
  //   _legJointLims[L_MAX]->data(i) = _legModel.getJoint("/"+std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + Leg_Axis_Names[i])->limits->upper;
  // }
  //}
  if (_myInput==PLATFORM_INPUT)
  {
    if (!_platformModel.initParam("/"+std::string(Panda_Tool_Names[_pandaTool_id]) + "_platform/robot_description")) {
        ROS_ERROR("Failed to parse urdf file");
    }

    for (unsigned int i = 0; i<NB_PLATFORM_AXIS; i++)
    {
      _platformJointLims[L_MIN]->data(i) = _platformModel.getJoint(std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->lower;
      //cout<<_platformJointLims[L_MIN]->data(i)<<endl;
      
      _platformJointLims[L_MAX]->data(i) = _platformModel.getJoint( std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->upper;
     // cout<<_platformJointLims[L_MAX]->data(i)<<endl;
      _platformJointLimsDelta->data(i) = min(abs(_platformJointLims[L_MIN]->data(i)),abs(_platformJointLims[L_MAX]->data(i)));
    }
  }

    if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
      ROS_ERROR("Failed to construct kdl tree");
      _stop = true;
    }

  KDL::Vector grav_vector(0.0, 0.0, GRAVITY);

  _myTree.getChain(std::string(Panda_Tool_Names[_pandaTool_id]) + "_panda_link0", std::string(Panda_Tool_Names[_pandaTool_id]) + "_panda_tool_wrist_link", _myPandaBaseToWristChain);
  _myTree.getChain(std::string(Panda_Tool_Names[_pandaTool_id]) + "_panda_link0", std::string(Panda_Tool_Names[_pandaTool_id]) + "_panda_tool_tip_link_ee", _myPandaBaseToTipChain);


  _myFKSolver_wrist = new KDL::ChainFkSolverPos_recursive(_myPandaBaseToWristChain);
  
  _myFKSolver_tip = new KDL::ChainFkSolverPos_recursive(_myPandaBaseToTipChain);

  _myJacSolver_wrist = new KDL::ChainJntToJacSolver(_myPandaBaseToWristChain);

  _mySegments = _myPandaBaseToWristChain.segments;
  _mySegmentsTip = _myPandaBaseToTipChain.segments;

  
  for (unsigned int joint_=0; joint_ < NB_PANDA_TOOL_AXIS_FULL; joint_++ )
  {
    _pandaToolJointLimsAll[L_MIN]->data(joint_) = _myModel.getJoint(std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + Panda_Tool_Axis_Names[joint_])->limits->lower;
    _pandaToolJointLimsAll[L_MAX]->data(joint_) = _myModel.getJoint(std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + Panda_Tool_Axis_Names[joint_])->limits->upper;
      
    if (joint_<NB_PANDA_TOOL_AXIS_RED)
    {
      _pandaToolJointLims[L_MIN]->data(joint_) = _pandaToolJointLimsAll[L_MIN]->data(joint_) ;
      _pandaToolJointLims[L_MAX]->data(joint_) = _pandaToolJointLimsAll[L_MAX]->data(joint_) ;
    }
  }


  
  /****************************** Inverse Kinematics of the tool ***********************************/
      _myVelIKSolver_wrist = new KDL::ChainIkSolverVel_wdls(_myPandaBaseToWristChain);
      _myPosIkSolver_wrist= new KDL::ChainIkSolverPos_NR_JL(_myPandaBaseToWristChain,*me->_pandaToolJointLims[L_MIN], *me->_pandaToolJointLims[L_MAX], *_myFKSolver_wrist,*_myVelIKSolver_wrist);
      
      _weightedTaskSpaceMassMatrix.block(3,3,3,3).diagonal()<< FLT_EPSILON, FLT_EPSILON, FLT_EPSILON;

      
      //_myVelIKSolver_wrist->setWeightTS(_weightedTaskSpaceMassMatrix);
      //_myVelIKSolver_wrist->setWeightJS(_weightedJointSpaceMassMatrix);
  /*                                                                                                */
  _tfListener = new tf2_ros::TransformListener(_tfBuffer);

}

pandaSurgicalGripper::~pandaSurgicalGripper() { me->_n.shutdown(); }

bool pandaSurgicalGripper::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{ 
  if (_flagRosControl)
    {
      _pubToolJointCommands = _n.advertise<std_msgs::Float64MultiArray>("/"+std::string(Panda_Tool_Names[_pandaTool_id])+"_panda/joint_position_controller/command", 1);
    }
  else
  {
    // _pubToolJointStates = _n.advertise<sensor_msgs::JointState>("/"+std::string(Panda_Tool_Names[_pandaTool_id])+"_pandaTool/joint_states", 1);
  }
  
  // _pubToolTipPose = _n.advertise<geometry_msgs::PoseStamped>("panda_tool_tip_pose", 1);
 
  /* 
    _subLegJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Panda_Tool_Names[_pandaTool_id])+"_leg/leg_joint_publisher/joint_states"
      , 1, boost::bind(&pandaSurgicalGripper::readLegJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  */
  
  _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_pandaTool_id])+"_platform/platform_joint_publisher/joint_states"
      , 1, boost::bind(&pandaSurgicalGripper::readPlatformJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      

  _subPandaToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_pandaTool_id])+"_panda/joint_states"
      , 1, boost::bind(&pandaSurgicalGripper::readPandaToolJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      

  /* 
  _subSharedGrasp = _n.subscribe<custom_msgs_gripper::SharedGraspingMsg>( "/"+std::string(Platform_Names[_pandaTool_id])+"_pandaTool/sharedGrasping"
      , 1, boost::bind(&pandaSurgicalGripper::readSharedGrasp, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      */

  // Subscriber definitions
  signal(SIGINT, pandaSurgicalGripper::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("The tool joint state publisher "
             "is about to start ");
    return true;
  } else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void pandaSurgicalGripper::stopNode(int sig) { me->_stop = true; }

void pandaSurgicalGripper::run() {
  
  while (!_stop) {
    
    if ((_subPlatformJointStates.getNumPublishers()==0 && _myInput==PLATFORM_INPUT) ) // || 
        //(_subLegJointStates.getNumPublishers()==0 && _myInput==LEG_INPUT))
        {
          ROS_ERROR_ONCE("No control input for the panda_tool connected");
        }
    else 
    {

      if (_flagTrocarBasePoseRetrieved && _flagPandaLink0FrameRetrieved && _flagPantaToolJointInitRetrieved)
      {
            //readFootTipBasePose();
            if (_flagPantaToolJointsRetrieved)
            {
              _pandaToolJointsAll = _pandaToolJointsGet;
            }

            switch (_myInput)
            {
                case PLATFORM_INPUT:
                {
                calculateDesiredFrame();
                performInverseKinematicsWithPlatform();
                break; 
                }
                default:
                {
                  break;  
                }
            }  
            if (_flagRosControl) 
            { 
              publishPandaToolJointCommands();  
            }
            else 
            { 
                // publishPandaToolJointStates();
            }

            performChainForwardKinematics();
            _pandaToolJointsAllSpeed = (_pandaToolJointsAll - _pandaToolJointsAllPrev) * (1.0/_dt);
            _flagPantaToolJointsRetrieved = false;
      }
      else
      {
        readTrocarBasePose();
        readPandaLink0Frame();
      }
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("Tool state variables stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}




void pandaSurgicalGripper::publishPandaToolJointCommands() {
  for (int k = 0; k < NB_PANDA_TOOL_AXIS_FULL; k++) {
    _msgJointCommands.data[k] = me->_pandaToolJointsAll(k);
  }
  _pubToolJointCommands.publish(_msgJointCommands);
}




void pandaSurgicalGripper::calculateDesiredFrame(){
  
  KDL::Frame targetFrame;
  KDL::Frame insertionFrame;
  
  targetFrame.p.data[0] = Utils_math<double>::map( _platformJoints(p_x),
                                                          -_platformJointLimsDelta->data(p_x), _platformJointLimsDelta->data(p_x), 
                                                          -0.15,0.15);
  targetFrame.p.data[1] = Utils_math<double>::map( _platformJoints(p_y),
                                                          -_platformJointLimsDelta->data(p_y), _platformJointLimsDelta->data(p_y), 
                                                          -0.15,0.15);
  targetFrame.p.data[2] = Utils_math<double>::map( _platformJoints(p_pitch),
                                                          -_platformJointLimsDelta->data(p_pitch), _platformJointLimsDelta->data(p_pitch), 
                                                          -0.15,0.15) ;
                                                     
  Eigen::Vector3d p_;
  tf::vectorKDLToEigen(targetFrame.p, p_);  
  Quaterniond q_ = Quaterniond::FromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0), p_.normalized());
  tf::quaternionEigenToKDL(q_,targetFrame.M);
   // targetFrame.M = targetFrame.M * _worldPandaLink0Frame.M;
  
  //insertionFrame.p.Zero(); insertionFrame.p.data[2] = 0.12;
  insertionFrame.p.Zero(); insertionFrame.p.data[2] = 0.00;

  cout<<"target: "<<targetFrame.p.data[0]<<" "<<targetFrame.p.data[1]<<" "<<targetFrame.p.data[2]<<endl;
  _desiredTargetFrame.p = _worldPandaLink0Frame.M * targetFrame.p + _trocarBasePoseFrame.p;// + _trocarBasePoseFrame.M * insertionFrame.p ;
  _desiredTargetFrame.M = _worldPandaLink0Frame.M * targetFrame.M ; //_trocarBasePoseFrame.M *

}



void pandaSurgicalGripper::performInverseKinematicsWithPlatform(){

  *_pandaToolJointsPrev = *_pandaToolJoints;
  int ret = _myPosIkSolver_wrist->CartToJnt(*me->_pandaToolJointsInit,_desiredTargetFrame,*me->_pandaToolJoints);
    if (ret<0)
    {
        *_pandaToolJoints=*_pandaToolJointsPrev;
        *_pandaToolJointsInit=*_pandaToolJointsPrev;
        if (_mySolutionFound)
        {
          ROS_ERROR("No tool IK solutions for the tool found yet... move around to find one");
          _mySolutionFound=false;
        }
    }
    else{
        if (!_mySolutionFound)
        {
          *_pandaToolJointsInit = *_pandaToolJoints;
          ROS_INFO("Solutions of tool IK found again!");
          _mySolutionFound=true;
        }
    }

  _pandaToolJointsAllPrev = _pandaToolJointsAll;
  // _pandaToolJoints->data(panda_joint7) = -Utils_math<double>::map( (_platformJoints(p_yaw) - _platformJointsOffset(p_yaw)) , 
  //                                   -25*DEG_TO_RAD, 25*DEG_TO_RAD, 
  //                                   _pandaToolJointLimsAll[L_MIN]->data(panda_joint7), _pandaToolJointLimsAll[L_MAX]->data(panda_joint7)) + M_PI_2;

  _pandaToolJointsAll(panda_tool_wrist_open_angle) = _hAxisFilterGrasp->update (Utils_math<double>::map( (-(_platformJoints(p_roll) - _platformJointsOffset(p_roll))),
                                           0.0*_platformJointLimsDelta->data(p_roll), 1.0*_platformJointLimsDelta->data(p_roll), 
                                           _pandaToolJointLimsAll[L_MIN]->data(panda_tool_wrist_open_angle), _pandaToolJointLimsAll[L_MAX]->data(panda_tool_wrist_open_angle)));
  _pandaToolJointsAll(panda_tool_wrist_open_angle_mimic) = _pandaToolJointsAll(panda_tool_wrist_open_angle) ;

   //cout<<"boo_now"<<endl;
  _pandaToolJointsPosFiltered =  _hAxisFilterPos->update(_pandaToolJoints->data.segment(0,NB_PANDA_TOOL_AXIS_RED));
  _pandaToolJointsAll.segment(0, NB_PANDA_TOOL_AXIS_RED) = _pandaToolJointsPosFiltered;
  _pandaToolJointsAll.cwiseMin(_pandaToolJointLimsAll[L_MAX]->data).cwiseMax(_pandaToolJointLimsAll[L_MIN]->data);

}

void pandaSurgicalGripper::performChainForwardKinematics() 
{
  KDL::Frame frame_;
  _myFrames.clear();
    //cout<<_pandaToolJoints->data.transpose()<<endl;
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
      _myFKSolver_wrist->JntToCart(*me->_pandaToolJoints, frame_, i + 1);
      _myFrames.push_back(frame_); 
    //cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
  }
  KDL::Frame frameTip_;
  _myFrames_tip.clear();
  _pandaToolJointsFull->data=_pandaToolJointsAll;
    //cout<<_pandaToolJointsFull->data.transpose()<<endl;
  for (unsigned int j = 0; j < _mySegmentsTip.size(); j++) {
      _myFKSolver_tip->JntToCart(*me->_pandaToolJointsFull, frameTip_, j + 1);
      // cout<<frameTip_.p.data[0]<<" "<<frameTip_.p.data[1]<<" "<<frameTip_.p.data[2]<<endl;
      _myFrames_tip.push_back(frameTip_);
  }  
  // publishPandaToolTipPose();
}

/*
void pandaSurgicalGripper::readLegJoints(const sensor_msgs::JointState::ConstPtr &msg)
{

  for (unsigned int i=0; i<NB_LEG_AXIS; i++)
  {
    me->_legJoints(i) = msg->position[i];
  }

  if (!_flagLegJointsConnected)
  {
    ROS_INFO("Leg joints received by the tool");
  }
  _flagLegJointsConnected = true;

  if (!_flagLegJointLimitsOffsetCalculated) {
    _legJointsOffset = _legJoints;
    if (fabs(_legJoints(hip_adduction)) > FLT_EPSILON) {

      _legJointsOffset(hip_adduction) = 0.7 * _legJointLims[L_MIN]->data(hip_adduction);
    }

    _flagLegJointLimitsOffsetCalculated = true;
  }
}
*/
void pandaSurgicalGripper::readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    me->_platformJoints(i) = msg->position[i];
    me->_platformVelocities(i) = msg->velocity[i];
    me->_platformEfforts(i) = msg->effort[i];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("Platform joints received by the tool");
  }
  _flagPlatformJointsConnected = true;

  if (!_flagPlatformJointLimitsOffsetCalculated) {
    _platformJointsOffset(p_roll) = 15.0*DEG_TO_RAD;
    _flagPlatformJointLimitsOffsetCalculated = true;
  }
}

void pandaSurgicalGripper::readPandaToolJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  if (!_flagPantaToolJointsRetrieved)
  {
    for (unsigned int i = 0; i < NB_PANDA_TOOL_AXIS_FULL; i++) {
      me->_pandaToolJointsGet(i) = msg->position[i];
    }
    _flagPantaToolJointsRetrieved=true;
  }
  
  
  if (!_flagPantaToolJointInitRetrieved) {
    ROS_INFO("Panda tool joints received by the tool");
    for (unsigned int i = 0; i < NB_PANDA_TOOL_AXIS_RED; i++) {
      me->_pandaToolJointsInit->data[i] = msg->position[i];
    }
  }
  _flagPantaToolJointInitRetrieved = true;

}



void pandaSurgicalGripper::readTrocarBasePose()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;


  if (_pandaTool_id==Panda_Tool_Name::LEFT)
  {
    original_frame = "left_panda_link0";
    //destination_frame  = "left_panda_tool_tip_link_ee";
    destination_frame  = "left_panda_tool_wrist_link";
  }
  else 
  {
    original_frame = "right_panda_link0";
    //destination_frame  = "right_panda_tool_tip_link_ee";
    destination_frame  = "right_panda_tool_wrist_link";
  }

  geometry_msgs::TransformStamped trocarBasePoseTransform_;

  try {
    trocarBasePoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
    
    
    if (!_flagTrocarBasePoseRetrieved) {
      _trocarBasePoseFrame = tf2::transformToKDL(trocarBasePoseTransform_);
      cout<<"trocar position w.r.t. base: "<<_trocarBasePoseFrame.p.data[0]<<" "<<
                                             _trocarBasePoseFrame.p.data[1]<<" "<<
                                             _trocarBasePoseFrame.p.data[2]<<" "<<endl;
      _flagTrocarBasePoseRetrieved = true;
    }
    
    else{
      
    }  
  } 
  
  
  catch (tf2::TransformException ex) {
    if (count>2)
    { ROS_ERROR("%s", ex.what());
      count = 0;
    }
    else
    {
      count++;
    }
    ros::Duration(1.0).sleep();
  }
}

void pandaSurgicalGripper::readPandaLink0Frame()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;


  if (_pandaTool_id==Panda_Tool_Name::LEFT)
  {
    original_frame = "world";
    destination_frame  = "left_panda_link0";
  }
  else 
  {
    original_frame = "world";
    destination_frame  = "right_panda_link0";
  }

  geometry_msgs::TransformStamped pandaLink0PoseTransform_;

  try {
    pandaLink0PoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
    
    
    if (!_flagPandaLink0FrameRetrieved) {
      _worldPandaLink0Frame = tf2::transformToKDL(pandaLink0PoseTransform_);
      _worldPandaLink0Frame.M.DoRotY(-M_PI_2); // Correction for the human perspective
      _flagPandaLink0FrameRetrieved = true;
    }
    
    else{
      
    }  
  } 
  
  
  catch (tf2::TransformException ex) {
    if (count>2)
    { ROS_ERROR("%s", ex.what());
      count = 0;
    }
    else
    {
      count++;
    }
    ros::Duration(1.0).sleep();
  }
}



/*
// void pandaSurgicalGripper::publishToolTipPose(){Panda
  _msgToolTipPose.header.stamp = ros::Time::now();
  tf::poseKDLToMsg(_myFrames_tip[NB_PANDA_TOOL_AXIS_FULL-1],_msgToolTipPose.pose);
  _pubToolTipPose.publish(_msgToolTipPose);
}
*/

/*
void pandaSurgicalGripper::readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg){
    for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
    {
      _hAxisFilterPosValue(i)=1.0 - msg->sGrasp_hFilters[Axis_Mod[i]];
    }

     _hAxisFilterGraspValue = 1.0 - msg->sGrasp_hFilters[p_roll];
     //cout<<_hAxisFilterGraspValue<<endl;

    _hAxisFilterPos->setAlphas(_hAxisFilterPosValue);
    _hAxisFilterGrasp->setAlpha(_hAxisFilterGraspValue);
}
*/

/*
void pandaSurgicalGripper::publishPandaToolJointStates() {
  // _mutex.lock();

  _msgJointStates.header.stamp = ros::Time::now();

  _msgJointStates.name.resize(NB_PANDA_TOOL_AXIS_FULL);
  _msgJointStates.position.resize(NB_PANDA_TOOL_AXIS_FULL);
  _msgJointStates.velocity.resize(NB_PANDA_TOOL_AXIS_FULL);
  _msgJointStates.effort.resize(NB_PANDA_TOOL_AXIS_FULL);

  for (int k = 0; k < NB_PANDA_TOOL_AXIS_FULL; k++) {
    _msgJointStates.name[k] = std::string(Panda_Tool_Names[_pandaTool_id]) + "_" + Panda_Tool_Axis_Names[k];
    _msgJointStates.velocity[k] = _pandaToolJointsAllSpeed(k);
    _msgJointStates.effort[k] = 0.0;
    _msgJointStates.position[k] = me->_pandaToolJointsAll(k);
  }
  _pubToolJointStates.publish(_msgJointStates);
  // _mutex.unlock();
}
*/