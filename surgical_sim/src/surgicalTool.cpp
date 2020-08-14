#include "surgicalTool.h"
#include <stdio.h>
#include <stdlib.h>
#include "tf_conversions/tf_eigen.h"
#include "Utils_math.h"
#include <iostream>

#define ListofToolAxes(enumeration, names) names,
char const *Tool_Axis_Names[]{TOOL_AXES};
#undef ListofToolAxes

#define ListofLegAxes(enumeration, names) names,
char const *Leg_Axis_Names[]{LEG_AXES};
#undef ListofLegAxes

#define ListofPlatformAxes(enumeration, names) names,
char const *Platform_Axis_Names[]{PLATFORM_AXES};
#undef ListofPlatformAxes


char const *Tool_Names[]{"none", "right", "left"};
char const *Leg_Names[]{"none", "right", "left"};
char const *Platform_Names[]{"none", "right", "left"};

surgicalTool *surgicalTool::me = NULL;


surgicalTool::surgicalTool(ros::NodeHandle &n_1, double frequency,
                   surgicalTool::Tool_Name tool_id, urdf::Model model_)
    : _n(n_1), _tool_id(tool_id), _loopRate(frequency), _dt(1.0f / frequency),
      _myModel(model_) {
   me = this;
  _stop = false;

  _myInput = LEG_INPUT;
  std::string inputName_;

  if (!_n.getParam("controlInput", inputName_))
    { ROS_ERROR("No controlInput param"); }

  if (inputName_ == "platform")
  {
    ROS_INFO("Control input of the tool changed to platform");
    _myInput=PLATFORM_INPUT;
  }
  std::cout<<_myInput<<endl;
  _toolJoints = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJoints->data.setZero();
  _toolJointsInit = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointsInit->data.setZero();

  _legJointLims[L_MIN] = new KDL::JntArray(NB_LEG_AXIS);
  _legJointLims[L_MAX] = new KDL::JntArray(NB_LEG_AXIS);

  _platformJointLims[L_MIN] = new KDL::JntArray(NB_PLATFORM_AXIS);
  _platformJointLims[L_MAX] = new KDL::JntArray(NB_PLATFORM_AXIS);

  _toolJointLims[L_MIN] = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointLims[L_MAX] = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointLimsAll[L_MIN] = new KDL::JntArray(NB_TOOL_AXIS_FULL);
  _toolJointLimsAll[L_MAX] = new KDL::JntArray(NB_TOOL_AXIS_FULL);

  _weightedJointSpaceMassMatrix.setIdentity();
  _weightedTaskSpaceMassMatrix.setIdentity();
  _myJacobian.resize(NB_TOOL_AXIS_RED);

  _footTipPosFrame.Identity();
  _footTipPosFrameInit.M.Identity();
  _footTipPosFrameInit.p.Zero();
  _footTipPosition.setZero();
  _footTipQuaternion.setIdentity();
  _footTipRotationMatrix.setIdentity();

  _flagFootTipPoseConnected =  false;
  _flagLegJointsConnected = false;
  _flagPlatformJointsConnected = false;

  _flagToolJointLimitsOffsetCalculated = false;
  _flagLegJointLimitsOffsetCalculated = false;
  _flagPlatformJointLimitsOffsetCalculated = false;

  _toolJointsAll.setZero();
  _toolJointsAllOffset.setZero();
 
  _legJoints.setZero();
  _legJointsOffset.setZero();

  _platformJoints.setZero();
  _platformJointsOffset.setZero();

  if (!_legModel.initParam("leg_description")) {
      ROS_ERROR("Failed to parse urdf of the leg");
    }

  if (!_platformModel.initParam("platform_description")) {
      ROS_ERROR("Failed to parse urdf file");
  }

    if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
      ROS_ERROR("Failed to construct kdl tree");
      _stop = true;
  }

  KDL::Vector grav_vector(0.0, 0.0, (double)GRAVITY);

  _myTree.getChain("tool_base_link", "tool_tip_link_ee", _myToolBaseToTipChain);

  _myFKSolver = new KDL::ChainFkSolverPos_recursive(_myToolBaseToTipChain);

  _myJacSolver = new KDL::ChainJntToJacSolver(_myToolBaseToTipChain);

  _mySegments = _myToolBaseToTipChain.segments;


  for (unsigned int i = 0; i<NB_LEG_AXIS; i++)
  {
    _legJointLims[L_MIN]->data(i) = _legModel.getJoint(Leg_Axis_Names[i])->limits->lower;
    _legJointLims[L_MAX]->data(i) = _legModel.getJoint(Leg_Axis_Names[i])->limits->upper;
  }

   for (unsigned int i = 0; i<NB_PLATFORM_AXIS; i++)
  {
    _platformJointLims[L_MIN]->data(i) = _platformModel.getJoint(Platform_Axis_Names[i])->limits->lower;
    _platformJointLims[L_MAX]->data(i) = _platformModel.getJoint(Platform_Axis_Names[i])->limits->upper;
  }

  for (unsigned int joint_=0; joint_ < NB_TOOL_AXIS_FULL; joint_++ )
  {
    _toolJointLimsAll[L_MIN]->data(joint_) = _myModel.getJoint(Tool_Axis_Names[joint_])->limits->lower;
    _toolJointLimsAll[L_MAX]->data(joint_) = _myModel.getJoint(Tool_Axis_Names[joint_])->limits->upper;
      
    if (joint_<NB_TOOL_AXIS_RED)
    {
      
      _toolJointLims[L_MIN]->data(joint_) = _toolJointLimsAll[L_MIN]->data(joint_) ;
      _toolJointLims[L_MAX]->data(joint_) = _toolJointLimsAll[L_MAX]->data(joint_) ;
    }
  }


  /****************************** Inverse Kinematics of the tool ***********************************/
      _myVelIKSolver = new KDL::ChainIkSolverVel_wdls(_myToolBaseToTipChain);

      _myPosIkSolver = new KDL::ChainIkSolverPos_NR_JL(_myToolBaseToTipChain,*me->_toolJointLims[L_MIN], *me->_toolJointLims[L_MAX], *_myFKSolver,*_myVelIKSolver);

      //_weightedTaskSpaceMassMatrix.block(3,3,3,3).diagonal()<< 1e-7, 1e-7, 1e-7;
      //_weightedJointSpaceMassMatrix.block(tool_insertion, tool_insertion, 2, 2).diagonal() << 1e-15, 1e-15;

      //_myVelIKSolver->setWeightTS(_weightedTaskSpaceMassMatrix);
      //_myVelIKSolver->setWeightJS(_weightedJointSpaceMassMatrix);
  /*                                                                                                */


  _tfListener = new tf2_ros::TransformListener(_tfBuffer);
}

surgicalTool::~surgicalTool() { me->_n.shutdown(); }

bool surgicalTool::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{ 

  _pubToolJointStates = _n.advertise<sensor_msgs::JointState>("joint_states", 1);
 
  _subLegJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_tool_id])+"/leg_joint_publisher/joint_states"
      , 1, boost::bind(&surgicalTool::readLegJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_tool_id])+"/platform_joint_publisher/joint_states"
      , 1, boost::bind(&surgicalTool::readPlatformJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      

  // Subscriber definitions
  signal(SIGINT, surgicalTool::stopNode);

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

void surgicalTool::stopNode(int sig) { me->_stop = true; }

void surgicalTool::run() {
  
  while (!_stop) {
    readFootTipBasePose();
    if (_flagFootTipPoseConnected && _flagLegJointsConnected) {
     
     switch (_myInput)
     {
        case PLATFORM_INPUT:
        {
          computeWithPlatformJoints4DoF();
          break;
        }
        default:
        {
          computeWithLegJoints4DoF();
        }
       break;
     }
     
      // performInverseKinematics();
      publishToolJointStates();
      performChainForwardKinematics();
      // computeToolManipulability();
      // publishManipulabilityEllipsoid();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("Tool state variables stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void surgicalTool::publishToolJointStates() {
  // _mutex.lock();

  _msgJointStates.header.stamp = ros::Time::now();

  _msgJointStates.name.resize(NB_TOOL_AXIS_FULL);
  _msgJointStates.position.resize(NB_TOOL_AXIS_FULL);
  _msgJointStates.velocity.resize(NB_TOOL_AXIS_FULL);
  _msgJointStates.effort.resize(NB_TOOL_AXIS_FULL);

  for (int k = 0; k < NB_TOOL_AXIS_FULL; k++) {
    _msgJointStates.name[k] = Tool_Axis_Names[k];
    _msgJointStates.velocity[k] = 0.0f;
    _msgJointStates.effort[k] = 0.0f;
    _msgJointStates.position[k] = me->_toolJointsAll(k);
  }
  _pubToolJointStates.publish(_msgJointStates);
  // _mutex.unlock();
}



void surgicalTool::readFootTipBasePose()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;

  if (_tool_id==Tool_Name::LEFT)
  {
    destination_frame = "left/pedal_tip";
    original_frame = "left/platform_base_link";
  }
  else 
  {
    destination_frame = "right/pedal_tip";
    original_frame = "right/platform_base_link";
  }

  geometry_msgs::TransformStamped footPoseTransform_;

  try {
    footPoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
    
    
    if (!_flagFootTipPoseConnected) {
      _footTipPosFrameInit = tf2::transformToKDL(footPoseTransform_);
    }
    
    else{
      KDL::Frame footPosFrame_temp = tf2::transformToKDL(footPoseTransform_);
      _footTipPosFrame.p = (footPosFrame_temp.p - _footTipPosFrameInit.p);
      
      tf::vectorKDLToEigen(_footTipPosFrame.p,_footTipPosition);
      // cout<<_footTipPosition.transpose()<<endl;
      if (_footTipPosition.norm() > FLT_EPSILON)
      {
        _footTipRotationMatrix = Utils_math<double>::rodriguesRotation(
            Eigen::Vector3d(0.0, 0.0, 1.0), _footTipPosition);
      }
      else
      {
        _footTipRotationMatrix.setIdentity();
      }
      //cout<<_footTipRotationMatrix<<endl;
      _footTipQuaternion = Eigen::Quaternion <double> (_footTipRotationMatrix);
      tf::quaternionEigenToKDL(_footTipQuaternion, _footTipPosFrame.M);
    }
    _flagFootTipPoseConnected = true;
    
  } catch (tf2::TransformException ex) {
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



void surgicalTool::performInverseKinematics(){
  int ret = _myPosIkSolver->CartToJnt(*me->_toolJointsInit,_footTipPosFrame,*me->_toolJoints);
  *_toolJointsInit = *_toolJoints;
  me->_toolJointsAll.segment(0, tool_wrist_yaw) = _toolJoints->data;
  if (ret<0)
  {
    if (_mySolutionFound) 
    {
      ROS_ERROR("No tool IK solutions for the tool found yet... move around to find one");
      _mySolutionFound=false;
    }
  }
  else{
    if(!_mySolutionFound)
    {
      ROS_INFO("Solutions of tool IK found again!");
      _mySolutionFound=true;
    }
  }
}


void surgicalTool::computeWithLegJoints7DoF()
{

  
  _toolJointsAll(tool_yaw) = -Utils_math<double>::map( (_legJoints(hip_roll) - _legJointsOffset(hip_roll)),
                                      _legJointLims[L_MIN]->data(hip_roll), _legJointLims[L_MAX]->data(hip_roll), 
                                      _toolJointLimsAll[L_MIN]->data(tool_yaw), _toolJointLimsAll[L_MAX]->data(tool_yaw));
  
  _toolJointsAll(tool_pitch) = Utils_math<double>::map( (_legJoints(knee_extension) - _legJointsOffset(knee_extension)),
                                      _legJointLims[L_MIN]->data(knee_extension), _legJointLims[L_MAX]->data(knee_extension), 
                                      _toolJointLimsAll[L_MIN]->data(tool_pitch), _toolJointLimsAll[L_MAX]->data(tool_pitch));
  
  _toolJointsAll(tool_roll) =  - Utils_math<double>::map( (_legJoints(ankle_yaw) - _legJointsOffset(ankle_yaw)) , 
                                _legJointLims[L_MIN]->data(ankle_yaw), _legJointLims[L_MAX]->data(ankle_yaw), 
                                _toolJointLimsAll[L_MIN]->data(tool_roll), _toolJointLimsAll[L_MAX]->data(tool_roll));
  
  _toolJointsAll(tool_wrist_pitch) =  Utils_math<double>::map( (_legJoints(ankle_pitch) - _legJointsOffset(ankle_pitch)),
                                      _legJointLims[L_MIN]->data(ankle_pitch), _legJointLims[L_MAX]->data(ankle_pitch), 
                                      _toolJointLimsAll[L_MIN]->data(tool_wrist_pitch), _toolJointLimsAll[L_MAX]->data(tool_wrist_pitch));
  
  
  _toolJointsAll(tool_wrist_yaw) =  Utils_math<double>::map( (_legJoints(ankle_roll) - _legJointsOffset(ankle_roll)),
                                    _legJointLims[L_MIN]->data(ankle_roll), _legJointLims[L_MAX]->data(ankle_roll), 
                                    _toolJointLimsAll[L_MIN]->data(tool_wrist_yaw), _toolJointLimsAll[L_MAX]->data(tool_wrist_yaw));
  
  _toolJointsAll(tool_insertion) = Utils_math<double>::map( (_legJoints(hip_adduction) - _legJointsOffset(hip_adduction)),
                                    _legJointLims[L_MIN]->data(hip_adduction), _legJointLims[L_MAX]->data(hip_adduction), 
                                    _toolJointLimsAll[L_MIN]->data(tool_insertion), _toolJointLimsAll[L_MAX]->data(tool_insertion));


  // _toolJointsAll(tool_wrist_open_angle) = Utils_math<double>::bound((M_PI_2/0.1) * _footAnklePosFrame.p.x(), 0.0, M_PI_2) ;
  // _toolJointsAll(tool_wrist_open_angle_mimic) = _toolJointsAll(tool_wrist_open_angle) ;

  if (!_flagToolJointLimitsOffsetCalculated)
  {
    _toolJointsAllOffset=_toolJointsAll;
    _toolJointsAllOffset(tool_insertion)-=0.05;
    _flagToolJointLimitsOffsetCalculated=true;
  }
  else
  {
    for (int i = 0 ; i<NB_TOOL_AXIS_FULL; i++)
    {
      _toolJointsAll(i)  = Utils_math<double>::bound(_toolJointsAll(i) - _toolJointsAllOffset(i),
                                              _toolJointLimsAll[L_MIN]->data(i), _toolJointLimsAll[L_MAX]->data(i));
    }
  }
  
}

void surgicalTool::computeWithLegJoints4DoF()
{

  
  _toolJointsAll(tool_yaw) = -Utils_math<double>::map( (_legJoints(hip_roll) - _legJointsOffset(hip_roll)),
                                      _legJointLims[L_MIN]->data(hip_roll), _legJointLims[L_MAX]->data(hip_roll), 
                                      _toolJointLimsAll[L_MIN]->data(tool_yaw), _toolJointLimsAll[L_MAX]->data(tool_yaw));
  
  _toolJointsAll(tool_pitch) = Utils_math<double>::map( (_legJoints(knee_extension) - _legJointsOffset(knee_extension)),
                                      _legJointLims[L_MIN]->data(knee_extension), _legJointLims[L_MAX]->data(knee_extension), 
                                      _toolJointLimsAll[L_MIN]->data(tool_pitch), _toolJointLimsAll[L_MAX]->data(tool_pitch));
  
  _toolJointsAll(tool_roll) =  - 0.25 *  Utils_math<double>::map( (_legJoints(ankle_yaw) - _legJointsOffset(ankle_yaw)) , 
                                _legJointLims[L_MIN]->data(ankle_yaw), _legJointLims[L_MAX]->data(ankle_yaw), 
                                _toolJointLimsAll[L_MIN]->data(tool_roll), _toolJointLimsAll[L_MAX]->data(tool_roll))
                               - 0.75 * Utils_math<double>::map( (_legJoints(ankle_roll) - _legJointsOffset(ankle_roll)) , 
                                _legJointLims[L_MIN]->data(ankle_roll), _legJointLims[L_MAX]->data(ankle_roll), 
                                _toolJointLimsAll[L_MIN]->data(tool_roll), _toolJointLimsAll[L_MAX]->data(tool_roll))
                                ;

  
  _toolJointsAll(tool_insertion) = - Utils_math<double>::map( (_legJoints(ankle_pitch) - _legJointsOffset(ankle_pitch)),
                                      _legJointLims[L_MIN]->data(ankle_pitch), _legJointLims[L_MAX]->data(ankle_pitch), 
                                      _toolJointLimsAll[L_MIN]->data(tool_insertion), _toolJointLimsAll[L_MAX]->data(tool_insertion));
  
  _toolJointsAll(tool_wrist_open_angle) = - Utils_math<double>::map( (_legJoints(hip_adduction) - _legJointsOffset(hip_adduction)),
                                          _legJointLims[L_MIN]->data(hip_adduction), _legJointLims[L_MAX]->data(hip_adduction), 
                                          _toolJointLimsAll[L_MIN]->data(tool_wrist_open_angle), _toolJointLimsAll[L_MAX]->data(tool_wrist_open_angle));
  
  _toolJointsAll(tool_wrist_open_angle_mimic) = _toolJointsAll(tool_wrist_open_angle) ;

  if (!_flagToolJointLimitsOffsetCalculated && _flagLegJointLimitsOffsetCalculated)
  {
    _toolJointsAllOffset=_toolJointsAll;
    _toolJointsAllOffset(tool_insertion)-=0.05;
    // if (fabs(_legJointsOffset(hip_adduction))> FLT_EPSILON )
    // {
    //     _toolJointsAllOffset(tool_wrist_open_angle)-= M_PI_4;
    //     _toolJointsAllOffset(tool_wrist_open_angle_mimic) = _toolJointsAllOffset(tool_wrist_open_angle);
    //     cout << _legJointsOffset(hip_adduction) << endl;
    // }
    _flagToolJointLimitsOffsetCalculated=true;
  }
  else
  {
    for (int i = 0 ; i<NB_TOOL_AXIS_FULL; i++)
    {
      _toolJointsAll(i)  = Utils_math<double>::bound(_toolJointsAll(i) - _toolJointsAllOffset(i),
                                              _toolJointLimsAll[L_MIN]->data(i), _toolJointLimsAll[L_MAX]->data(i));
    }
  }
  
}


void surgicalTool::computeWithPlatformJoints4DoF()
{

  
  _toolJointsAll(tool_yaw) = Utils_math<double>::map( (_platformJoints(p_x) - _platformJointsOffset(p_x)),
                                      _platformJointLims[L_MIN]->data(p_x), _platformJointLims[L_MAX]->data(p_x), 
                                      _toolJointLimsAll[L_MIN]->data(tool_yaw), _toolJointLimsAll[L_MAX]->data(tool_yaw));
  
  _toolJointsAll(tool_pitch) = - Utils_math<double>::map( (_platformJoints(p_y) - _platformJointsOffset(p_y)),
                                      _platformJointLims[L_MIN]->data(p_y), _platformJointLims[L_MAX]->data(p_y), 
                                      _toolJointLimsAll[L_MIN]->data(tool_pitch), _toolJointLimsAll[L_MAX]->data(tool_pitch));
  
  _toolJointsAll(tool_roll) =  - 0.0 *  Utils_math<double>::map( (_platformJoints(p_roll) - _platformJointsOffset(p_roll)) , 
                                _platformJointLims[L_MIN]->data(p_roll), _platformJointLims[L_MAX]->data(p_roll), 
                                _toolJointLimsAll[L_MIN]->data(tool_roll), _toolJointLimsAll[L_MAX]->data(tool_roll))
                               - 1.0 * Utils_math<double>::map( 2*(_platformJoints(p_yaw) - _platformJointsOffset(p_yaw)) , 
                                _platformJointLims[L_MIN]->data(p_yaw), _platformJointLims[L_MAX]->data(p_yaw), 
                                _toolJointLimsAll[L_MIN]->data(tool_roll), _toolJointLimsAll[L_MAX]->data(tool_roll))
                                ;

  
  _toolJointsAll(tool_insertion) = - Utils_math<double>::map( (_platformJoints(p_pitch) - _platformJointsOffset(p_pitch)),
                                      _platformJointLims[L_MIN]->data(p_pitch), _platformJointLims[L_MAX]->data(p_pitch), 
                                      _toolJointLimsAll[L_MIN]->data(tool_insertion), _toolJointLimsAll[L_MAX]->data(tool_insertion));
  
  _toolJointsAll(tool_wrist_open_angle) = - Utils_math<double>::map( (_platformJoints(p_roll) - _platformJointsOffset(p_roll)),
                                          _platformJointLims[L_MIN]->data(p_roll), _platformJointLims[L_MAX]->data(p_roll), 
                                          _toolJointLimsAll[L_MIN]->data(tool_wrist_open_angle), _toolJointLimsAll[L_MAX]->data(tool_wrist_open_angle));
  
  _toolJointsAll(tool_wrist_open_angle_mimic) = _toolJointsAll(tool_wrist_open_angle) ;

  if (!_flagToolJointLimitsOffsetCalculated && _flagLegJointLimitsOffsetCalculated)
  {
    _toolJointsAllOffset=_toolJointsAll;
    _toolJointsAllOffset(tool_insertion)-=0.05;
    _flagToolJointLimitsOffsetCalculated=true;
  }
  else
  {
    for (int i = 0 ; i<NB_TOOL_AXIS_FULL; i++)
    {
      _toolJointsAll(i)  = Utils_math<double>::bound(_toolJointsAll(i) - _toolJointsAllOffset(i),
                                              _toolJointLimsAll[L_MIN]->data(i), _toolJointLimsAll[L_MAX]->data(i));
    }
  }
  
}


void surgicalTool::performChainForwardKinematics() 
{
  KDL::Frame frame_;
  _myFrames.clear();
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myFKSolver->JntToCart(*me->_toolJoints, frame_, i + 1);
    _myFrames.push_back(frame_);
  }
}


void surgicalTool::readLegJoints(const sensor_msgs::JointState::ConstPtr &msg)
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

void surgicalTool::readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    me->_platformJoints(i) = msg->position[i];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("Platform joints received by the tool");
  }
  _flagPlatformJointsConnected = true;

  if (!_flagPlatformJointLimitsOffsetCalculated) {
    _platformJointsOffset = _platformJoints;
    _flagPlatformJointLimitsOffsetCalculated = true;
  }
}





















// void surgicalTool::publishManipulabilityEllipsoid() {
// _mutex.lock();
//   VectorXd svdValues = _mySVD.singularValues();
//   Matrix<double, 3,3> svdVectors = _mySVD.matrixU().block(0,0,3,3);
//   Quaternion<double> svdSingQuaternion(svdVectors);
//   std::string frame_name;
//   std::string ns_name;
//   frame_name = _tool_id == RIGHT ? "/right/hip_base_link" :
//   "/left/hip_base_link";
//   ns_name = _tool_id == RIGHT ? "/right" : "/left";
//   _msgManipEllipsoid.header.frame_id = frame_name;
//   _msgManipEllipsoid.header.stamp = ros::Time::now();
//   _msgManipEllipsoid.ns = ns_name;
//   _msgManipEllipsoid.id = 0;
//   _msgManipEllipsoid.type = visualization_msgs::Marker::SPHERE;
//   _msgManipEllipsoid.action = visualization_msgs::Marker::ADD;
//   _msgManipEllipsoid.pose.position.x = _footTipPosFrame.p.x();
//   _msgManipEllipsoid.pose.position.y = _footTipPosFrame.p.y();
//   _msgManipEllipsoid.pose.position.z = _footTipPosFrame.p.z();
//   _msgManipEllipsoid.pose.orientation.x = svdSingQuaternion.x();
//   _msgManipEllipsoid.pose.orientation.y = svdSingQuaternion.y();
//   _msgManipEllipsoid.pose.orientation.z = svdSingQuaternion.z();
//   _msgManipEllipsoid.pose.orientation.w = svdSingQuaternion.w();
//   _msgManipEllipsoid.scale.x = svdValues(0);
//   _msgManipEllipsoid.scale.y = svdValues(1);
//   _msgManipEllipsoid.scale.z = svdValues(2);
//   _msgManipEllipsoid.color.a = 0.5; // Don't forget to set the alpha!
//   _msgManipEllipsoid.color.r = svdValues(3);
//   _msgManipEllipsoid.color.g = svdValues(4);
//   _msgManipEllipsoid.color.b = svdValues(5);
//   // only if using a MESH_RESOURCE marker type:
//   // marker.mesh_resource =
//   "package://pr2_description/meshes/base_v0/base.dae";
//   _pubManipEllipsoid.publish(_msgManipEllipsoid);
//   // _mutex.unlock();
// }

// void surgicalTool::computeToolManipulability()
// {
//   //_myTorqueFDSolver->U;
//   _myJacSolver->JntToJac(*_toolJoints,_myJacobian);
//   Matrix<double, NB_AXIS_WRENCH, NB_TOOL_AXIS_RED> weightedJacobian =
//   _myJacobian.data*_weightedJointSpaceMassMatrix.inverse();
//   //_mySVD.compute(_myJacobian.data*_weightedJointSpaceMassMatrix.inverse(),ComputeThinU
//   | ComputeThinV);
//   _mySVD.compute(weightedJacobian.normalized(), ComputeThinU | ComputeThinV);
//   //cout<<_mySVD.matrixU()<<endl;
//   //cout<<_mySVD.singularValues().transpose()<<endl;
//   //cout <<
//   _mySVD.singularValues().minCoeff()/_mySVD.singularValues().maxCoeff() <<
//   endl;
// }
