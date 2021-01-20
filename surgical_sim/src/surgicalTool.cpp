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



#define MIN_THRESHOLD 5.0
#define MAX_THRESHOLD 15.0
#define STD 0.7

const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};

const float LimitsX[2][2] = {{-0.15,0.05},{-0.05,0.15}};



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

  if (inputName_.compare("platform") == 0)
  {
    ROS_INFO("Control input of the tool changed to platform");
    _myInput=PLATFORM_INPUT;
  }

  std::string toolTypeStr_;

  if (!_n.getParam("toolType", toolTypeStr_))
    { ROS_ERROR("No toolType param"); }

  if (toolTypeStr_.compare("camera") == 0)
  {
    ROS_INFO("The tool you are controlling is a camera");
    _tool_type=CAMERA;
  } else if (toolTypeStr_.compare("forceps") == 0){
    ROS_INFO("The tool you are controlling is a forceps");
    _tool_type=FORCEPS;
  } else
  {
    _tool_type=FORCEPS; // default
  }

  _nDOF=NB_TOOL_AXIS_FULL;
    if (_tool_type==FORCEPS)
  {
    _nDOF=NB_TOOL_AXIS_FULL;
  } else {
    _nDOF=NB_TOOL_AXIS_RED;
  }
  _msgJointCommands.data.resize(_nDOF);
  //std::cout<<_myInput<<endl;
  _toolJoints = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJoints->data.setZero();
  _toolJointsFull = new KDL::JntArray(_nDOF);
  _toolJointsFull->data.setZero();
  _toolJointsInit = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointsInit->data.setZero();

  _legJointLims[L_MIN] = new KDL::JntArray(NB_LEG_AXIS);
  _legJointLims[L_MIN]->data.setZero();
  _legJointLims[L_MAX] = new KDL::JntArray(NB_LEG_AXIS);
  _legJointLims[L_MAX]->data.setZero();

  _platformJointLims[L_MIN] = new KDL::JntArray(NB_PLATFORM_AXIS);
  _platformJointLims[L_MIN]->data.setZero();
  _platformJointLims[L_MAX] = new KDL::JntArray(NB_PLATFORM_AXIS);
  _platformJointLims[L_MAX]->data.setZero();
  _platformJointLimsDelta = new KDL::JntArray(NB_PLATFORM_AXIS);

  _toolJointLims[L_MIN] = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointLims[L_MAX] = new KDL::JntArray(NB_TOOL_AXIS_RED);
  _toolJointLimsAll[L_MIN] = new KDL::JntArray(_nDOF);
  _toolJointLimsAll[L_MAX] = new KDL::JntArray(_nDOF);

  _toolJointLims[L_MIN]->data.setZero();
  _toolJointLims[L_MAX]->data.setZero();
  _toolJointLimsAll[L_MIN]->data.setZero();
  _toolJointLimsAll[L_MAX]->data.setZero();

  _weightedJointSpaceMassMatrix.setIdentity();
  _weightedTaskSpaceMassMatrix.setIdentity();
  _myJacobian.resize(NB_TOOL_AXIS_RED);

  _desiredTargetFrame.M.Identity();
  _desiredTargetFrame.p.Zero();
  


  _footTipPosFrame.Identity();
  _footTipPosFrameInit.M.Identity();
  _footTipPosFrameInit.p.Zero();
  _footTipPosition.setZero();
  _footTipQuaternion.setIdentity();
  _footTipRotationMatrix.setIdentity();

  _flagSharedGrasp=false;
  _flagFootTipPoseConnected =  false;
  _flagLegJointsConnected = false;
  _flagPlatformJointsConnected = false;
  _flagRosControl=false;
  _flagCurrentToolJointsRead=false;

  _mySolutionFound = true;

  _flagToolJointLimitsOffsetCalculated = false;
  _flagLegJointLimitsOffsetCalculated = false;
  _flagPlatformJointLimitsOffsetCalculated = false;
  
  _toolJointsAll.resize(_nDOF);
  _toolJointsAllPrev.resize(_nDOF);
  _toolJointsAllSpeed.resize(_nDOF);
  _toolJointsAllOffset.resize(_nDOF);

  _toolJointsAll.setZero();
  _toolJointsAllPrev.setZero();
  _toolJointsAllSpeed.setZero();
  _toolJointsAllOffset.setZero();


  _toolJointsPosFiltered.setZero();

  _thresholds.setZero();
  
  
   _hAxisFilterPosValue.setZero();
   _hAxisFilterGraspValue=0.0;

   _hAxisFilterPos = new MatLP_Filterd(_hAxisFilterPosValue);
   _hAxisFilterPos->setAlphas(_hAxisFilterPosValue);

   _hAxisFilterGrasp = new LP_Filterd(_hAxisFilterGraspValue);
   _hAxisFilterGrasp->setAlpha(_hAxisFilterGraspValue);

  _legJoints.setZero();
  _legJointsOffset.setZero();

  _platformJoints.setZero();
  _platformJointsOffset.setZero();
  _platformVelocities.setZero();
  _platformEfforts.setZero();


  if (!_n.getParam("sharedGraspOn", _flagSharedGrasp))
  { 
      ROS_ERROR("No sharedGrasp param"); 
  }

  if (_flagSharedGrasp) cout<<"flag shared grasp enabled"<<endl;

  if (!_n.getParam("rosControl", _flagRosControl))
  { 
      ROS_ERROR("No rosControl param"); 
  }

  if (_flagSharedGrasp) cout<<"flag ros control for the tool enabled"<<endl;

  if (_myInput==LEG_INPUT)
  {
    if (!_legModel.initParam("/"+std::string(Tool_Names[_tool_id]) + "_leg/robot_description")) {
        ROS_ERROR("Failed to parse urdf of the leg");
    }
    else
    {
      for (unsigned int i = 0; i<NB_LEG_AXIS; i++)
      {
        _legJointLims[L_MIN]->data(i) = _legModel.getJoint("/"+std::string(Tool_Names[_tool_id]) + "_" + Leg_Axis_Names[i])->limits->lower;
        _legJointLims[L_MAX]->data(i) = _legModel.getJoint("/"+std::string(Tool_Names[_tool_id]) + "_" + Leg_Axis_Names[i])->limits->upper;
      }
    }
    


  }
  else
  {
    if (!_platformModel.initParam("/"+std::string(Tool_Names[_tool_id]) + "_platform/robot_description")) {
        ROS_ERROR("Failed to parse urdf file of plaform");
        _stop=true;
    }
    else
    {
      for (unsigned int i = 0; i<NB_PLATFORM_AXIS; i++)
      {
        _platformJointLims[L_MIN]->data(i) = _platformModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->lower;
        //cout<<_platformJointLims[L_MIN]->data(i)<<endl;
        _platformJointLims[L_MAX]->data(i) = _platformModel.getJoint( std::string(Tool_Names[_tool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->upper;
        //cout<<_platformJointLims[L_MAX]->data(i)<<endl;
        _platformJointLimsDelta->data(i) = min(abs(_platformJointLims[L_MIN]->data(i)),abs(_platformJointLims[L_MAX]->data(i)));
      }
    }
    
    
  _tool_selfRotation = R_SPEED;
  std::string toolSelfRotationControlStr;
  if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"toolSelfRotationControl", toolSelfRotationControlStr))
  { 
      ROS_ERROR("No toolSelfRotationControl (position,speed) param"); 
  }
  if( toolSelfRotationControlStr.compare("position")==0)
  {
    _tool_selfRotation = R_POSITION;
  }
  
  _speedControlGainCamera = 0.5;
  
  
  if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"speedControlGainCamera", _speedControlGainCamera))
  { 
      ROS_ERROR("No speedControlGainCamera  param"); 
  }
  _speedControlGainSelfRotation=50.0; 
  if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"speedControlGainSelfRotation", _speedControlGainSelfRotation))
  { 
      ROS_ERROR("No speedControlGainSelfRotation  param"); 
  }
  


  _deadZoneValues = 0.2*_platformJointLimsDelta->data.segment(0,NB_PLATFORM_AXIS);
  _deadZoneValues(p_yaw) =2.5*DEG_TO_RAD;
  //cout<<_deadZoneValues.transpose()<<endl;

  std::vector<double> deadZoneValuesPlatform;
  deadZoneValuesPlatform.resize(NB_PLATFORM_AXIS);
  if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"deadZoneValuesPlatform", deadZoneValuesPlatform))
  { 
      ROS_ERROR("No deadZoneValuesPlatform  param"); 
  }

  new (&_deadZoneValues) Map<Matrix<double,NB_PLATFORM_AXIS,1>>(deadZoneValuesPlatform.data());
    
  }

    if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
      ROS_ERROR("Failed to construct kdl tree");
      _stop = true;

  }

  KDL::Vector grav_vector(0.0, 0.0, GRAVITY);

  _myTree.getChain(std::string(Tool_Names[_tool_id]) + "_tool_base_link", std::string(Tool_Names[_tool_id]) + "_tool_wrist_link", _myToolBaseToWristChain);
  _myTree.getChain(std::string(Tool_Names[_tool_id]) + "_tool_base_link", std::string(Tool_Names[_tool_id]) + "_tool_tip_link_ee", _myToolBaseToTipChain);


  _myFKSolver_wrist = new KDL::ChainFkSolverPos_recursive(_myToolBaseToWristChain);
  _myFKSolver_tip = new KDL::ChainFkSolverPos_recursive(_myToolBaseToTipChain);

  _myJacSolver_wrist = new KDL::ChainJntToJacSolver(_myToolBaseToWristChain);

  _mySegments = _myToolBaseToWristChain.segments;
  _mySegmentsTip = _myToolBaseToTipChain.segments;


  // double test = Utils_math<double>::map(2.5,-5,5,-1,1);
  // cout<<test<<endl;

  for (unsigned int joint_=0; joint_ < _nDOF; joint_++ )
  {
    _toolJointLimsAll[L_MIN]->data(joint_) = _myModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + Tool_Axis_Names[joint_])->limits->lower;
    _toolJointLimsAll[L_MAX]->data(joint_) = _myModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + Tool_Axis_Names[joint_])->limits->upper;
      
    if (joint_<NB_TOOL_AXIS_RED)
    {
      _toolJointLims[L_MIN]->data(joint_) = _toolJointLimsAll[L_MIN]->data(joint_) ;
      _toolJointLims[L_MAX]->data(joint_) = _toolJointLimsAll[L_MAX]->data(joint_) ;
    }
  }


  /****************************** Inverse Kinematics of the tool ***********************************/
      _myVelIKSolver_wrist = new KDL::ChainIkSolverVel_wdls(_myToolBaseToWristChain);
      _myPosIkSolver_wrist= new KDL::ChainIkSolverPos_NR_JL(_myToolBaseToWristChain,*me->_toolJointLims[L_MIN], *me->_toolJointLims[L_MAX], *_myFKSolver_wrist,*_myVelIKSolver_wrist);
      
      
      _weightedTaskSpaceMassMatrix.block(3,3,3,3).diagonal()<< FLT_EPSILON, FLT_EPSILON, FLT_EPSILON;
      //_weightedJointSpaceMassMatrix.block(tool_joint4_insertion, tool_joint4_insertion, 2, 2).diagonal() << 1e-15, 1e-15;
      
      _myVelIKSolver_wrist->setWeightTS(_weightedTaskSpaceMassMatrix);
     //_myVelIKSolver->setWeightJS(_weightedJointSpaceMassMatrix);
  /*                                                                                                */

  _tfListener = new tf2_ros::TransformListener(_tfBuffer);
}

surgicalTool::~surgicalTool() { me->_n.shutdown(); }

bool surgicalTool::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{ 
  if (_flagRosControl)
    {
      _pubToolJointCommands = _n.advertise<std_msgs::Float64MultiArray>("/"+std::string(Tool_Names[_tool_id])+"_tool/joint_position_controller/command", 1);
      _subCurrentToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_tool_id])+"_tool/joint_states"
      , 1, boost::bind(&surgicalTool::readCurrentToolJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  else
  {
    _pubToolJointStates = _n.advertise<sensor_msgs::JointState>("/"+std::string(Tool_Names[_tool_id])+"_tool/joint_states", 1);
  }
  
  _pubToolTipPose = _n.advertise<geometry_msgs::PoseStamped>("tool_tip_pose", 1);
 
  _subLegJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_tool_id])+"_leg/leg_joint_publisher/joint_states"
      , 1, boost::bind(&surgicalTool::readLegJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_tool_id])+"_platform/platform_joint_publisher/joint_states"
      , 1, boost::bind(&surgicalTool::readPlatformJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      

  _subSharedGrasp = _n.subscribe<custom_msgs_gripper::SharedGraspingMsg>( "/"+std::string(Platform_Names[_tool_id])+"_tool/sharedGrasping"
      , 1, boost::bind(&surgicalTool::readSharedGrasp, this, _1),
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
  static int count = 0;
  while (!_stop) {
    
    if ((_subPlatformJointStates.getNumPublishers()==0 && _myInput==PLATFORM_INPUT) || 
        (_subLegJointStates.getNumPublishers()==0 && _myInput==LEG_INPUT))
        {
          if (count>5)
          {
            ROS_ERROR_ONCE("No control input for the tool connected");
          }
        }
    else {
        performChainForwardKinematics();
        //readFootTipBasePose();
        switch (_myInput)
        {
            case PLATFORM_INPUT:
            {
        //       //computeWithPlatformTask4DoF();
        //       //computeWithPlatformJoints4DoF();
                if (_flagCurrentToolJointsRead)
                {
                  calculateDesiredFrame();
                  performInverseKinematics();
                }
                
            break; 
            }
            default:
            {
        //       computeWithLegJoints4DoF();
            }
          break;
        }  
          if (_flagRosControl) 
          {
            publishToolJointCommands();
          } else { publishToolJointStates();
          }
          // ROS_ERROR("BOO1");
          _toolJointsAllSpeed = (_toolJointsAll - _toolJointsAllPrev) * (1.0/_dt);
          _toolJointsAllPrev = _toolJointsAll;
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

  _msgJointStates.name.resize(_nDOF);
  _msgJointStates.position.resize(_nDOF);
  _msgJointStates.velocity.resize(_nDOF);
  _msgJointStates.effort.resize(_nDOF);

  for (int k = 0; k < _nDOF; k++) {
    _msgJointStates.name[k] = std::string(Tool_Names[_tool_id]) + "_" + Tool_Axis_Names[k];
    _msgJointStates.velocity[k] = _toolJointsAllSpeed(k);
    _msgJointStates.effort[k] = 0.0;
    _msgJointStates.position[k] = me->_toolJointsAll(k);
  }
  _pubToolJointStates.publish(_msgJointStates);
  // _mutex.unlock();
}

void surgicalTool::publishToolJointCommands() {
  // _mutex.lock();
  
  if (_flagCurrentToolJointsRead)
  {
   for (size_t i = 0; i < _nDOF; i++)
    { 
      if(i==tool_joint3_yaw)
      { 
          _msgJointCommands.data[i] = me->_toolJointsAll(i);//
      }
      else
      {
         _msgJointCommands.data[i] =  Utils_math<double>::bound(me->_toolJointsAll(i), me->_toolJointLimsAll[L_MIN]->data(i),me->_toolJointLimsAll[L_MAX]->data(i));
      }

    }

  _pubToolJointCommands.publish(_msgJointCommands);
  }
  // _mutex.unlock();
}




void surgicalTool::calculateDesiredFrame(){

  if(_tool_type==FORCEPS){
    _desiredTargetFrame.p.data[0] =Utils_math<double>::map( _platformJoints(p_x),
                                                            -_platformJointLimsDelta->data(p_x), _platformJointLimsDelta->data(p_x), 
                                                            LimitsX[_tool_id-1][0],LimitsX[_tool_id-1][1]);
    _desiredTargetFrame.p.data[1] = Utils_math<double>::map( _platformJoints(p_y),
                                                            -_platformJointLimsDelta->data(p_y), _platformJointLimsDelta->data(p_y), 
                                                            -0.15,0.15);
    _desiredTargetFrame.p.data[2] = -0.12 + Utils_math<double>::map( _platformJoints(p_pitch),
                                                            -_platformJointLimsDelta->data(p_pitch), 0.5*_platformJointLimsDelta->data(p_pitch), 
                                                            -0.15,0.15);
    _desiredTargetFrame.p.data[2] =       Utils_math<double>::bound(_desiredTargetFrame.p.data[2],-0.24,-0.06);      
  }else { //! _tool_type=CAMERA

    Eigen::Vector3d relativeFrame;
    relativeFrame.setZero();
    Eigen::Vector3d desiredNewFrame;
    desiredNewFrame.setZero();
    Eigen::Quaterniond prevDesiredTargetRot;
    prevDesiredTargetRot.setIdentity();
    Eigen::Vector3d prevDesiredTargetPos;
    prevDesiredTargetPos.setZero();

    
      relativeFrame(0) = Utils_math<double>::map( Utils_math<double>::deadZone(_platformJoints(p_x),-_deadZoneValues(p_x), _deadZoneValues(p_x)),
                                                            -_platformJointLimsDelta->data(p_x), _platformJointLimsDelta->data(p_x), 
                                                            -1.0,1.0);                                                            
      relativeFrame(1) = Utils_math<double>::map(Utils_math<double>::deadZone(_platformJoints(p_y),-_deadZoneValues(p_y), _deadZoneValues(p_y)),
                                                              -_platformJointLimsDelta->data(p_y), _platformJointLimsDelta->data(p_y), 
                                                              -1.0,1.0);
      relativeFrame(2) = Utils_math<double>::map(Utils_math<double>::deadZone(_platformJoints(p_pitch),-_deadZoneValues(p_pitch), _deadZoneValues(p_pitch)),
                                                              -_platformJointLimsDelta->data(p_pitch), _platformJointLimsDelta->data(p_pitch), 
                                                              -1.0,1.0);
     
     tf::quaternionKDLToEigen(_myFrames[_mySegments.size()-1].M,prevDesiredTargetRot);
     tf::vectorKDLToEigen(_myFrames[_mySegments.size()-1].p,prevDesiredTargetPos);

      
      desiredNewFrame = prevDesiredTargetRot._transformVector(prevDesiredTargetRot.inverse()._transformVector(prevDesiredTargetPos) + relativeFrame*_speedControlGainCamera*_dt);
      desiredNewFrame = Utils_math<double>::bound(desiredNewFrame,0.27);
     // std::cout<<desiredNewFrame.transpose()<<std::endl;
      tf::vectorEigenToKDL(desiredNewFrame,_desiredTargetFrame.p);
      _desiredTargetFrame.p.data[2] = Utils_math<double>::bound(_desiredTargetFrame.p.data[2],-0.24,-0.06);      
  }

  Eigen::Vector3d p_;
  tf::vectorKDLToEigen(_desiredTargetFrame.p,p_);  
  Quaterniond q_ = Quaterniond::FromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0), p_);
  tf::quaternionEigenToKDL(q_,_desiredTargetFrame.M);
}


void surgicalTool::publishToolTipPose(){
  _msgToolTipPose.header.stamp = ros::Time::now();
  tf::poseKDLToMsg(_myFrames_tip[_nDOF-1],_msgToolTipPose.pose);
  _pubToolTipPose.publish(_msgToolTipPose);
}


// void surgicalTool::readFootTipBasePose()
// {
//   static int count = 0;
//   std::string original_frame;
//   std::string destination_frame;

//   if (_tool_id==Tool_Name::LEFT)
//   {
//     destination_frame = "left_platform_pedal_tip";
//     original_frame = "left_platform_base_link";
//   }
//   else 
//   {
//     destination_frame = "right_platform_pedal_tip";
//     original_frame = "right_platform_base_link";
//   }

//   geometry_msgs::TransformStamped footPoseTransform_;

//   try {
//     footPoseTransform_ = _tfBuffer.lookupTransform(
//         original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
    
    
//     if (!_flagFootTipPoseConnected) {
//       _footTipPosFrameInit = tf2::transformToKDL(footPoseTransform_);
//     }
    
//     else{
//       KDL::Frame footPosFrame_temp = tf2::transformToKDL(footPoseTransform_);
//       _footTipPosFrame.p = (footPosFrame_temp.p - _footTipPosFrameInit.p);
//       tf::vectorKDLToEigen(_footTipPosFrame.p,_footTipPosition);
//       // cout<<_footTipPosition.transpose()<<endl;
//       if (_footTipPosition.norm() > FLT_EPSILON)
//       {
//         _footTipRotationMatrix = Utils_math<double>::rodriguesRotation(
//             Eigen::Vector3d(0.0, 0.0, 1.0), _footTipPosition);
//       }
//       else
//       {
//         _footTipRotationMatrix.setIdentity();
//       }
//       //cout<<_footTipRotationMatrix<<endl;
//       _footTipQuaternion = Eigen::Quaternion <double> (_footTipRotationMatrix);
//       tf::quaternionEigenToKDL(_footTipQuaternion, _footTipPosFrame.M);
//     }
//     _flagFootTipPoseConnected = true;
    
//   } catch (tf2::TransformException ex) {
//     if (count>2)
//     { ROS_ERROR("%s", ex.what());
//       count = 0;
//     }
//     else
//     {
//       count++;
//     }
//     ros::Duration(1.0).sleep();
//   }
// }



void surgicalTool::performInverseKinematics(){
  *me->_toolJointsInit = *me->_toolJoints;
  int ret = _myPosIkSolver_wrist->CartToJnt(*me->_toolJointsInit,_desiredTargetFrame,*me->_toolJoints);

  if(_tool_selfRotation==R_POSITION)  
  {
    _toolJoints->data(tool_joint3_yaw) = -Utils_math<double>::map( (_platformJoints(p_yaw)) , 
                                    -25*DEG_TO_RAD, 25*DEG_TO_RAD, 
                                    -1.5*M_PI, 1.5*M_PI) + M_PI_2;
  } else
  // {
     _toolJoints->data(tool_joint3_yaw) = _toolJointsAllPrev(tool_joint3_yaw) - Utils_math<double>::map( Utils_math<double>::deadZone(_platformJoints(p_yaw),-_deadZoneValues(p_yaw),_deadZoneValues(p_yaw)), 
                                    -25*DEG_TO_RAD, 25*DEG_TO_RAD, -1.0, 1.0) * _speedControlGainSelfRotation * _dt ;
    // cout<<_toolJoints->data(tool_joint3_yaw)<<endl;                                  
    _toolJoints->data(tool_joint3_yaw) = Utils_math<double>::bound(_toolJoints->data(tool_joint3_yaw),M_PI_2-1.5*M_PI,M_PI_2+1.5*M_PI);

  // }

  if (_tool_type==FORCEPS)
    {
      _toolJointsAll(tool_joint5_wrist_open_angle) = _hAxisFilterGrasp->update (Utils_math<double>::map( (-(_platformJoints(p_roll) - _platformJointsOffset(p_roll))),
                                              0*_platformJointLimsDelta->data(p_roll), 1.0*_platformJointLimsDelta->data(p_roll), 
                                              _toolJointLimsAll[L_MIN]->data(tool_joint5_wrist_open_angle), _toolJointLimsAll[L_MAX]->data(tool_joint5_wrist_open_angle)));
      _toolJointsAll(tool_joint5_wrist_open_angle_mimic) = _toolJointsAll(tool_joint5_wrist_open_angle) ;
    }
 
  _toolJointsPosFiltered =  _hAxisFilterPos->update(_toolJoints->data.segment(0,NB_TOOL_AXIS_RED));
  _toolJointsAll.segment(0, NB_TOOL_AXIS_RED) = _toolJointsPosFiltered;
  _toolJointsAll.cwiseMin(_toolJointLimsAll[L_MAX]->data).cwiseMax(_toolJointLimsAll[L_MIN]->data);

  if (ret<0)
  {
    if (_mySolutionFound) 
    {
      *me->_toolJoints = *me->_toolJointsInit;
      
        ROS_ERROR("No tool IK solutions for the tool found yet... move around to find one");
      _mySolutionFound=false;
    }
  }
  else{
    if(!_mySolutionFound)
    {
      *me->_toolJointsInit = *me->_toolJoints;
      ROS_INFO("Solutions of tool IK found again!");
      _mySolutionFound=true;
    }
  }
}

void surgicalTool::performChainForwardKinematics() 
{
  KDL::Frame frame_;
  _myFrames.clear();
  //cout<<_toolJoints->data.transpose()<<endl;
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
      _myFKSolver_wrist->JntToCart(*me->_toolJoints, frame_, i + 1);
      _myFrames.push_back(frame_); 
    //  cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
  }
  KDL::Frame frameTip_;
  _myFrames_tip.clear();
  _toolJointsFull->data=_toolJointsAll;
  //cout<<_toolJointsFull->data.transpose()<<endl;
  for (unsigned int j = 0; j < _mySegmentsTip.size(); j++) {
      _myFKSolver_tip->JntToCart(*me->_toolJointsFull, frameTip_, j + 1);
      // cout<<frameTip_.p.data[0]<<" "<<frameTip_.p.data[1]<<" "<<frameTip_.p.data[2]<<endl;
      _myFrames_tip.push_back(frameTip_);
  }  
  publishToolTipPose();
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


void surgicalTool::readCurrentToolJoints(const sensor_msgs::JointState::ConstPtr &msg) {


  _msgToolCurrentJointStates = *msg;

  if (!_flagCurrentToolJointsRead) {
    ROS_INFO("Current Tools joints received by the tool");
      // for (size_t i = 0; i < NB_TOOL_AXIS_RED; i++)
      // {
      //   _toolJointsInit->data[i] = msg->position[i];
      // }
    _flagCurrentToolJointsRead = true;
  }

}



void surgicalTool::readSharedGrasp(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr &msg){
    if (_tool_type==FORCEPS)
    {
      for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
      {
        _hAxisFilterPosValue(i)=1.0 - msg->sGrasp_hFilters[Axis_Mod[i]];
      }

      _hAxisFilterGraspValue = 1.0 - msg->sGrasp_hFilters[p_roll];
      //cout<<_hAxisFilterGraspValue<<endl;

      _hAxisFilterPos->setAlphas(_hAxisFilterPosValue);
      _hAxisFilterGrasp->setAlpha(_hAxisFilterGraspValue);
    }
}