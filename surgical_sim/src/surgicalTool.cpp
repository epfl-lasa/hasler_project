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


char const *Tool_Names[]{"none","right", "left"};
char const *Leg_Names[]{"none","right", "left"};
char const *Platform_Names[]{"none","right", "left"};

surgicalTool *surgicalTool::me = NULL;



#define MIN_THRESHOLD 5.0
#define MAX_THRESHOLD 15.0
#define STD 0.7

const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};

const double LimitsCart[NB_CART_AXIS][NB_TOOLS][NB_LIMS] = {{{-0.15,0.05},{0.05,0.15}},  //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]
                                                                        {{-0.15,0.15},{-0.15,0.15}}, //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]
                                                                        {{-0.26,-0.06},{-0.26,-0.06}} }; //! [[RIGHT_MIN RIGHT_MAX] [LEFT_MIN LEFT_MAX]]

const double DesiredPlatformWorkspace[NB_PLATFORM_AXIS][NB_TOOLS][NB_LIMS] = {{{-0.090,0.090},{-0.090,0.090}},  //! [RIGHT_Y_MIN RIGHT_Y_MAX] [LEFT_Y_MIN LEFT_Y_MAX]
                                                                                          {{-0.0975,0.0975},{-0.0975,0.0975}}, //! [RIGHT_X_MIN RIGHT_X_MAX] [LEFT_X_MIN LEFT_X_MAX]
                                                                                          {{-11.0*DEG_TO_RAD,5.0*DEG_TO_RAD},{-11.0*DEG_TO_RAD,5.0*DEG_TO_RAD}}, //! [RIGHT_PITCH_MIN RIGHT_PITCH_MAX] [LEFT_PITCH_MIN LEFT_PITCH_MAX]
                                                                                          {{0.0*DEG_TO_RAD,15.0*DEG_TO_RAD},{-15.0*DEG_TO_RAD,0.0*DEG_TO_RAD}}, //! [RIGHT_ROLL_MIN RIGHT_ROLL_MAX] [LEFT_ROLL_MIN LEFT_ROLL_MAX]
                                                                                          {{-20.0*DEG_TO_RAD,20.0*DEG_TO_RAD},{-20.0*DEG_TO_RAD,20.0*DEG_TO_RAD}} }; //! [RIGHT_YAW_MIN RIGHT_YAW_MAX] [LEFT_YAW_MIN LEFT_YAW_MAX]


surgicalTool::surgicalTool(ros::NodeHandle &n_1, double frequency,
                   surgicalTool::Tool_Name tool_id, urdf::Model model_)
    : _n(n_1), _tool_id(tool_id), _loopRate(frequency), _dt(1.0f / frequency),
      _myModel(model_) {
   me = this;
  _stop = false;
  _flagToolEnabled=false;
  _flagForwardKinematicsStarted=false;

  _myInput = LEG_INPUT;
  std::string inputName_;

  if (!_n.getParam("controlInput", inputName_))
  { 
      ROS_ERROR("[%s tool]: No controlInput param",Tool_Names[_tool_id]); 
  }

  if (inputName_.compare("platform") == 0)
  {
    ROS_INFO("[%s tool]: Control input changed to platform",Tool_Names[_tool_id]);
    _myInput=PLATFORM_INPUT;
  }
  //! Get parameter of interaction mode
  // std::string interactionMode_="individual";
  // if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"interactionMode", interactionMode_))
  // { 
  //     ROS_ERROR("[%s tool]: No interactionMode param",Tool_Names[_tool_id]); 
  // }
  // _myInteractionMode = interactionMode_.compare("mixed")==0 ? MIXED_MODE : INDIVIDUAL_MODE;
  bool useOneFootForTwoTools = false;
  if (!_n.getParam("/useOneFootForTwoTools", useOneFootForTwoTools))
  { 
      ROS_INFO("[%s tool]: No useOneFootForTwoTools param, default to false",Tool_Names[_tool_id]); 
  }else
  {
    useOneFootForTwoTools = false;
  }
  _myInteractionMode= useOneFootForTwoTools ? MIXED_MODE : INDIVIDUAL_MODE;

  if (_myInteractionMode==MIXED_MODE)
  {
      //! Get parameter of 
     _graspITofAuxPlatform = p_pitch; 
     std::vector<std::string> mappingAux_;

    mappingAux_.assign(NB_PLATFORM_AXIS,"");
    
   
    if (!_n.getParam("/mixed_platform/mappingAux", mappingAux_))
    { 
      ROS_ERROR("[%s tool]: No /mixed_platform/mappingAux present ",Tool_Names[_tool_id]);
    }else {
      ROS_INFO("[%s tool]: mappingAux  param loaded",Tool_Names[_tool_id]); 
    }   
    
      for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      {
          if (mappingAux_[i].compare("grasp")==0)
          {
            _graspITofAuxPlatform = i;;
            break;
          } 
      }   

  }
  

  std::string toolTypeStr_;
  if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"toolType", toolTypeStr_))
  { 
      ROS_ERROR("[%s tool]: No toolType param for tool", Tool_Names[_tool_id] );
  }
  if (toolTypeStr_.compare("camera") == 0)
  {
    ROS_INFO("[%s tool]: The tool is a camera",Tool_Names[_tool_id]);
    _tool_type=CAMERA;
  } else if (toolTypeStr_.compare("forceps") == 0){
    ROS_INFO("[%s tool]: The tool is a forceps", Tool_Names[_tool_id]);
    _tool_type=FORCEPS;
  } else
  {
    ROS_ERROR("[%s tool]: No type of tool detected, aborting", Tool_Names[_tool_id]);
    _stop=true;
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
  _toolJoints.resize(NB_TOOL_AXIS_RED);
  _toolJoints.data.setZero();
  _toolJointsFull.resize(_nDOF);
  _toolJointsFull.data.setZero();
  _toolJointsInit.resize(NB_TOOL_AXIS_RED);
  _toolJointsInit.data.setZero();
  if (_tool_id==RIGHT_TOOL)
  {
    _toolJoints.data<<1.270095140171179e-12, -0.6202494859926144, 1.5707963267948974, 0.10162325267042625;
    _toolJointsInit.data<<1.270095140171179e-12, -0.6202494859926144, 1.5707963267948974, 0.10162325267042625;
  }
  else
  {
    _toolJoints.data<<-0.09072372302531484, 0.8903616344174168, 1.5038493544012193, 0.13449672820206968;
    _toolJointsInit.data<<-0.09072372302531484, 0.8903616344174168, 1.5038493544012193, 0.13449672820206968;
  } 



  
  _legJointLims[L_MIN].resize(NB_LEG_AXIS);
  _legJointLims[L_MIN].data.setZero();
  _legJointLims[L_MAX].resize(NB_LEG_AXIS);
  _legJointLims[L_MAX].data.setZero();

  _platformJointLims[L_MIN].resize(NB_PLATFORM_AXIS);
  _platformJointLims[L_MIN].data.setZero();
  _platformJointLims[L_MAX].resize(NB_PLATFORM_AXIS);
  _platformJointLims[L_MAX].data.setZero();
  // _platformJointLimsDelta.resize(NB_PLATFORM_AXIS);

  _toolJointLims[L_MIN].resize(NB_TOOL_AXIS_RED);
  _toolJointLims[L_MAX].resize(NB_TOOL_AXIS_RED);
  _toolJointLimsAll[L_MIN].resize(_nDOF);
  _toolJointLimsAll[L_MAX].resize(_nDOF);

  _toolJointLims[L_MIN].data.setZero();
  _toolJointLims[L_MAX].data.setZero();
  _toolJointLimsAll[L_MIN].data.setZero();
  _toolJointLimsAll[L_MAX].data.setZero();

  _weightedJointSpaceMassMatrix.setIdentity();
  _weightedTaskSpaceMassMatrix.setIdentity();
  _myJacobian.resize(NB_TOOL_AXIS_RED);


  _desiredToolEEFrame.Identity();
  _desiredToolEEFrame.M.Identity();
  _desiredToolEEFrame.p.Zero();

  _desiredToolEEFrameOffset.setZero();
  


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
  
  _toolJointsAll.resize(_nDOF);
  _toolJointsAllPrev.resize(_nDOF);
  _toolJointsAllSpeed.resize(_nDOF);

  _toolJointsAll.setZero();
  _toolJointsAllPrev.setZero();
  _toolJointsAllSpeed.setZero();


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
      ROS_ERROR("[%s tool]: No sharedGrasp param", Tool_Names[_tool_id]); 
  }

  if (_flagSharedGrasp) cout<<"flag shared grasp enabled"<<endl;

  if (!_n.getParam("rosControl", _flagRosControl))
  { 
      ROS_ERROR("[%s tool]: No rosControl param", Tool_Names[_tool_id]); 
  }

  if (_flagSharedGrasp) cout<<"flag ros control for the tool enabled"<<endl;

  if (_myInput==LEG_INPUT)
  {
    if (!_legModel.initParam("/"+std::string(Tool_Names[_tool_id]) + "_leg/robot_description")) {
        ROS_ERROR("[%s tool]: Failed to parse urdf of the leg", Tool_Names[_tool_id]);
    }
    else
    {
      for (unsigned int i = 0; i<NB_LEG_AXIS; i++)
      {
        _legJointLims[L_MIN].data(i) = _legModel.getJoint("/"+std::string(Tool_Names[_tool_id]) + "_" + Leg_Axis_Names[i])->limits->lower;
        _legJointLims[L_MAX].data(i) = _legModel.getJoint("/"+std::string(Tool_Names[_tool_id]) + "_" + Leg_Axis_Names[i])->limits->upper;
      }
    }
    


  }
  else
  {
    if (!_platformModel.initParam("/"+std::string(Tool_Names[_tool_id]) + "_platform/robot_description")) {
        ROS_ERROR("[%s tool]: Failed to parse urdf file of plaform", Tool_Names[_tool_id]);
        _stop=true;
    }
    else
    {
      for (unsigned int i = 0; i<NB_PLATFORM_AXIS; i++)
      {
        _platformJointLims[L_MIN].data(i) = _platformModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->lower;
        //cout<<_platformJointLims[L_MIN].data(i)<<endl;
        _platformJointLims[L_MAX].data(i) = _platformModel.getJoint( std::string(Tool_Names[_tool_id]) + "_" + std::string(Platform_Axis_Names[i]))->limits->upper;
        //cout<<_platformJointLims[L_MAX].data(i)<<endl;
      }
    }


      for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      {
        for (size_t j = 0; j < NB_LIMS; j++)
        {
          _desiredPlatformWSLims(i,j) = DesiredPlatformWorkspace[i][_tool_id-1][j];
        }
      }


       std::vector<double> desiredPlatformWorkspaceLimsMin;
      desiredPlatformWorkspaceLimsMin.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"desiredPlatformWorkspaceLimsMin", desiredPlatformWorkspaceLimsMin))
      { 
          ROS_ERROR(" [%s tool]: No desiredPlatformWorkspaceLimsMin  param", Tool_Names[_tool_id]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _desiredPlatformWSLims(i,L_MIN)=desiredPlatformWorkspaceLimsMin[i];
        }
      
      std::vector<double> desiredPlatformWorkspaceLimsMax;
      desiredPlatformWorkspaceLimsMax.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"desiredPlatformWorkspaceLimsMax", desiredPlatformWorkspaceLimsMax))
      { 
          ROS_ERROR(" [%s tool]: No desiredPlatformWorkspaceLimsMax  param", Tool_Names[_tool_id]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _desiredPlatformWSLims(i,L_MAX)=desiredPlatformWorkspaceLimsMax[i];
        }
        
      
        
      _tool_selfRotation = R_SPEED;
      std::string toolSelfRotationControlStr;
      if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"toolSelfRotationControl", toolSelfRotationControlStr))
      { 
          ROS_ERROR("[%s tool]: No toolSelfRotationControl (position,speed) param", Tool_Names[_tool_id]); 
      }
      if( toolSelfRotationControlStr.compare("position")==0)
      {
        _tool_selfRotation = R_POSITION;
      }
      
      _speedControlGainCamera = 0.5;
      _speedControlGainSelfRotation=50.0; 
      
      if (_tool_selfRotation==R_SPEED)
      {
        if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"speedControlGainCamera", _speedControlGainCamera))
        { 
            ROS_ERROR("[%s tool]: No speedControlGainCamera  param", Tool_Names[_tool_id]); 
        }
        if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"speedControlGainSelfRotation", _speedControlGainSelfRotation))
        { 
            ROS_ERROR("[%s tool]: No speedControlGainSelfRotation  param", Tool_Names[_tool_id]); 
        }
      }

      _platformJointsOffset.setZero();
      // std::vector<double> platformJointOffsets;
      // platformJointOffsets.resize(NB_PLATFORM_AXIS);
      // if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"platformJointOffsets", platformJointOffsets))
      // { 
      //     ROS_ERROR(" [%s tool]: No platformJointOffsets  param", Tool_Names[_tool_id]); 
      // }
      //   for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
      //   {
      //     _platformJointsOffset(i)=platformJointOffsets[i];
      //   }

      _deadZoneValues = 0.2*_desiredPlatformWSLims;
      //cout<<_deadZoneValues.transpose()<<endl;

      std::vector<double> deadZoneValuesPlatformMin;
      deadZoneValuesPlatformMin.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"deadZoneValuesPlatformMin", deadZoneValuesPlatformMin))
      { 
          ROS_ERROR(" [%s tool]: No deadZoneValuesPlatformMin  param", Tool_Names[_tool_id]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _deadZoneValues(i,L_MIN)=deadZoneValuesPlatformMin[i];
        }
      
      std::vector<double> deadZoneValuesPlatformMax;
      deadZoneValuesPlatformMax.resize(NB_PLATFORM_AXIS);
      if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"deadZoneValuesPlatformMax", deadZoneValuesPlatformMax))
      { 
          ROS_ERROR(" [%s tool]: No deadZoneValuesPlatformMax  param", Tool_Names[_tool_id]); 
      }
        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {
          _deadZoneValues(i,L_MAX)=deadZoneValuesPlatformMax[i];
        }

      


  }

    if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
      ROS_ERROR("[%s tool]: Failed to construct kdl tree", Tool_Names[_tool_id]);
      _stop = true;

  }


  for (size_t i = 0; i < NB_CART_AXIS; i++)
  {
    for (size_t j = 0; j < NB_LIMS; j++)
    {
      _cartesianLimits(i,j) = LimitsCart[i][_tool_id-1][j];
    }
  }

   std::vector<double> cartLimsX;
   std::vector<double> cartLimsY;
   std::vector<double> cartLimsZ;
  
    cartLimsX.resize(NB_LIMS);     cartLimsY.resize(NB_LIMS);    cartLimsZ.resize(NB_LIMS);


    if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"cartLimsX", cartLimsX))
    { 
        ROS_ERROR("[%s tool]: No cartLimsX param", Tool_Names[_tool_id]); 
    }

     if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"cartLimsY", cartLimsY))
    { 
        ROS_ERROR("[%s tool]: No cartLimsY param", Tool_Names[_tool_id]); 
    }

     if (!_n.getParam("/"+std::string(Tool_Names[_tool_id])+"_tool/"+"cartLimsZ", cartLimsZ))
    { 
        ROS_ERROR("[%s tool]: No cartLimsZ param", Tool_Names[_tool_id]); 
    }
      
      
      for (size_t i = 0; i < NB_LIMS; i++)
      {
        _cartesianLimits(CART_X,i) = cartLimsX[i];
        _cartesianLimits(CART_Y,i) = cartLimsY[i];
        _cartesianLimits(CART_Z,i) = cartLimsZ[i];
      }
  

  KDL::Vector grav_vector(0.0, 0.0, GRAVITY);

  _myTree.getChain(std::string(Tool_Names[_tool_id]) + "_tool_base_link", std::string(Tool_Names[_tool_id]) + "_tool_wrist_link", _myToolBaseToWristChain);
  _myTree.getChain(std::string(Tool_Names[_tool_id]) + "_tool_base_link", std::string(Tool_Names[_tool_id]) + "_tool_tip_link_ee", _myToolBaseToTipChain);


  _myFKSolver_wrist = new KDL::ChainFkSolverPos_recursive(_myToolBaseToWristChain);
  _myFKSolver_tip = new KDL::ChainFkSolverPos_recursive(_myToolBaseToTipChain);

  _myJacSolver_wrist = new KDL::ChainJntToJacSolver(_myToolBaseToWristChain);

  _mySegments = _myToolBaseToWristChain.segments;
  _mySegmentsTip = _myToolBaseToTipChain.segments;
  _myFrames.assign(_mySegments.size(),KDL::Frame(KDL::Rotation::Identity(),KDL::Vector::Zero()));
  _myFrames_tip.assign(_mySegmentsTip.size(),KDL::Frame(KDL::Rotation::Identity(),KDL::Vector::Zero()));


  // double test = Utils_math<double>::map(2.5,-5,5,-1,1);
  // cout<<test<<endl;

  for (unsigned int joint_=0; joint_ < _nDOF; joint_++ )
  {
    _toolJointLimsAll[L_MIN].data(joint_) = _myModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + Tool_Axis_Names[joint_])->limits->lower;
    _toolJointLimsAll[L_MAX].data(joint_) = _myModel.getJoint(std::string(Tool_Names[_tool_id]) + "_" + Tool_Axis_Names[joint_])->limits->upper;
      
    if (joint_<NB_TOOL_AXIS_RED)
    {
      _toolJointLims[L_MIN].data(joint_) = _toolJointLimsAll[L_MIN].data(joint_) ;
      _toolJointLims[L_MAX].data(joint_) = _toolJointLimsAll[L_MAX].data(joint_) ;
    }
    if (_tool_id==LEFT_TOOL){
     _toolJointLims[L_MIN].data(tool_joint3_yaw) = -1.6*M_PI - 1.5*M_PI_2; 
     _toolJointLims[L_MAX].data(tool_joint3_yaw) =  1.6*M_PI - 0.0*M_PI_2;
     _toolJointLimsAll[L_MIN].data(tool_joint3_yaw) = -1.6*M_PI - 1.5*M_PI_2; 
     _toolJointLimsAll[L_MAX].data(tool_joint3_yaw) =  1.6*M_PI - 0.0*M_PI_2;
    }
    if (_tool_id==RIGHT_TOOL){
     _toolJointLims[L_MIN].data(tool_joint3_yaw) = -1.6*M_PI + 0.0*M_PI_2; 
     _toolJointLims[L_MAX].data(tool_joint3_yaw) =  1.6*M_PI + 1.5*M_PI_2;
     _toolJointLimsAll[L_MIN].data(tool_joint3_yaw) = -1.6*M_PI + 0.0*M_PI_2; 
     _toolJointLimsAll[L_MAX].data(tool_joint3_yaw) =  1.6*M_PI + 1.5*M_PI_2;
    }
  }


  /****************************** Inverse Kinematics of the tool ***********************************/
      _myVelIKSolver_wrist = new KDL::ChainIkSolverVel_wdls(_myToolBaseToWristChain);
      _myPosIkSolver_wrist= new KDL::ChainIkSolverPos_NR_JL(_myToolBaseToWristChain,_toolJointLims[L_MIN], _toolJointLims[L_MAX], *_myFKSolver_wrist,*_myVelIKSolver_wrist);
      
      
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
   _subCurrentToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_tool_id])+"_tool/joint_states"
      , 1, boost::bind(&surgicalTool::readCurrentToolJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  if (_flagRosControl)
    {
      _pubToolJointCommands = _n.advertise<std_msgs::Float64MultiArray>("/"+std::string(Tool_Names[_tool_id])+"_tool/joint_position_controller/command", 1);
    }
  else
  {
    _pubToolJointStates = _n.advertise<sensor_msgs::JointState>("/"+std::string(Tool_Names[_tool_id])+"_tool/joint_states", 1);
  }
  
  _pubToolTipPose = _n.advertise<geometry_msgs::PoseStamped>("tool_tip_pose", 1);
 
  _subLegJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_tool_id])+"_leg/leg_joint_publisher/joint_states"
      , 1, boost::bind(&surgicalTool::readLegJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  
  if (_myInteractionMode==MIXED_MODE)
  {
    _subPlatformJointStates = _n.subscribe<custom_msgs::TwoFeetOneToolMsg>( "/mixed_platform/platform_state"
      , 1, boost::bind(&surgicalTool::readMixedPlatformControlState, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      
  } else
  {
    _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Platform_Names[_tool_id])+"_platform/platform_joint_publisher/joint_states"
    , 1, boost::bind(&surgicalTool::readPlatformJoints, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      
  }
    
  _subSharedGrasp = _n.subscribe<custom_msgs_gripper::SharedGraspingMsg>( "/"+std::string(Platform_Names[_tool_id])+"_tool/sharedGrasping"
      , 1, boost::bind(&surgicalTool::readSharedGrasp, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      



  // Subscriber definitions
  signal(SIGINT, surgicalTool::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO(" [%s tool]: The tool joint state publisher is about to start ", Tool_Names[_tool_id]);
    return true;
  } else {
    ROS_ERROR("[%s tool]: The ros node has a problem.", Tool_Names[_tool_id]);
    return false;
  }
}

void surgicalTool::stopNode(int sig) { me->_stop = true; }

void surgicalTool::run() {
  static int count = 0;
  while (!_stop) {
    if ((_subPlatformJointStates.getNumPublishers()==0 && _myInput==PLATFORM_INPUT) || 
        (_subLegJointStates.getNumPublishers()==0 && _myInput==LEG_INPUT)) {
          if (count>10){
            ROS_WARN_ONCE("[%s tool]: No control input for the tool connected", Tool_Names[_tool_id]);
            count=0;
          }
          count++;
        } else {
          ROS_INFO_ONCE("[%s tool]: The control input for the tool connected", Tool_Names[_tool_id]);
          if (_flagCurrentToolJointsRead){
            performChainForwardKinematics();
            if (_flagToolEnabled || _myInteractionMode==INDIVIDUAL_MODE) {          
            ROS_INFO_ONCE("[%s tool]: The control started", Tool_Names[_tool_id]);
            //readFootTipBasePose();
            switch (_myInput)
            {
                case PLATFORM_INPUT:
                {

                      calculateDesiredFrame();

            //       //computeWithPlatformTask4DoF();
            //       //computeWithPlatformJoints4DoF();
                
                    
                      performInverseKinematics();                  
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
          }
        }
        ros::spinOnce();
        _loopRate.sleep();
  }
  ROS_INFO(" [%s tool]: Tool state variables stopped", Tool_Names[_tool_id]);
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
    _msgJointStates.position[k] = _toolJointsAll(k);
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
        _msgJointCommands.data[i] =  Utils_math<double>::bound(_toolJointsAll(i), _toolJointLimsAll[L_MIN].data(i),_toolJointLimsAll[L_MAX].data(i));
    }

  _pubToolJointCommands.publish(_msgJointCommands);
  }
  // _mutex.unlock();
}




void surgicalTool::calculateDesiredFrame(){

  if(_tool_type==FORCEPS){
     Eigen::Vector3d relativeFrame;
    relativeFrame.setZero();
    Eigen::Vector3d desiredNewFrame;
    desiredNewFrame.setZero();
    Eigen::Vector3d defaultFrame;
    defaultFrame.setZero();

    defaultFrame(CART_X) = Utils_math<double>::map( 0.0,
                                                      _desiredPlatformWSLims(p_x,L_MIN), _desiredPlatformWSLims(p_x,L_MAX), 
                                                      _cartesianLimits(CART_X,L_MIN),_cartesianLimits(CART_X,L_MAX));
    defaultFrame(CART_Y) = Utils_math<double>::map( 0.0,
                                                      _desiredPlatformWSLims(p_y,L_MIN), _desiredPlatformWSLims(p_y,L_MAX), 
                                                      _cartesianLimits(CART_Y,L_MIN),_cartesianLimits(CART_Y,L_MAX));
    defaultFrame(CART_Z) = Utils_math<double>::map( 0.0,
                                                      _desiredPlatformWSLims(p_pitch,L_MIN), _desiredPlatformWSLims(p_pitch,L_MAX), 
                                                      _cartesianLimits(CART_Z,L_MIN),_cartesianLimits(CART_Z,L_MAX));
    

    relativeFrame(CART_X) = Utils_math<double>::map( _platformJoints(p_x) + _platformJointsOffset(p_x),
                                                      _desiredPlatformWSLims(p_x,L_MIN), _desiredPlatformWSLims(p_x,L_MAX), 
                                                      _cartesianLimits(CART_X,L_MIN),_cartesianLimits(CART_X,L_MAX));
    relativeFrame(CART_Y) = Utils_math<double>::map( _platformJoints(p_y) + _platformJointsOffset(p_y),
                                                      _desiredPlatformWSLims(p_y,L_MIN), _desiredPlatformWSLims(p_y,L_MAX), 
                                                      _cartesianLimits(CART_Y,L_MIN),_cartesianLimits(CART_Y,L_MAX));
    relativeFrame(CART_Z) = Utils_math<double>::map( _platformJoints(p_pitch) + _platformJointsOffset(p_pitch),
                                                      _desiredPlatformWSLims(p_pitch,L_MIN), _desiredPlatformWSLims(p_pitch,L_MAX), 
                                                      _cartesianLimits(CART_Z,L_MIN),_cartesianLimits(CART_Z,L_MAX));
    
    desiredNewFrame = relativeFrame +  (_desiredToolEEFrameOffset-defaultFrame);
    
    desiredNewFrame(CART_X) =  Utils_math<double>::bound(desiredNewFrame(CART_X),_cartesianLimits(CART_X,L_MIN),_cartesianLimits(CART_X,L_MAX));
    desiredNewFrame(CART_Y) =  Utils_math<double>::bound(desiredNewFrame(CART_Y),_cartesianLimits(CART_Y,L_MIN),_cartesianLimits(CART_Y,L_MAX));
    desiredNewFrame(CART_Z) =  Utils_math<double>::bound(desiredNewFrame(CART_Z),_cartesianLimits(CART_Z,L_MIN),_cartesianLimits(CART_Z,L_MAX));
    tf::vectorEigenToKDL(desiredNewFrame,_desiredToolEEFrame.p);  
    
  }else { //! _tool_type=CAMERA

    Eigen::Vector3d relativeFrame;
    relativeFrame.setZero();
    Eigen::Vector3d desiredNewFrame;
    desiredNewFrame.setZero();
    Eigen::Quaterniond prevDesiredTargetRot;
    prevDesiredTargetRot.setIdentity();
    Eigen::Vector3d prevDesiredTargetPos;
    prevDesiredTargetPos.setZero();
    
    static Eigen::Matrix<double,NB_PLATFORM_AXIS,1> averageOfPlatformLims = _desiredPlatformWSLims.rowwise().mean().cwiseAbs();
    
      relativeFrame(CART_X) = Utils_math<double>::map( Utils_math<double>::deadZone(_platformJoints(p_x),_deadZoneValues(p_x,L_MIN), _deadZoneValues(p_x,L_MAX)),
                                                            -averageOfPlatformLims(p_x), averageOfPlatformLims(p_x), 
                                                            -1.0,1.0);                                                            
      relativeFrame(CART_Y) = -Utils_math<double>::map(Utils_math<double>::deadZone(_platformJoints(p_y),_deadZoneValues(p_y,L_MIN), _deadZoneValues(p_y,L_MAX)),
                                                              -averageOfPlatformLims(p_y), averageOfPlatformLims(p_y), 
                                                              -1.0,1.0); //! The minus sign, is because from the point of view of the camera Up is -Y
      relativeFrame(CART_Z) = -Utils_math<double>::map(Utils_math<double>::deadZone(_platformJoints(p_pitch),_deadZoneValues(p_pitch,L_MIN), _deadZoneValues(p_pitch,L_MAX)),
                                                              -averageOfPlatformLims(p_pitch), averageOfPlatformLims(p_pitch),  
                                                              -1.0,1.0);                                                              
     
     //std::cout<<relativeFrame.transpose()<<std::endl;
     tf::quaternionKDLToEigen(_myFrames[_mySegments.size()-1].M,prevDesiredTargetRot);
     tf::vectorKDLToEigen(_myFrames[_mySegments.size()-1].p,prevDesiredTargetPos);

      
      desiredNewFrame = prevDesiredTargetRot._transformVector(prevDesiredTargetRot.inverse()._transformVector(prevDesiredTargetPos) + relativeFrame*_speedControlGainCamera*_dt);
      
      desiredNewFrame(CART_X) =  Utils_math<double>::bound(desiredNewFrame(CART_X),_cartesianLimits(CART_X,L_MIN),_cartesianLimits(CART_X,L_MAX));
      desiredNewFrame(CART_Y) =  Utils_math<double>::bound(desiredNewFrame(CART_Y),_cartesianLimits(CART_Y,L_MIN),_cartesianLimits(CART_Y,L_MAX));
      desiredNewFrame(CART_Z) =  Utils_math<double>::bound(desiredNewFrame(CART_Z),_cartesianLimits(CART_Z,L_MIN),_cartesianLimits(CART_Z,L_MAX));
      tf::vectorEigenToKDL(desiredNewFrame,_desiredToolEEFrame.p);
  }
  
  Eigen::Vector3d p_;
  tf::vectorKDLToEigen(_desiredToolEEFrame.p,p_);  
  Quaterniond q_ = Quaterniond::FromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0), p_);
  tf::quaternionEigenToKDL(q_,_desiredToolEEFrame.M);
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
  
  _toolJointsInit.data = _toolJoints.data;

  int ret = _myPosIkSolver_wrist->CartToJnt(_toolJointsInit,_desiredToolEEFrame,_toolJoints);

  if (ret<0)
  {
    if (_mySolutionFound) 
    {  
      ROS_ERROR("[%s tool]: No tool IK solutions for the tool  found yet... move around to find one", Tool_Names[_tool_id]);
      _toolJoints.data = _toolJointsInit.data;
      _mySolutionFound=false;
    }
  }
  else{
    if(!_mySolutionFound)
    {
      ROS_INFO(" [%s tool]: Solutions of tool  IK found again!", Tool_Names[_tool_id]);
      _toolJointsInit.data = _toolJoints.data;
      _mySolutionFound=true;
    }
   }
   

  
  if(_tool_selfRotation==R_POSITION)  
  {
    _toolJoints.data(tool_joint3_yaw) = -Utils_math<double>::map( _platformJoints(p_yaw) + _platformJointsOffset(p_yaw) , 
                                    _desiredPlatformWSLims(p_yaw,L_MIN),_desiredPlatformWSLims(p_yaw,L_MAX), 
                                    _toolJointLimsAll[L_MIN].data(tool_joint3_yaw),_toolJointLimsAll[L_MAX].data(tool_joint3_yaw)) ;
  } else  {
    double speedControlGainSelfRotationMod = 0.0;
    
    if (_tool_id==RIGHT_TOOL){
        speedControlGainSelfRotationMod = _platformJoints(p_yaw) >_deadZoneValues(p_yaw,L_MAX) ? 20.0*_speedControlGainSelfRotation : _speedControlGainSelfRotation;
    }else{

        speedControlGainSelfRotationMod = _platformJoints(p_yaw) < _deadZoneValues(p_yaw,L_MIN) ? 20.0*_speedControlGainSelfRotation : _speedControlGainSelfRotation;
    }

     _toolJoints.data(tool_joint3_yaw) = _toolJointsAllPrev(tool_joint3_yaw) - Utils_math<double>::map( Utils_math<double>::deadZone(_platformJoints(p_yaw) + _platformJointsOffset(p_yaw),_deadZoneValues(p_yaw,L_MIN),_deadZoneValues(p_yaw,L_MAX)), 
                                    _desiredPlatformWSLims(p_yaw,L_MIN),_desiredPlatformWSLims(p_yaw,L_MAX), -1.0, 1.0) * _speedControlGainSelfRotation * _dt ;
    // cout<<_toolJoints.data(tool_joint3_yaw)<<endl;     
   }
    _toolJoints.data(tool_joint3_yaw) = Utils_math<double>::bound(_toolJoints.data(tool_joint3_yaw), _toolJointLimsAll[L_MIN].data(tool_joint3_yaw),_toolJointLimsAll[L_MAX].data(tool_joint3_yaw));                             

  if (_tool_type==FORCEPS)
    {
      
      
      if (_myInteractionMode==INDIVIDUAL_MODE)
      {
        _toolJointsAll(tool_joint5_wrist_open_angle) = _hAxisFilterGrasp->update (Utils_math<double>::map(_platformJoints(p_roll) + _platformJointsOffset(p_roll),
                                                    _desiredPlatformWSLims(p_roll,L_MIN),_desiredPlatformWSLims(p_roll,L_MAX), 
                                                    _toolJointLimsAll[L_MAX].data(tool_joint5_wrist_open_angle), _toolJointLimsAll[L_MIN].data(tool_joint5_wrist_open_angle)));
      } else //! MIXED_MODE
      {
        _toolJointsAll(tool_joint5_wrist_open_angle) = _hAxisFilterGrasp->update (Utils_math<double>::map(_platformJoints(p_roll) + _platformJointsOffset(p_roll),
                                            _desiredPlatformWSLims(_graspITofAuxPlatform,L_MAX),_desiredPlatformWSLims(_graspITofAuxPlatform,L_MIN), 
                                            _toolJointLimsAll[L_MAX].data(tool_joint5_wrist_open_angle), _toolJointLimsAll[L_MIN].data(tool_joint5_wrist_open_angle)));
      }
      // cout<<_toolJointsAll(tool_joint5_wrist_open_angle)<<endl;                                                    
      _toolJointsAll(tool_joint5_wrist_open_angle_mimic) = _toolJointsAll(tool_joint5_wrist_open_angle) ;
    }
 
  _toolJointsPosFiltered =  _hAxisFilterPos->update(_toolJoints.data.segment(0,NB_TOOL_AXIS_RED));
  _toolJointsAll.segment(0, NB_TOOL_AXIS_RED) = _toolJointsPosFiltered;
  _toolJointsAll.cwiseMin(_toolJointLimsAll[L_MAX].data).cwiseMax(_toolJointLimsAll[L_MIN].data);

}

void surgicalTool::performChainForwardKinematics() 
{
  KDL::Frame frame_;
  //cout<<_toolJoints.data.transpose()<<endl;
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
      _myFKSolver_wrist->JntToCart(_toolJoints, frame_, i + 1);
      _myFrames[i]=frame_; 
    //  cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
  }
  KDL::Frame frameTip_;
  _myFrames_tip.resize(_mySegmentsTip.size());
  _toolJointsFull.data=_toolJointsAll;
  //cout<<_toolJointsFull.data.transpose()<<endl;
  for (unsigned int j = 0; j < _mySegmentsTip.size(); j++) {
      _myFKSolver_tip->JntToCart(_toolJointsFull, frameTip_, j + 1);
      // cout<<frameTip_.p.data[0]<<" "<<frameTip_.p.data[1]<<" "<<frameTip_.p.data[2]<<endl;
      _myFrames_tip[j] = frameTip_;
  }  
  publishToolTipPose();
  if(!_flagForwardKinematicsStarted)
  {
    std::cout<<"Current frame of the wrist: "
             <<_myFrames[_mySegments.size()-1].p[0]<<" "
             <<_myFrames[_mySegments.size()-1].p[1]<<" "
             <<_myFrames[_mySegments.size()-1].p[2]<<" "<<std::endl;
  }
  _flagForwardKinematicsStarted=true;
}


void surgicalTool::readLegJoints(const sensor_msgs::JointState::ConstPtr &msg)
{

  for (unsigned int i=0; i<NB_LEG_AXIS; i++)
  {
    me->_legJoints(i) = msg->position[i];
  }

  if (!_flagLegJointsConnected)
  {
    ROS_INFO("[%s tool]: leg joints received by the tool", Tool_Names[_tool_id]);
  }
  _flagLegJointsConnected = true;
}

void surgicalTool::readPlatformJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    me->_platformJoints(i) = msg->position[i];
    me->_platformVelocities(i) = msg->velocity[i];
    me->_platformEfforts(i) = msg->effort[i];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("[%s tool]: platform joints received by the tool",Tool_Names[_tool_id]);
  }
  _flagPlatformJointsConnected = true;
}

void surgicalTool::readMixedPlatformControlState(const custom_msgs::TwoFeetOneToolMsg::ConstPtr &msg) {
  bool enableNow_ = false;
  if(_flagForwardKinematicsStarted)
  {
    enableNow_ = msg->currentTool==_tool_id;

    if (!_flagToolEnabled && enableNow_)
    { 
      tf::vectorKDLToEigen(_myFrames[_mySegments.size()-1].p,_desiredToolEEFrameOffset);
      cout<<"desiredToolOffset: "<<_desiredToolEEFrameOffset.transpose()<<endl;
      _flagToolEnabled=true;
    } 
      _flagToolEnabled=enableNow_;

    if (_flagToolEnabled)
    {
      for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
          me->_platformJoints(i) = msg->mixedPlatformJointState.position[i];
          me->_platformVelocities(i) = msg->mixedPlatformJointState.velocity[i];
          me->_platformEfforts(i) = msg->mixedPlatformJointState.effort[i];
          me->_platformJointsOffset(i) = msg->mixedPlatformOffset[i];
        }  
        
    }
    if (!_flagPlatformJointsConnected) {
        ROS_INFO("[%s tool]: platform joints received by the tool",Tool_Names[_tool_id]);
      }
    _flagPlatformJointsConnected = true;
  }
}


void surgicalTool::readCurrentToolJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  _msgToolCurrentJointStates = *msg;

  if (!_flagCurrentToolJointsRead) {
    ROS_INFO("[%s tool]: Current joints received by the tool", Tool_Names[_tool_id]);
      for (size_t i = 0; i < NB_TOOL_AXIS_RED; i++)
      {
        _toolJoints.data[i] = msg->position[i];
      }
      _toolJointsInit.data = _toolJoints.data;
  }
  _flagCurrentToolJointsRead = true;

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
    ROS_INFO_ONCE("[%s tool]: sharedGrasp read!",Tool_Names[_tool_id]);
}