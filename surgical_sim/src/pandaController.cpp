#include "pandaController.h"
#include <stdio.h>
#include <stdlib.h>
#include "tf_conversions/tf_eigen.h"
#include "Utils_math.h"
#include <iostream>

#define ListofPandaAxes(enumeration, names) names,
char const *Panda_Axis_Names[]{PANDA_AXES};
#undef ListofPandaAxes

char const *Panda_Names[]{"none", "right", "left"};

pandaController *pandaController::me = NULL;


pandaController::pandaController(ros::NodeHandle &n_1, double frequency,
                   pandaController::Panda_Name panda_id, urdf::Model model_)
    : _n(n_1), _panda_id(panda_id), _loopRate(frequency), _dt(1.0f / frequency),
      _myModel(model_) {
   me = this;
  _stop = false;

  std::string inputName_;

  _pandaJoints = new KDL::JntArray(NB_PANDA_AXIS);
  _pandaJoints->data.setZero();
  _pandaJointsInit = new KDL::JntArray(NB_PANDA_AXIS);
  _pandaJointsInit->data.setZero();
  _pandaJointsGet = new KDL::JntArray(NB_PANDA_AXIS);
  _pandaJointsGet->data.setZero();

  _pandaJointLims[L_MIN] = new KDL::JntArray(NB_PANDA_AXIS);
  _pandaJointLims[L_MIN]->data.setZero();
  _pandaJointLims[L_MAX] = new KDL::JntArray(NB_PANDA_AXIS);
  _pandaJointLims[L_MAX]->data.setZero();
 
  _myJacobian.resize(NB_PANDA_AXIS);

  _desiredTargetFrame.M.Identity();
  _desiredTargetFrame.p.Zero();

  _worldPandaLink0Frame.M.Identity();
  _worldPandaLink0Frame.p.Zero();

  _flagToolFrameRetrieved =  false;
  _flagPandaJointInitRetrieved = false;
  _flagPandaLink0FrameRetrieved = false;
  _mySolutionFound=true;

  _flagPandaJointsRetrieved = false;
     
  _msgJointCommands.data.resize(NB_PANDA_AXIS);

    if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
      ROS_ERROR("Failed to construct kdl tree");
      _stop = true;
    }

  KDL::Vector grav_vector(0.0, 0.0, GRAVITY);

  _myTree.getChain(std::string(Panda_Names[_panda_id]) + "_panda_link0", std::string(Panda_Names[_panda_id]) + "_panda_link8", _myPandaBaseToWristChain);


  _myFKSolver_wrist = new KDL::ChainFkSolverPos_recursive(_myPandaBaseToWristChain);
  
  _myJacSolver_wrist = new KDL::ChainJntToJacSolver(_myPandaBaseToWristChain);

  _mySegments = _myPandaBaseToWristChain.segments;

  
  for (unsigned int joint_=0; joint_ < NB_PANDA_AXIS; joint_++ )
  {
    _pandaJointLims[L_MIN]->data(joint_) = _myModel.getJoint(std::string(Panda_Names[_panda_id]) + "_" + std::string(Panda_Axis_Names[joint_]))->limits->lower;
    _pandaJointLims[L_MAX]->data(joint_) = _myModel.getJoint(std::string(Panda_Names[_panda_id]) + "_" + std::string(Panda_Axis_Names[joint_]))->limits->upper;
  }


  
  /****************************** Inverse Kinematics of the panda ***********************************/
      _myVelIKSolver_wrist = new KDL::ChainIkSolverVel_wdls(_myPandaBaseToWristChain);
      _myPosIkSolver_wrist= new KDL::ChainIkSolverPos_NR_JL(_myPandaBaseToWristChain,*me->_pandaJointLims[L_MIN], *me->_pandaJointLims[L_MAX], *_myFKSolver_wrist,*_myVelIKSolver_wrist);
      
      //_myVelIKSolver_wrist->setWeightTS(_weightedTaskSpaceMassMatrix);
      //_myVelIKSolver_wrist->setWeightJS(_weightedJointSpaceMassMatrix);
  /*                                                                                                */
  _tfListener = new tf2_ros::TransformListener(_tfBuffer);

}

pandaController::~pandaController() { me->_n.shutdown(); }

bool pandaController::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{ 
  _pubPandaJointCommands = _n.advertise<std_msgs::Float64MultiArray>("/"+std::string(Panda_Names[_panda_id])+"_panda/joint_position_controller/command", 1);
  _subPandaJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Panda_Names[_panda_id])+"_panda/joint_states"
      , 1, boost::bind(&pandaController::readPandaJoints, this, _1),
      ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      

  // Subscriber definitions
  signal(SIGINT, pandaController::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("The panda controller "
             "is about to start ");
    return true;
  } else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void pandaController::stopNode(int sig) { me->_stop = true; }

void pandaController::run() {
  
  while (!_stop) {
    
      readToolBaseFrame();
      if (_flagToolFrameRetrieved && _flagPandaJointInitRetrieved)
      {  
        ROS_INFO_ONCE("The tool found!");
        performInverseKinematics();
        publishPandaJointCommands();  
       // performChainForwardKinematics();
        _flagToolFrameRetrieved = false;
      }
      else
      {
       // readToolBaseFrame();
       ROS_WARN_ONCE("The tool is not found yet");
      }
    
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("Panda controller stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}




void pandaController::publishPandaJointCommands() {
  for (int k = 0; k < NB_PANDA_AXIS; k++) {
    _msgJointCommands.data[k] = me->_pandaJoints->data[k];
  }
  _pubPandaJointCommands.publish(_msgJointCommands);
}




void pandaController::performInverseKinematics(){

  int ret = _myPosIkSolver_wrist->CartToJnt(*me->_pandaJointsInit,_desiredTargetFrame,*me->_pandaJoints);
    if (ret<0)
    {
        *_pandaJoints=*_pandaJointsInit;
        if (_mySolutionFound)
        {
          ROS_ERROR("No panda IK solutions for the panda found yet... move around to find one");
          _mySolutionFound=false;
        }
    }
    else{
        *_pandaJointsInit = *_pandaJoints;
        if (!_mySolutionFound)
        {
          ROS_INFO("Solutions of panda IK found again!");
          _mySolutionFound=true;
        }
    }

}

// void pandaController::performChainForwardKinematics() 
// {
//   KDL::Frame frame_;
//   _myFrames.clear();
//     //cout<<_pandaJoints->data.transpose()<<endl;
//   for (unsigned int i = 0; i < _mySegments.size(); i++) {
//       _myFKSolver_wrist->JntToCart(*me->_pandaJoints, frame_, i + 1);
//       _myFrames.push_back(frame_); 
//     //cout<<_myFrames[i].p.data[0]<<" "<<_myFrames[i].p.data[1]<<" "<<_myFrames[i].p.data[2]<<endl;
//   }
//   KDL::Frame frame_;
//   _myFrames.clear();
//     //cout<<_pandaJoints->data.transpose()<<endl;
//   for (unsigned int j = 0; j < _mySegments.size(); j++) {
//       _myFKSolver_tip->JntToCart(*me->_pandaJoints, frame_, j + 1);
//       _myFrames.push_back(frame_);
//   }  
//   // publishPandaTipPose();
// }


void pandaController::readToolBaseFrame()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;


  original_frame = std::string(Panda_Names[_panda_id]) + "_panda_link0";


  destination_frame  = std::string(Panda_Names[_panda_id]) + "_tool_main_link";

  geometry_msgs::TransformStamped pandaMainLinkTransform_;

  try {
    pandaMainLinkTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));
    
    
    if (!_flagToolFrameRetrieved) {
      _desiredTargetFrame = tf2::transformToKDL(pandaMainLinkTransform_);
      _flagToolFrameRetrieved = true;
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

void pandaController::readPandaLink0Frame()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;



  original_frame = "world";
  destination_frame  = std::string(Panda_Names[_panda_id]) + "_panda_link0";

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


void pandaController::readPandaJoints(const sensor_msgs::JointState::ConstPtr &msg) {

  if (!_flagPandaJointsRetrieved)
  {
    for (unsigned int i = 0; i < NB_PANDA_AXIS; i++) {
      me->_pandaJointsGet->data[i] = msg->position[i];
    }
    _flagPandaJointsRetrieved=true;
  }
  
  
  if (!_flagPandaJointInitRetrieved) {
    ROS_INFO("Panda tool joints received by the tool");
    for (unsigned int i = 0; i < NB_PANDA_AXIS; i++) {
      me->_pandaJointsInit->data[i] = msg->position[i];
      // cout<<_pandaJointsInit->data[i]<<endl;
    }
    _flagPandaJointInitRetrieved = true;
  }
  

}