#include "legRobot.h"
#include <stdio.h>
#include <stdlib.h>
#include "tf_conversions/tf_eigen.h"


#define ListofLegAxes(enumeration, names) names,
char const *Leg_Axis_Names[]{LEG_AXES};
#undef ListofLegAxes

char const *Leg_Names[]{"none", "right", "left"};

legRobot *legRobot::me = NULL;


legRobot::legRobot(ros::NodeHandle &n_1, double frequency,
                   legRobot::Leg_Name leg_id, urdf::Model model_)
    : _n(n_1), _leg_id(leg_id), _loopRate(frequency), _freq(frequency), _dt(1.0f / frequency),
    _myModel(model_) {
  me = this;
  _stop = false;
  _flagFootPoseConnected = false;
  _legJoints.resize(NB_LEG_AXIS);
  _legJoints.data.setZero();
  _legJointsPrev.resize(NB_LEG_AXIS);
  _legJointsPrev.data.setZero();
  _legJointsInit.resize(NB_LEG_AXIS);
  _legJointsInit.data.setZero();

  _legJointsVel.resize(NB_LEG_AXIS);
  _legJointsVel.data.setZero();

  _legJointsVelPrev.resize(NB_LEG_AXIS);
  _legJointsVelPrev.data.setZero();

  _legJointsAcc.resize(NB_LEG_AXIS);
  _legJointsAcc.data.setZero();
  Eigen::VectorXd filterVelGains((int) NB_LEG_AXIS);
  Eigen::VectorXd filterAccGains((int) NB_LEG_AXIS);
  filterVelGains.setConstant(0.95);
  filterAccGains.setConstant(0.99);
  _legVelFilter.setAlphas(filterVelGains);
  _legAccFilter.setAlphas(filterAccGains);
  _gravityTorques.resize(NB_LEG_AXIS);
  _gravityTorques.data.setZero();
  _coriolisTorques.resize(NB_LEG_AXIS); _coriolisTorques.data.setZero();
  _inertialTorques.resize(NB_LEG_AXIS);_inertialTorques.data.setZero();
  _maxWrench<<15.0,15.0,100.0,2.5,2.5,2.5;
  
  _totalTorques.resize(NB_LEG_AXIS);
  _totalTorques.data.setZero();
  _legJointLims[L_MIN].resize(NB_LEG_AXIS);
  _legJointLims[L_MAX].resize(NB_LEG_AXIS);
  _wrenchGravityFootBase.setZero();
  _gravityVector << 0.0, 0.0, (double)GRAVITY;
  _mySolutionFound=true;
  _netCoG.setZero();
  _supportWrenchEigen.setZero();
  _weightedJointSpaceMassMatrix.setZero();
  _myJacobian.resize(NB_LEG_AXIS);
  _myJointSpaceInertiaMatrix.resize(NB_LEG_AXIS);
  _hipPosFrame.Identity();
  
  _footPosFrame.Identity();

  _flagFootPoseConnected =  false;
  _flagHipPoseConnected =  false;
  
  _flagPlatformJointStateConnected=false;


  _decimationVel=(int) (_freq/200.0); _decimationAcc= (int) (_freq/100.0);  _innerCounterVel=0; _innerCounterAcc=0;
  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("[%s leg]: Failed to construct kdl tree",Leg_Names[_leg_id]);
    _stop=true;
  }

  KDL::Vector grav_vector(0.0, 0.0, (double)GRAVITY);

  _myTree.getChain(std::string(Leg_Names[_leg_id]) + "_leg_hip_base_link", std::string(Leg_Names[_leg_id]) + "_leg_foot_base", _myFootBaseChain);

  _myChainDyn = new KDL::ChainDynParam(_myFootBaseChain, grav_vector);

  _myFKSolver = new KDL::ChainFkSolverPos_recursive(_myFootBaseChain);

  _myJacSolver = new KDL::ChainJntToJacSolver(_myFootBaseChain);

  _mySegments = _myFootBaseChain.segments;

  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myArticulatedBodyInertias.push_back(KDL::ArticulatedBodyInertia(_mySegments[i].getInertia()));
  }

  for (int joint_=0; joint_<NB_LEG_AXIS; joint_++ )
  {
    _legJointLims[L_MIN].data(joint_) = _myModel.getJoint(std::string(Leg_Names[_leg_id]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->lower;
    _legJointLims[L_MAX].data(joint_) = _myModel.getJoint(std::string(Leg_Names[_leg_id]) + "_leg_" + std::string(Leg_Axis_Names[joint_]))->limits->upper;
  }
  _myVelIKSolver = new KDL::ChainIkSolverVel_wdls(_myFootBaseChain);

  _myTorqueFDSolver = new KDL::ChainFdSolverTorque_wdls(_myFootBaseChain);

  _myPosIkSolver = new KDL::ChainIkSolverPos_NR_JL(_myFootBaseChain,me->_legJointLims[L_MIN], me->_legJointLims[L_MAX], *_myFKSolver,*_myVelIKSolver);

  _tfListener = new tf2_ros::TransformListener(_tfBuffer);
}

legRobot::~legRobot() { me->_n.shutdown(); }

bool legRobot::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{
  
  _pubLegJointStates = _n.advertise<sensor_msgs::JointState>("joint_states", 1);
  _pubFootBaseWrench = _n.advertise<geometry_msgs::WrenchStamped>("leg_foot_base_wrench", 1);
  _pubNetCoG = _n.advertise<geometry_msgs::PointStamped>("leg_cog",1);
  _pubManipEllipsoidRot = _n.advertise<visualization_msgs::Marker>("leg_manipulability_rot", 0);
  _pubManipEllipsoidLin = _n.advertise<visualization_msgs::Marker>("leg_manipulability_lin", 0);

  _subPlatformJointState = _n.subscribe<sensor_msgs::JointState>("/" + std::string(Leg_Names[_leg_id]) + "_platform/platform_joint_publisher/joint_states",1,readPlatformJointState);
  // Subscriber definitions
  signal(SIGINT, legRobot::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("[%s leg]: The leg joint state publisher is about to start ",Leg_Names[_leg_id]);
    return true;
  } else {
    ROS_ERROR("[%s leg]: The ros node has a problem.",Leg_Names[_leg_id]);
    return false;
  }
}

void legRobot::stopNode(int sig) { me->_stop = true;  me->publishFootBaseGravityWrench(); me->publishLegJointStates(); }

void legRobot::run() {
  
  while (!_stop) {
    if (_flagPlatformJointStateConnected)
    {
      ROS_INFO_ONCE("[%s leg]: the platform joint state can be read now",Leg_Names[_leg_id]);
      readFootBasePose();
      readHipBasePose();
      if (_flagFootPoseConnected) {
        computedWeightingMatrixes();
        performInverseKinematics();
        publishLegJointStates();
        computeIDTorque();
        performChainForwardKinematics();
        computeNetCoG();
        computeFootBaseWrenchForwardDynamics();
        computeLegManipulability();
        publishManipulabilityEllipsoidRot();
        publishManipulabilityEllipsoidLin();
        _innerCounterVel++;
        _innerCounterAcc++;
    }
    }
      ros::spinOnce();
      _loopRate.sleep();
  }

  ROS_INFO("[%s leg]: Leg state variables stopped",Leg_Names[_leg_id]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void legRobot::publishLegJointStates() {
  // _mutex.lock();
  if(!_stop)
  {
    _msgJointStates.header.stamp = ros::Time::now();
    _msgJointStates.name.resize(NB_LEG_AXIS);
    _msgJointStates.position.resize(NB_LEG_AXIS);
    _msgJointStates.velocity.resize(NB_LEG_AXIS);
    _msgJointStates.effort.resize(NB_LEG_AXIS);

    for (int k = 0; k < NB_LEG_AXIS; k++) {
      _msgJointStates.name[k] = std::string(Leg_Names[_leg_id]) + "_leg_" + std::string(Leg_Axis_Names[k]);
      _msgJointStates.position[k] = me->_legJoints.data[k];
      _msgJointStates.velocity[k] = me->_legJointsVel.data[k];
      _msgJointStates.effort[k] = me->_totalTorques.data[k];
    }
  }else
  {
    _msgJointStates.header.stamp = ros::Time::now();
    _msgJointStates.name.resize(NB_LEG_AXIS);
    _msgJointStates.position.resize(NB_LEG_AXIS);
    _msgJointStates.velocity.resize(NB_LEG_AXIS);
    _msgJointStates.effort.resize(NB_LEG_AXIS);

    for (int k = 0; k < NB_LEG_AXIS; k++) {
      _msgJointStates.name[k] = std::string(Leg_Names[_leg_id]) + "_leg_" + std::string(Leg_Axis_Names[k]);
      _msgJointStates.position[k] = me->_legJoints.data[k];
      _msgJointStates.velocity[k] = 0.0;
      _msgJointStates.effort[k] = 0.0;
    }
  }
  _pubLegJointStates.publish(_msgJointStates);
  // _mutex.unlock();
}



void legRobot::readFootBasePose()
{
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;

  if (_leg_id==Leg_Name::LEFT)
  {
    destination_frame = "left_platform_foot_rest";
    original_frame = "left_leg_hip_base_link";
  }
  else {
    destination_frame = "right_platform_foot_rest";
    original_frame = "right_leg_hip_base_link";
  }

  geometry_msgs::TransformStamped footPoseTransform_;

  try {
    footPoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

    _footPosFrame = tf2::transformToKDL(footPoseTransform_);
    _flagFootPoseConnected = true;
    
  } catch (tf2::TransformException ex) {
    if (count>2)
    { ROS_ERROR("[%s leg]: %s",Leg_Names[_leg_id], ex.what());
      count = 0;
    }
    else
    {
      count++;
    }
    ros::Duration(1.0).sleep();
  }
}

void legRobot::readHipBasePose() {
  static int count = 0;
  std::string original_frame;
  std::string destination_frame;

  if (_leg_id == Leg_Name::LEFT) {
    destination_frame = "left_leg_hip_base_link";
    original_frame = "left_platform_base_link";
  } else {
    destination_frame = "right_leg_hip_base_link";
    original_frame = "right_platform_base_link";
  }

  geometry_msgs::TransformStamped hipPoseTransform_;

  try {
    hipPoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

    _hipPosFrame = tf2::transformToKDL(hipPoseTransform_);
    _flagHipPoseConnected = true;

  } catch (tf2::TransformException ex) {
    if (count > 2) {
      ROS_ERROR("[%s leg]: %s",Leg_Names[_leg_id], ex.what());
      count = 0;
    } else {
      count++;
    }
    ros::Duration(1.0).sleep();
  }
}


void legRobot::performInverseKinematics(){
  if (_innerCounterAcc>_decimationAcc)
  {
    _legJointsAcc.data = _legAccFilter.update((_legJointsVel.data - _legJointsVelPrev.data) * _freq);
    _innerCounterAcc=0;
    _legJointsVelPrev.data = _legJointsVel.data;
  }
  if (_innerCounterVel>_decimationVel)
  {
    _legJointsVel.data = _legVelFilter.update((_legJoints.data - _legJointsPrev.data) * _freq);
    _innerCounterVel=0;
  }
  _legJointsPrev.data = _legJoints.data;
  
  int ret = _myPosIkSolver->CartToJnt(me->_legJointsInit,_footPosFrame,me->_legJoints);
  _legJointsInit.data = _legJoints.data;
  if (ret<0)
  {   
    _legJoints.data = _legJointsPrev.data;
    _legJointsInit.data = _legJointsPrev.data;
    if(_mySolutionFound)
    {
      ROS_ERROR("[%s leg]: No leg IK solutions found yet... move around to find one",Leg_Names[_leg_id]);
      _mySolutionFound=false;
    }
  }
  else{
    if(!_mySolutionFound)
    {
      ROS_INFO("[%s leg]: Solutions of leg IK found again!",Leg_Names[_leg_id]);
      _mySolutionFound=true;
    }
  }
}

//! For IK FAST 61
// void legRobot::processAngles(Eigen::MatrixXd ikSolutions_) {

//   Eigen::MatrixXd::Index maxIndex[2];
//   Eigen::VectorXd leg_joints_temp((int) NB_LEG_AXIS,1);
  
//   int NB_JOINTS_TO_PROCESS = (int)NB_LEG_AXIS-3; // All except the last three

//   for (int i=0; i<2; i++)
//   {
//     ikSolutions_.row(hip_extension).maxCoeff(&maxIndex[i]); //! Takes the solution that involves the thigh not colliding with the chain
//     leg_joints_temp = ikSolutions_.col(maxIndex[i]);

//     if ((leg_joints_temp.segment(0,NB_JOINTS_TO_PROCESS).array().abs() > 100 * DEG_TO_RAD).any()==0)
//     {
//       _legJoints.data = leg_joints_temp;
//     } 
    
//   }
//    leg_joints_temp = _legJoints.data;
// }

void legRobot::computeIDTorque() {
  _myChainDyn->JntToGravity(_legJoints,_gravityTorques);
  _myChainDyn->JntToCoriolis(_legJoints,_legJointsVel,_coriolisTorques);
  KDL::JntArray temp_torque;
  
  temp_torque.resize(NB_LEG_AXIS); temp_torque.data.setZero();
  Add(_gravityTorques,_coriolisTorques,temp_torque);
  Add(temp_torque,_inertialTorques,_totalTorques);
  
 // cout<<_inertialTorques.data.transpose()<<endl;
}

void legRobot::computeNetCoG(){

  //! Calculate the net center of gravity of the leg
  Eigen::Vector3d cogLink = Eigen::Vector3d::Zero();
  _netCoG.setZero();
  double totalmass = 0.0;
  double linkmass = 0.0;
  
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
   // _myFKSolver->JntToCart(me->_legJoints, frame_, i+1);
    tf::vectorKDLToEigen(_hipPosFrame * _myFrames[i] * _mySegments[i].getInertia().getCOG(), cogLink);
    
    linkmass = _mySegments[i].getInertia().getMass();
    _netCoG += cogLink * linkmass;
    totalmass += linkmass; 
    //cout << _mySegments[i].getName()<<":"<< cogLink.transpose() <<" mass:"<< linkmass <<  endl;
  }
  _netCoG /= totalmass;
  publishNetCoG();

}

void legRobot::publishNetCoG() {
  //! Keep send the same valuest that the leg is broadcasting
  // _mutex.lock();
  std::string frame_name;
   frame_name = _leg_id==RIGHT ? "right_platform_base_link" : "left_platform_base_link"; 
  _msgNetCoG.header.stamp = ros::Time::now();
  _msgNetCoG.header.frame_id = frame_name; 
  _msgNetCoG.point.x = _netCoG(0);
  _msgNetCoG.point.y = _netCoG(1);
  _msgNetCoG.point.z = _netCoG(2);

  _pubNetCoG.publish(_msgNetCoG);
  // _mutex.unlock();
}

void legRobot::computeFootBaseWrenchForwardDynamics() {
  // It computes the forward dynamics of the gravity compensation torques of the
  // leg towards the foot(end effector)
  // WrenchOfGravity_footbase = J^-T * torques_of_gravity_compensation_in_leg
  // This wrench will be "replicated" by the foot platform to alleviate the
  // gravity compensation.
  _myTorqueFDSolver->JntToCart(_legJoints, _totalTorques,
                               _supportWrenchEigen);
  _supportWrenchEigen =  _supportWrenchEigen.cwiseMax(-_maxWrench).cwiseMin(_maxWrench);
  
  publishFootBaseGravityWrench();
}

void legRobot::publishFootBaseGravityWrench() {
  //! Keep send the same valuest that the leg is broadcasting
  // _mutex.lock();
  if(!_stop)
  {
    std::string frame_name;
    frame_name = _leg_id == RIGHT ? "right_leg_foot_base"
                                  : "left_leg_foot_base";
    _msgFootBaseWrench.header.stamp = ros::Time::now();
    _msgFootBaseWrench.header.frame_id = frame_name;
    _msgFootBaseWrench.wrench.force.x = _supportWrenchEigen(0);
    _msgFootBaseWrench.wrench.force.y = _supportWrenchEigen(1);
    _msgFootBaseWrench.wrench.force.z = _supportWrenchEigen(2);
    _msgFootBaseWrench.wrench.torque.x = _supportWrenchEigen(3);
    _msgFootBaseWrench.wrench.torque.y = _supportWrenchEigen(4);
    _msgFootBaseWrench.wrench.torque.z = _supportWrenchEigen(5);
  }else

  {
    std::string frame_name;
    frame_name = _leg_id == RIGHT ? "right_leg_foot_base"
                                  : "left_leg_foot_base";
    _msgFootBaseWrench.header.stamp = ros::Time::now();
    _msgFootBaseWrench.header.frame_id = frame_name;
    _msgFootBaseWrench.wrench.force.x = 0.0f;
    _msgFootBaseWrench.wrench.force.y = 0.0f;
    _msgFootBaseWrench.wrench.force.z = 0.0f;
    _msgFootBaseWrench.wrench.torque.x = 0.0f;
    _msgFootBaseWrench.wrench.torque.y = 0.0f;
    _msgFootBaseWrench.wrench.torque.z = 0.0f;
  }

  _pubFootBaseWrench.publish(_msgFootBaseWrench);
  // _mutex.unlock();
}



void legRobot::computedWeightingMatrixes()
{
  _myChainDyn->JntToMass(_legJoints, _myJointSpaceInertiaMatrix);
  Multiply(_myJointSpaceInertiaMatrix,_legJointsAcc,_inertialTorques);
  _weightedJointSpaceMassMatrix = _myJointSpaceInertiaMatrix.data;
  //_myVelIKSolver->setWeightJS(_weightedJointSpaceMassMatrix.normalized());
  //_myTorqueFDSolver->setWeightJS(_weightedJointSpaceMassMatrix.normalized());
}

void legRobot::computeLegManipulability()
{
  //_myTorqueFDSolver->U;
  _myJacSolver->JntToJac(_legJoints,_myJacobian);
  Matrix<double, NB_AXIS_WRENCH, NB_LEG_AXIS> weightedJacobian = _myJacobian.data*_weightedJointSpaceMassMatrix.inverse();
  //_mySVD.compute(_myJacobian.data*_weightedJointSpaceMassMatrix.inverse(),ComputeThinU | ComputeThinV);
  _mySVD.compute(weightedJacobian.normalized(), ComputeThinU | ComputeThinV);
  //cout<<_mySVD.matrixU()<<endl;
  // cout<<"Singular Values: "<<_mySVD.singularValues().transpose()<<endl;
  // cout<<"Leg Yoshikawa Manipulability: "<<_mySVD.singularValues().prod()<<endl;
  // cout<<"Leg Jacobian Condition Number: "<<_mySVD.singularValues().minCoeff()/_mySVD.singularValues().maxCoeff() << endl;
}

void legRobot::performChainForwardKinematics() 
{
  KDL::Frame frame_;
  _myFrames.clear();
  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myFKSolver->JntToCart(me->_legJoints, frame_, i + 1);
    _myFrames.push_back(frame_);
  }
}

// void legRobot::processArticulatedBodyInertias()
// {
//   _myTotalArticulatedBodyInertia.Zero();
//   for (unsigned int i = 0; i < _mySegments.size(); i++) {
//     cout << _myArticulatedBodyInertias[i].M << endl;
//     KDL::ArticulatedBodyInertia temp_ =(_myFrames[i]).M * _myArticulatedBodyInertias[i];
//     // _myTotalArticulatedBodyInertia = _myTotalArticulatedBodyInertia + (temp_).RefPoint(
//     // _myFrames[_mySegments.size()].p - _myFrames[i].p);
//   } 
// }

void legRobot::publishManipulabilityEllipsoidRot() {
  // _mutex.lock();
  VectorXd svdValues = _mySVD.singularValues();
  Matrix<double, 3,3> svdVectors = _mySVD.matrixU().block(3,3,3,3);
  Quaternion<double> svdSingQuaternion(svdVectors);
  std::string frame_name;
  std::string ns_name;
  frame_name = _leg_id == RIGHT ? "right_leg_hip_base_link" : "left_leg_hip_base_link";
  // ns_name = _leg_id == RIGHT ? "right" : "left";
  _msgManipEllipsoidRot.header.frame_id = frame_name;
  _msgManipEllipsoidRot.header.stamp = ros::Time::now();
  _msgManipEllipsoidRot.ns = ns_name;
  _msgManipEllipsoidRot.id = 0;
  _msgManipEllipsoidRot.type = visualization_msgs::Marker::SPHERE;
  _msgManipEllipsoidRot.action = visualization_msgs::Marker::ADD;
  _msgManipEllipsoidRot.pose.position.x = _footPosFrame.p.x();
  _msgManipEllipsoidRot.pose.position.y = _footPosFrame.p.y();
  _msgManipEllipsoidRot.pose.position.z = _footPosFrame.p.z();
  _msgManipEllipsoidRot.pose.orientation.x = svdSingQuaternion.x();
  _msgManipEllipsoidRot.pose.orientation.y = svdSingQuaternion.y();
  _msgManipEllipsoidRot.pose.orientation.z = svdSingQuaternion.z();
  _msgManipEllipsoidRot.pose.orientation.w = svdSingQuaternion.w();
  _msgManipEllipsoidRot.scale.x = svdValues(3) * 1.0;
  _msgManipEllipsoidRot.scale.y = svdValues(4) * 1.0;
  _msgManipEllipsoidRot.scale.z = svdValues(5) * 1.0;
  _msgManipEllipsoidRot.color.a = 0.5; // Don't forget to set the alpha!
  _msgManipEllipsoidRot.color.r = 0;
  _msgManipEllipsoidRot.color.g = 1;
  _msgManipEllipsoidRot.color.b = 0;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubManipEllipsoidRot.publish(_msgManipEllipsoidRot);
  // _mutex.unlock();
}

void legRobot::publishManipulabilityEllipsoidLin() {
  // _mutex.lock();
  VectorXd svdValues = _mySVD.singularValues();
  Matrix<double, 3,3> svdVectors = _mySVD.matrixU().block(0,0,3,3);
  Quaternion<double> svdSingQuaternion(svdVectors);
  std::string frame_name;
  std::string ns_name;
  frame_name = _leg_id == RIGHT ? "right_leg_hip_base_link" : "left_leg_hip_base_link";
  // ns_name = _leg_id == RIGHT ? "right" : "left";
  _msgManipEllipsoidLin.header.frame_id = frame_name;
  _msgManipEllipsoidLin.header.stamp = ros::Time::now();
  _msgManipEllipsoidLin.ns = ns_name;
  _msgManipEllipsoidLin.id = 0;
  _msgManipEllipsoidLin.type = visualization_msgs::Marker::SPHERE;
  _msgManipEllipsoidLin.action = visualization_msgs::Marker::ADD;
  _msgManipEllipsoidLin.pose.position.x = _footPosFrame.p.x();
  _msgManipEllipsoidLin.pose.position.y = _footPosFrame.p.y();
  _msgManipEllipsoidLin.pose.position.z = _footPosFrame.p.z();
  _msgManipEllipsoidLin.pose.orientation.x = svdSingQuaternion.x();
  _msgManipEllipsoidLin.pose.orientation.y = svdSingQuaternion.y();
  _msgManipEllipsoidLin.pose.orientation.z = svdSingQuaternion.z();
  _msgManipEllipsoidLin.pose.orientation.w = svdSingQuaternion.w();
  _msgManipEllipsoidLin.scale.x = svdValues(0);
  _msgManipEllipsoidLin.scale.y = svdValues(1);
  _msgManipEllipsoidLin.scale.z = svdValues(2);
  _msgManipEllipsoidLin.color.a = 0.5; // Don't forget to set the alpha!
  _msgManipEllipsoidLin.color.r = 0;
  _msgManipEllipsoidLin.color.g = 1;
  _msgManipEllipsoidLin.color.b = 1;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  _pubManipEllipsoidLin.publish(_msgManipEllipsoidLin);
  // _mutex.unlock();
}


void legRobot::readPlatformJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    me->_msgPlatformJointState = *msg;
    me->_flagPlatformJointStateConnected=true;
}

