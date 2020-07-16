#include "legRobot.h"
#include <stdio.h>
#include <stdlib.h>
#include "tf_conversions/tf_eigen.h"
#define IK_VERSION 61
#include "ikfast61Solver.cpp"


using namespace ikfast;

#define ListofLegAxes(enumeration, names) names,
char const *Leg_Axis_Names[]{LEG_AXES};
#undef ListofLegAxes

char const *Leg_Names[]{"none", "right", "left"};

legRobot *legRobot::me = NULL;

legRobot::legRobot(ros::NodeHandle &n_1, double frequency, legRobot::Leg_Name leg_id, urdf::Model model_)
    : _n(n_1), _leg_id(leg_id), _loopRate(frequency),
      _dt(1.0f / frequency), _myModel(model_) {
   me = this;
  _stop = false;
  _flagFootPoseConnected = false;
  _legJoints = new KDL::JntArray(NB_LEG_AXIS);
  _legJoints->data.setZero();
  _legJointsInit = new KDL::JntArray(NB_LEG_AXIS);
  _legJointsInit->data.setZero();
  _gravityTorques = new KDL::JntArray(NB_LEG_AXIS);
  _legJointLims[L_MIN] = new KDL::JntArray(NB_LEG_AXIS);
  _legJointLims[L_MAX] = new KDL::JntArray(NB_LEG_AXIS);
  _gravityTorques->data.setZero();
  _wrenchGravityFootBase.setZero();
  _gravityVector << 0.0, 0.0, (double)GRAVITY;
  _mySolutionFound=true;
  _netCoG.setZero();

  if (!kdl_parser::treeFromUrdfModel(_myModel, _myTree)) {
    ROS_ERROR("Failed to construct kdl tree");
    _stop=true;
  }

  KDL::Vector grav_vector(0.0, 0.0, (double)GRAVITY);

  _myTree.getChain("virtual_platform_base_link", "foot_base", _myFootBaseChain);

  _myChainDyn = new KDL::ChainDynParam(_myFootBaseChain, grav_vector);

  _myFKSolver = new KDL::ChainFkSolverPos_recursive(_myFootBaseChain);

  _mySegments = _myFootBaseChain.segments;
  

  for (int joint_=0; joint_<NB_LEG_AXIS; joint_++ )
  {
    _legJointLims[L_MIN]->data(joint_) = _myModel.getJoint(Leg_Axis_Names[joint_])->limits->lower;
    _legJointLims[L_MAX]->data(joint_) = _myModel.getJoint(Leg_Axis_Names[joint_])->limits->upper;
  }
  _myVelIKSolver = new KDL::ChainIkSolverVel_wdls(_myFootBaseChain);

  _myPosIkSolver = new KDL::ChainIkSolverPos_NR_JL(_myFootBaseChain,*me->_legJointLims[L_MIN], *me->_legJointLims[L_MAX], *_myFKSolver,*_myVelIKSolver);

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

  // Subscriber definitions
  signal(SIGINT, legRobot::stopNode);

  if (_n.ok()) {
    ros::spinOnce();
    ROS_INFO("The leg joint state publisher "
             "is about to start ");
    return true;
  } else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void legRobot::stopNode(int sig) { me->_stop = true; }

void legRobot::run() {
  
  while (!_stop) {
    readFootBasePose();
    if (_flagFootPoseConnected) {
      performInverseKinematics();
      publishLegJointStates();
      computeGravityTorque();
      computeFootBaseGravityWrench();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("Leg state variables stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void legRobot::publishLegJointStates() {
  // _mutex.lock();

  _msgJointStates.header.stamp = ros::Time::now();

  _msgJointStates.name.resize(NB_LEG_AXIS);
  _msgJointStates.position.resize(NB_LEG_AXIS);
  _msgJointStates.velocity.resize(NB_LEG_AXIS);
  _msgJointStates.effort.resize(NB_LEG_AXIS);

  for (int k = 0; k < NB_LEG_AXIS; k++) {
    _msgJointStates.name[k] = Leg_Axis_Names[k];
    _msgJointStates.position[k] = me->_legJoints->data[k];
    _msgJointStates.velocity[k] = 0.0f;
    _msgJointStates.effort[k] = 0.0f;
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
    destination_frame = "left/foot_rest";
    original_frame = "left/virtual_platform_base_link";
  }
  else {
    destination_frame = "right/foot_rest";
    original_frame = "right/virtual_platform_base_link";
  }

  geometry_msgs::TransformStamped footPoseTransform_;

  try {
    footPoseTransform_ = _tfBuffer.lookupTransform(
        original_frame.c_str(), destination_frame.c_str(), ros::Time(0));

    _footPosFrame = tf2::transformToKDL(footPoseTransform_);
    _flagFootPoseConnected = true;
    
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

void legRobot::performInverseKinematics(){
  int ret = _myPosIkSolver->CartToJnt(*me->_legJointsInit,_footPosFrame,*me->_legJoints);
  *_legJointsInit = *_legJoints;
  if (ret<0)
  {
    if (_mySolutionFound) 
    {
      ROS_ERROR("No leg IK solutions found yet... move around to find one");
      _mySolutionFound=false;
    }
  }
  else{
    if(!_mySolutionFound)
    {
      ROS_INFO("Solutions of leg IK found again!");
      _mySolutionFound=true;
    }
  }
}


void legRobot::processAngles(Eigen::MatrixXd ikSolutions_) {

  Eigen::MatrixXd::Index maxIndex[2];
  Eigen::VectorXd leg_joints_temp((int) NB_LEG_AXIS,1);
  
  int NB_JOINTS_TO_PROCESS = (int)NB_LEG_AXIS-3; // All except the last three

  for (int i=0; i<2; i++)
  {
    ikSolutions_.row(hip_extension).maxCoeff(&maxIndex[i]); //! Takes the solution that involves the thigh not colliding with the chain
    leg_joints_temp = ikSolutions_.col(maxIndex[i]);

    if ((leg_joints_temp.segment(0,NB_JOINTS_TO_PROCESS).array().abs() > 100 * DEG_TO_RAD).any()==0)
    {
      _legJoints->data = leg_joints_temp;
    } 
    
  }
   leg_joints_temp = _legJoints->data;
}

void legRobot::computeGravityTorque() {
  _myChainDyn->JntToGravity(*_legJoints,*_gravityTorques);
  //cout<<_gravityTorques->data.transpose()<<endl;
}

void legRobot::computeFootBaseGravityWrench(){

  //! Calculate the net center of gravity of the leg
  //! Make a system of two equations to know the
  Eigen::Vector3d cogLink = Eigen::Vector3d::Zero();
  KDL::Frame frame_;
  _netCoG.setZero();
  double totalmass = 0.0;
  double linkmass = 0.0;

  for (unsigned int i = 0; i < _mySegments.size(); i++) {
    _myFKSolver->JntToCart(*me->_legJoints, frame_, i+1);
    tf::vectorKDLToEigen(frame_ * _mySegments[i].getInertia().getCOG(), cogLink);
    linkmass = _mySegments[i].getInertia().getMass();
    _netCoG += cogLink * linkmass;
    _myFrames.push_back(frame_);
    totalmass += linkmass;
    //cout << _mySegments[i].getName()<<":"<< cogLink.transpose() <<" mass:"<< linkmass <<  endl;
  }
  _netCoG /= totalmass;
  publishNetCoG();
  Eigen::Vector3d totalWeight = totalmass * _gravityVector;
  _wrenchGravityFootBase.segment(0,3) = totalWeight;
  _wrenchGravityFootBase.segment(3,3) = _netCoG.cross(totalWeight);
  publishFootBaseGravityWrench();
}

void legRobot::publishNetCoG() {
  //! Keep send the same valuest that the leg is broadcasting
  // _mutex.lock();
  std::string frame_name;
  frame_name = _leg_id==RIGHT ? "/right/virtual_platform_base_link" : "/left/virtual_platform_base_link"; 
  _msgNetCoG.header.stamp = ros::Time::now();
  _msgNetCoG.header.frame_id = frame_name; 
  _msgNetCoG.point.x = _netCoG(0);
  _msgNetCoG.point.y = _netCoG(1);
  _msgNetCoG.point.z = _netCoG(2);

  _pubNetCoG.publish(_msgNetCoG);
  // _mutex.unlock();
}

void legRobot::publishFootBaseGravityWrench() {
  //! Keep send the same valuest that the leg is broadcasting
  // _mutex.lock();
  std::string frame_name;
  frame_name = _leg_id == RIGHT ? "/right/virtual_platform_base_link"
                                : "/left/virtual_platform_base_link";
  _msgFootBaseWrench.header.stamp = ros::Time::now();
  _msgFootBaseWrench.header.frame_id = frame_name;
  _msgFootBaseWrench.wrench.force.x = _wrenchGravityFootBase(0);
  _msgFootBaseWrench.wrench.force.y = _wrenchGravityFootBase(1);
  _msgFootBaseWrench.wrench.force.z = _wrenchGravityFootBase(2);
  _msgFootBaseWrench.wrench.torque.x = _wrenchGravityFootBase(3);
  _msgFootBaseWrench.wrench.torque.y = _wrenchGravityFootBase(4);
  _msgFootBaseWrench.wrench.torque.z = _wrenchGravityFootBase(5);
  _pubFootBaseWrench.publish(_msgFootBaseWrench);
  // _mutex.unlock();
}
