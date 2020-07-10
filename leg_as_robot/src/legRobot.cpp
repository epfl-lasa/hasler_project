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

legRobot::legRobot(ros::NodeHandle &n_1, double frequency, legRobot::Leg_Name leg_id)
    : _n(n_1), _leg_id(leg_id), _loopRate(frequency),
      _dt(1.0f / frequency) {
   me = this;
  _stop = false;
  _flagFootPoseConnected = false;
  _leg_joints.setZero();
  _footPosition.setZero();
  _footQuaternion.setIdentity();
  _footRotationMatrix.setIdentity();
}

legRobot::~legRobot() { me->_n.shutdown(); }

bool legRobot::init() //! Initialization of the node. Its datatype
                                //! (bool) reflect the success in
                                //! initialization
{
  
  _pubLegJointStates = _n.advertise<sensor_msgs::JointState>("joint_states", 1);
 

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
    readFootPose();
    if (_flagFootPoseConnected) {
      // cout<<_footPosition<<endl;
      publishFootJointStates();
      performInverseKinematics();
    }
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("Leg state variables stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void legRobot::publishFootJointStates() {
  //! Keep send the same valuest that the leg is broadcasting
  // _mutex.lock();

  _msgJointStates.header.stamp = ros::Time::now();

  _msgJointStates.name.resize(NB_LEG_AXIS);
  _msgJointStates.position.resize(NB_LEG_AXIS);
  _msgJointStates.velocity.resize(NB_LEG_AXIS);
  _msgJointStates.effort.resize(NB_LEG_AXIS);

  for (int k = 0; k < NB_LEG_AXIS; k++) {
    _msgJointStates.name[k] = Leg_Axis_Names[k];
    _msgJointStates.position[k] = _leg_joints[k];
    _msgJointStates.velocity[k] = 0.0f;
    _msgJointStates.effort[k] = 0.0f;
  }
  _pubLegJointStates.publish(_msgJointStates);
  // _mutex.unlock();
}

void legRobot::readFootPose()
{
  std::string original_frame;
  std::string destination_frame;

  if (_leg_id==Leg_Name::LEFT)
  {
    destination_frame = "/left/foot_rest";
    original_frame = "/left/platform_base_link";
  }
  else {
    destination_frame = "/right/foot_rest";
    original_frame = "/right/platform_base_link";
  }

  tf::StampedTransform footPoseTransform_;

  try {
    _footPoseListener.lookupTransform(original_frame.c_str(), destination_frame.c_str(),
                                      ros::Time(0), footPoseTransform_);

    tf::vectorTFToEigen (footPoseTransform_.getOrigin(),_footPosition);
    tf::quaternionTFToEigen (footPoseTransform_.getRotation(),_footQuaternion);
    tf::matrixTFToEigen(footPoseTransform_.getBasis(), _footRotationMatrix);
    
    _flagFootPoseConnected = true;
    
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void legRobot::performInverseKinematics() {

  // Usage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0
  IkSolutionList<IkReal> solutions;
  IkReal eerot[9], eetrans[3];
  std::vector<IkReal> vfree(1);

  eerot[0] = _footRotationMatrix(0,0);
  eerot[1] = _footRotationMatrix(0,1);
  eerot[2] = _footRotationMatrix(0,2);
  eetrans[0] = _footPosition(0);
  eerot[3] = _footRotationMatrix(1,0);
  eerot[4] = _footRotationMatrix(1,1);
  eerot[5] = _footRotationMatrix(1,2);
  eetrans[1] = _footPosition(1);
  eerot[6] =  _footRotationMatrix(2,0);
  eerot[7] =  _footRotationMatrix(2,1);
  eerot[8] =  _footRotationMatrix(2,2);
  eetrans[2] = _footPosition(2);

  vfree[0] = 4;
  
  bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  if (!bSuccess) {
    cerr<<"Failed to get ik solution"<<endl;
  }

  printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
  std::vector<double> solvalues(GetNumJoints());
  for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
    const IkSolutionBase<double> &sol = solutions.GetSolution(i);
    printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
    std::vector<double> vsolfree(sol.GetFree().size());
    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
    for (std::size_t j = 0; j < solvalues.size(); ++j)
      printf("%.15f, ", solvalues[j]);
    printf("\n");
  }
}