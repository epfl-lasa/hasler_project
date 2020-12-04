#include "SurgicalTask.h"
#include "Utils.h"
#include <chrono>
#include <thread>
#include "gazebo_msgs/SetModelState.h"
#include "qpSolver.h"

SurgicalTask* SurgicalTask::me = NULL;


SurgicalTask::SurgicalTask(ros::NodeHandle &n, double frequency):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{
  me = this;
  
  _sphericalTrocarId[LEFT] = 14;
  _sphericalTrocarId[RIGHT] = 23;
  _usePredefinedTrocars = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolMass[LEFT] = 0.95f;
  _toolComPositionFromSensor[LEFT] << 0.0f,0.0f,0.1f;
  _toolMass[RIGHT] = 0.95f;
  _toolComPositionFromSensor[RIGHT] << 0.0f,0.0f,0.1f;


  for(int r = 0; r < NB_ROBOTS; r++)
  {
    _x[r].setConstant(0.0f);
    _q[r].setConstant(0.0f);
    
    _xd[r].setConstant(0.0f);
    _vd[r].setConstant(0.0f);
    _vdTool[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _qd[r].setConstant(0.0f);
    _qdPrev[r].setConstant(0.0f);
    
    _xdOffset[r].setConstant(0.0f);
    _joyAxes[r].setConstant(0.0f);
    _selfRotationCommand[r] = 0.0f;

    _footPose[r].setConstant(0.0f);
    _footPoseFiltered[r].setConstant(0.0f);
    _footWrench[r].setConstant(0.0f);
    _footTwist[r].setConstant(0.0f);
    _trocarInput[r].setConstant(0.0f);
    _vdOffset[r].setConstant(0.0f);
    _sequenceID[r] = 0;
    _joystickSequenceID[r] = 100;
    _desiredFootWrench[r].setConstant(0.0f);
    _xRobotBaseOrigin[r].setConstant(0.0f);
    _qRobotBaseOrigin[r] << 1.0f, 0.0f, 0.0f, 0.0f;
    _D[r].setConstant(0.0f);

    _robotMode[r] = TROCAR_INSERTION;
    _ikJoints[r].resize(7);
    _currentJoints[r].resize(7);

    _trocarPosition[r].setConstant(0.0f);
    _wrenchCount[r] = 0;
    _filterGainFootAxis[r].setConstant(1.0f);
    _footOffset[r].setConstant(0.0f);
    _desiredOffset[r].setConstant(0.0f);
    _desiredGripperPosition[r] = 0.0f;
    _dRCMTool[r] = 0.0f;

    _stiffness[r] = 0.0f;
    _selfRotationCommand[r] = 0.0f;
    _firstRobotPose[r] = false;
    _firstRobotBaseFrame[r] = false;
    _firstJointsUpdate[r] = false;
    _firstRobotTwist[r] = false;
    _firstFootSharedGrasping[r] = false;
    _firstHumanInput[r] = false;
    _firstSphericalTrocarFrame[r] = false;
    _alignedWithTrocar[r] = false;
    _trocarsRegistered[r] = false;
    _firstDampingMatrix[r] = false;
    _firstPublish[r] = false;
    _wRRobotBasis[r].setIdentity();
  }
  _stop = false;
  _firstGripper = false;
  _optitrackOK = false;
  _usePredefinedTrocars = false;
  _useTaskAdaptation = false;


  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);
  _optitrackCount = 0;
  _optitrackInitialized = false;

  _Roptitrack << 1.0f, 0.0f, 0.0f,
                 0.0f, 1.0f, 0.0f,
                 0.0f, 0.0f, 1.0f;  


   _Rcamera << std::cos(-M_PI/4.0f), -std::sin(-M_PI/4.0f), 0.0f,
               std::sin(-M_PI/4.0f), std::cos(-M_PI/4.0f), 0.0f,
               0.0f, 0.0f, 1.0f;
  
  _Rcamera.setIdentity();

  _filteredForceGain = 0.9f;
}


bool SurgicalTask::init() 
{
  _useRobot.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/useRobot", _useRobot))
  {
    ROS_ERROR("Couldn't retrieve the use robot boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use robot: %d %d\n", (int) _useRobot[LEFT], (int)_useRobot[RIGHT]);
  }

  if (!_nh.getParam("SurgicalTask/useFranka", _useFranka))
  {
    ROS_ERROR("Couldn't retrieve the use Franka boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use Franka: %d\n", (int) _useFranka);
  }

  _humanInputDevice.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/humanInputDevice", _humanInputDevice))
  {
    ROS_ERROR("Couldn't retrieve the human input devices");
    return false;
  }
  else
  {
    ROS_INFO("Human input device: %d %d\n", (int) _humanInputDevice[LEFT], (int)_humanInputDevice[RIGHT]);
  }

  _linearMapping.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/linearMapping", _linearMapping))
  {
    ROS_ERROR("Couldn't retrieve the linear mappings");
    return false;
  }
  else
  {
    ROS_INFO("Linear mapping: %d %d\n", (int) _linearMapping[LEFT], (int)_linearMapping[RIGHT]);
  }

  _selfRotationMapping.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/selfRotationMapping", _selfRotationMapping))
  {
    ROS_ERROR("Couldn't retrieve the self rotation mappings");
    return false;
  }
  else
  {
    ROS_INFO("Self rotation mapping: %d %d\n", (int) _selfRotationMapping[LEFT], (int)_selfRotationMapping[RIGHT]);
  }


  _controlStrategy.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/controlStrategy", _controlStrategy))
  {
    ROS_ERROR("Couldn't retrieve the control strategies");
    return false;
  }
  else
  {
    ROS_INFO("Ccontrol strategy: %d %d\n", (int) _controlStrategy[LEFT], (int)_controlStrategy[RIGHT]);
  }

  if (!_nh.getParam("SurgicalTask/jointImpedanceStiffnessGain", _jointImpedanceStiffnessGain))
  {
    ROS_ERROR("Couldn't retrieve the joint impedance stiffness gain");
    return false;
  }
  else
  {
    ROS_INFO("Joint impedance stiffness gain %f\n", _jointImpedanceStiffnessGain);
  }

  if (!_nh.getParam("SurgicalTask/useSim", _useSim))
  {
    ROS_ERROR("Couldn't retrieve use sim boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use sim: %d\n", (int) _useSim);
  }

  if (!_nh.getParam("SurgicalTask/useOptitrack", _useOptitrack))
  {
    ROS_ERROR("Couldn't retrieve the use optitrack boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use optitrack: %d\n", (int) _useOptitrack);
  }

  _toolOffsetFromEE.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/toolOffsetFromEE", _toolOffsetFromEE))
  {
    ROS_ERROR("Couldn't retrieve the tool offsets from end effector");
    return false;
  }
  else
  {
    ROS_INFO("Tool offset from EE: %f %f\n", _toolOffsetFromEE[LEFT], _toolOffsetFromEE[RIGHT]);
  }

  _footInterfaceRange[LEFT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/leftFootInterfaceRange", _footInterfaceRange[LEFT]))
  {
    ROS_ERROR("Couldn't retrieve the left foot interface ranges");
    return false;
  }
  else
  {
    ROS_INFO("Left foot interface range: %f %f %f %f %f\n", _footInterfaceRange[LEFT][FOOT_X], _footInterfaceRange[LEFT][FOOT_Y], _footInterfaceRange[LEFT][FOOT_PITCH], _footInterfaceRange[LEFT][FOOT_ROLL], _footInterfaceRange[LEFT][FOOT_YAW]);
  }

  _footInterfaceRange[RIGHT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/rightFootInterfaceRange", _footInterfaceRange[RIGHT]))
  {
    ROS_ERROR("Couldn't retrieve the right foot interface ranges");
    return false;
  }
  else
  {
    ROS_INFO("Right foot interface range: %f %f %f %f %f\n", _footInterfaceRange[RIGHT][FOOT_X], _footInterfaceRange[RIGHT][FOOT_Y], _footInterfaceRange[RIGHT][FOOT_PITCH], _footInterfaceRange[RIGHT][FOOT_ROLL], _footInterfaceRange[RIGHT][FOOT_YAW]);
  }

  _footInterfaceDeadZone.resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/footInterfaceDeadZone", _footInterfaceDeadZone))
  {
    ROS_ERROR("Couldn't retrieve the foot interfaces deadzones");
    return false;
  }
  else
  {
    ROS_INFO("Foot interfaces deadzones: %f %f %f %f %f\n", _footInterfaceDeadZone[FOOT_X], _footInterfaceDeadZone[FOOT_Y], _footInterfaceDeadZone[FOOT_PITCH], _footInterfaceDeadZone[FOOT_ROLL], _footInterfaceDeadZone[FOOT_YAW]);
  }


  _trocarSpaceVelocityGains.resize(TROCAR_SPACE_DIM);
  if (!_nh.getParam("SurgicalTask/trocarSpaceVelocityGains", _trocarSpaceVelocityGains))
  {
    ROS_ERROR("Couldn't retrieve the trocar space velocity gains");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space velocity gains: %f %f %f %f\n", _trocarSpaceVelocityGains[V_UP], _trocarSpaceVelocityGains[V_RIGHT], _trocarSpaceVelocityGains[V_INSERTION], _trocarSpaceVelocityGains[W_SELF_ROTATION]);
  }


  if (!_nh.getParam("SurgicalTask/useSafetyLimits", _useSafetyLimits))
  {
    ROS_ERROR("Couldn't retrieve the use safety limit boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use safety limits: %d\n", (int) _useSafetyLimits);
  }


  if (!_nh.getParam("SurgicalTask/safetyLimitsStiffnessGain", _safetyLimitsStiffnessGain))
  {
    ROS_ERROR("Couldn't retrieve the safety limits stiffness gain");
    return false;
  }
  else
  {
    ROS_INFO("Safety limits stiffness gain: %f\n", _safetyLimitsStiffnessGain);
  }

  if (!_nh.getParam("SurgicalTask/allowTaskAdaptation", _allowTaskAdaptation))
  {
    ROS_ERROR("Couldn't retrieve the allow task adaptation boolean");
    return false;
  }
  else
  {
    ROS_INFO("Allow task adaptation: %d\n", (int) _allowTaskAdaptation);
  }



  if (!_nh.getParam("SurgicalTask/toolTipLinearVelocityLimit", _toolTipLinearVelocityLimit))
  {
    ROS_ERROR("Couldn't retrieve the tool tip linear velocity limit");
    return false;
  }
  else
  {
    ROS_INFO("Tool tip linear velocity limit: %f\n", _toolTipLinearVelocityLimit);
  }


  if (!_nh.getParam("SurgicalTask/toolTipSelfAngularVelocityLimit", _toolTipSelfAngularVelocityLimit))
  {
    ROS_ERROR("Couldn't retrieve the tool tip self angular velocity limit");
    return false;
  }
  else
  {
    ROS_INFO("Tool tip self angular velocity limit: %f\n", _toolTipSelfAngularVelocityLimit);
  }


  if (!_nh.getParam("SurgicalTask/trocarSpacePyramidBaseSize", _trocarSpacePyramidBaseSize))
  {
    ROS_ERROR("Couldn't retrieve the trocar space pyramid base size");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space pyramid base size: %f %f\n", _trocarSpacePyramidBaseSize[LEFT], _trocarSpacePyramidBaseSize[RIGHT]);
  }


  std::vector<float> temp;
  temp.resize(6);
  if (!_nh.getParam("SurgicalTask/trocarSpacePyramidBaseOffset", temp))
  {
    ROS_ERROR("Couldn't retrieve the trocar space pyramid base offset");
    return false;
  }
  else
  {
    _trocarSpacePyramidBaseOffset[LEFT] << temp[0], temp[1], temp[2];
    _trocarSpacePyramidBaseOffset[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Trocar space square center offset: LEFT: %f %f %f RIGHT: %f %f %f", _trocarSpacePyramidBaseOffset[LEFT](0), _trocarSpacePyramidBaseOffset[LEFT](1), _trocarSpacePyramidBaseOffset[LEFT](2),
                                                                                  _trocarSpacePyramidBaseOffset[RIGHT](0), _trocarSpacePyramidBaseOffset[RIGHT](1), _trocarSpacePyramidBaseOffset[RIGHT](2));
  }


  if (!_nh.getParam("SurgicalTask/trocarSpaceMinZOffset", _trocarSpaceMinZOffset))
  {
    ROS_ERROR("Couldn't retrieve the trocar space min z offset");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space min z offset: %f %f\n", _trocarSpaceMinZOffset[LEFT], _trocarSpaceMinZOffset[RIGHT]);
  }


  if (!_nh.getParam("SurgicalTask/trocarSpaceLinearDSFixedGain", _trocarSpaceLinearDSFixedGain))
  {
    ROS_ERROR("Couldn't retrieve the trocar space linear ds fixed gain");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space linear ds fixed gain: %f\n", _trocarSpaceLinearDSFixedGain);
  }

  if (!_nh.getParam("SurgicalTask/trocarSpaceLinearDSGaussianGain", _trocarSpaceLinearDSGaussianGain))
  {
    ROS_ERROR("Couldn't retrieve the trocar space linear ds gaussian gain");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space linear ds gaussian gain: %f\n", _trocarSpaceLinearDSGaussianGain);
  }


  if (!_nh.getParam("SurgicalTask/trocarSpaceLinearDSGaussianWidth", _trocarSpaceLinearDSGaussianWidth))
  {
    ROS_ERROR("Couldn't retrieve the trocar space linear ds gaussian width");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space linear ds gaussian width: %f\n", _trocarSpaceLinearDSGaussianWidth);
  }


  if (!_nh.getParam("SurgicalTask/trocarSpaceSelfRotationGain", _trocarSpaceSelfRotationGain))
  {
    ROS_ERROR("Couldn't retrieve the trocar space self rotation gain");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space self rotation gain: %f\n", _trocarSpaceSelfRotationGain);
  }

  if (!_nh.getParam("SurgicalTask/trocarSpaceSelfRotationRange", _trocarSpaceSelfRotationRange))
  {
    ROS_ERROR("Couldn't retrieve the trocar space self rotation range");
    return false;
  }
  else
  {
    ROS_INFO("Trocar space self rotation range: %f\n", _trocarSpaceSelfRotationRange);
  }

  if (!_nh.getParam("SurgicalTask/taskAdaptationAlignmentGain", _taskAdaptationAlignmentGain))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation alignment gain");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation alignment gain: %f\n", _taskAdaptationAlignmentGain);
  }

  if (!_nh.getParam("SurgicalTask/taskAdaptationGaussianWidth", _taskAdaptationGaussianWidth))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation gaussian width");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation gaussian width: %f\n", _taskAdaptationGaussianWidth);
  }

  if (!_nh.getParam("SurgicalTask/taskAdaptationConvergenceGain", _taskAdaptationConvergenceGain))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation convergence gain");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation convergence gain: %f\n", _taskAdaptationConvergenceGain);
  }

  if (!_nh.getParam("SurgicalTask/taskAdaptationProximityGain", _taskAdaptationProximityGain))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation proximity gain");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation proximity gain: %f\n", _taskAdaptationProximityGain);
  }

  if (!_nh.getParam("SurgicalTask/taskAdaptationExponentialGain", _taskAdaptationExponentialGain))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation exponential gain");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation exponential gain: %f\n", _taskAdaptationExponentialGain);
  }  

  if (!_nh.getParam("SurgicalTask/taskAdaptationOverallGain", _taskAdaptationOverallGain))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation overall gain");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation overall gain: %f\n", _taskAdaptationOverallGain);
  }

  if (!_nh.getParam("SurgicalTask/gripperRange", _gripperRange))
  {
    ROS_ERROR("Couldn't retrieve the gripper range");
    return false;
  }
  else
  {
    ROS_INFO("Gripper range: %f\n", _gripperRange);
  }


  // return false;

  if(_useRobot[LEFT])
  {
    // Subscriber definitions
    _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/left_lwr/ee_pose", 1, boost::bind(&SurgicalTask::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/left_lwr/ee_vel", 1, boost::bind(&SurgicalTask::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useFranka)
    {
      _subCurrentJoints[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/left_lwr/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subCurrentJoints[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/left_panda/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    }

    _subDampingMatrix[LEFT] = _nh.subscribe<std_msgs::Float32MultiArray>("/left_lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SurgicalTask::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useSim)
    {
      _subForceTorqueSensor[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&SurgicalTask::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  
    if(_humanInputDevice[LEFT] == JOYSTICK)
    {
      _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg_v3>("/FI_Output/Left",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    // Publisher definitions
    _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/left_lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/left_lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[LEFT] = _nh.advertise<geometry_msgs::Wrench>("/left_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[LEFT] = _nh.advertise<geometry_msgs::Wrench>("SurgicalTask/filteredWrenchLeft", 1);
    _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg_v5>("/left/surgical_task/foot_input", 1);
    _pubNullspaceCommand[LEFT] = _nh.advertise<std_msgs::Float32MultiArray>("/left_lwr/joint_controllers/passive_ds_command_nullspace", 1);
    
    if(!_useFranka)
    {
      _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/command_joint_pos", 1);
    }
    else
    {
      if(_useSim)
      {
        _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/joint_position_controller/command", 1);
      }
      else
      {
        _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/joint_impedance_controller/qd", 1);
      }
    }    
    // _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_lwr/PositionController/command", 1);
    // _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/passive_ds_nullspace_joints", 1);
    _pubRobotData[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/passive_ds_robot_data", 1);
    
    if(!_useFranka)
    {
      _pubStiffness[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/stiffness", 1);
    }
    else
    {
      _pubStiffness[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/joint_impedance_controller/k_gains", 1);
    }

    // _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/passive_ds_nullspace_joints", 1);


  }

  if(_useRobot[RIGHT])
  {
    _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/right_lwr/ee_pose", 1, boost::bind(&SurgicalTask::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/right_lwr/ee_vel", 1, boost::bind(&SurgicalTask::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subCurrentJoints[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_lwr/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SurgicalTask::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
   
    if(!_useSim)
    {
      _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&SurgicalTask::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
   
    if(_humanInputDevice[RIGHT] == JOYSTICK)
    {
      _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg_v3>("/FI_Output/Right",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subFootSharedGrasping[RIGHT] = _nh.subscribe<custom_msgs_gripper::SharedGrasping>("/right/sharedGrasping",1, boost::bind(&SurgicalTask::updateFootSharedGrasping,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    _subGripper = _nh.subscribe("/right/gripperOutput", 1, &SurgicalTask::updateGripperOutput, this, ros::TransportHints().reliable().tcpNoDelay());

    _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/right_lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/right_lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("/right_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("SurgicalTask/filteredWrenchRight", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg_v5>("/right/surgical_task/foot_input", 1);
    _pubNullspaceCommand[RIGHT] = _nh.advertise<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_command_nullspace", 1);
    _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/command_joint_pos", 1);
    // _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/passive_ds_nullspace_joints", 1);
    _pubGripper = _nh.advertise<custom_msgs_gripper::GripperInputMsg>("/right/gripperInput", 1);
    _pubStiffness[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/stiffness", 1);

  }

  _pubSurgicalTaskState = _nh.advertise<surgical_task::SurgicalTaskStateMsg>("surgical_task/state", 1);



  _subOptitrackPose[RIGHT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[RIGHT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());


  // _subSphericalTrocars = _nb.subscribe<std_msgs::Float32MultiArray>("/spherical_trocar_frames")

  _outputFile.open(ros::package::getPath(std::string("robotic_experiments"))+"/data_foot/bou.txt");

  signal(SIGINT,SurgicalTask::stopNode);

  // _startThread = true;
  // if(pthread_create(&_thread, NULL, &SurgicalTask::startIkLoop, this))
  // {
  //     throw std::runtime_error("Cannot create reception thread");  
  // }


  if(!_useSim)
  {
    _xRobotBaseOrigin[LEFT].setConstant(0.0f);
     _xRobotBaseOrigin[LEFT] << 0.0f, 1.04f, 0.0f;
    _xRobotBaseOrigin[RIGHT].setConstant(0.0f);
  }


  if(_usePredefinedTrocars)
  {
    Eigen::Vector3f temp;
    _trocarPosition[LEFT] << -0.304, -0.432f, 0.696f-_toolOffsetFromEE[LEFT];
    temp << 0.265f,-0.490f,-0.830f;
    temp.normalize();
    _trocarOrientation[LEFT] << temp;

    _trocarPosition[RIGHT] << -0.308f, 0.471f, 0.741f-_toolOffsetFromEE[RIGHT];
    temp << 0.116f,0.220f,-0.968f;
    temp.normalize();
    _trocarOrientation[RIGHT] << temp; 
  }

  _pillarsId.resize(3);
  // _pillarsId << 0, 1, 2;
  _pillarsId <<  1, 3, 10;

  // if(_useRobot[RIGHT])
  // {
  //   _nbTasks += 1;
  // }

  _nbTasks = 3; 

  _beliefsC.resize(_nbTasks);
  _dbeliefsC.resize(_nbTasks);
  for(int k = 0; k < _nbTasks; k++)
  {
    if(!(_useRobot[RIGHT] && k == _nbTasks-1))
    {
      _firstPillarsFrame[k] = false;
    }
    _beliefsC(k) = 0.0f;
    _dbeliefsC(k) = 0.0f;
  }

  _beliefsC(0) = 1.0f;

  _pillarsPosition.resize(_pillarsId.size(),3);
  _pillarsPosition.setConstant(0.0f);

  Eigen::Vector3f p0;
  p0 <<-0.413005,  0.443508, 0.0539682;

  if(!_useSim)
  {
    _pillarsPosition.row(0) = p0;
    _pillarsPosition.row(1) << p0(0), p0(1)-0.064, p0(2);
    _pillarsPosition.row(2) << p0(0)+0.064, p0(1)-0.064, p0(2);    
  }
  

  // Read Neural network matrices
  _p[0].resize(50,4);
  _p[1].resize(50,1);
  _p[2].resize(3,50);
  _p[3].resize(3,1);

  for(int id = 0; id < 4; id++)
  {
    std::ifstream file(ros::package::getPath(std::string("surgical_task"))+"/p"+std::to_string(id+1)+".txt");   
    if (file.is_open())
    {
      for(int k = 0; k < _p[id].rows(); k++)
      {
        for(int m = 0; m < _p[id].cols(); m++)
        {
          file >> _p[id](k,m);
        }
      }
      std::cerr << _p[id] << std::endl << std::endl;
      file.close();
    }
    else
    {
      // return false;
    }
  }

  if(_useFranka)
  {
    _robotID = Utils<float>::ROBOT_ID::FRANKA_PANDA;
  }
  else
  {
    _robotID = Utils<float>::ROBOT_ID::KUKA_LWR;
  }

  _qpSolverRCM[LEFT].setRobot(_robotID);
  _qpSolverRCM[RIGHT].setRobot(_robotID);

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[SurgicalTask]: The SurgicalTask node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[SurgicalTask]: The SurgicalTask node has a problem.");
    return false;
  }
}


void SurgicalTask::run()
{
  while(!_stop) 
  {
    if(allSubscribersOK() && allFramesOK() && _trocarsRegistered[LEFT] && _trocarsRegistered[RIGHT])
    {
      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/left_lwr/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);
      ros::param::getCached("/right_lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
          
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      // logData();
    }
    else
    {
      if(_useSim)
      {
        if(!allFramesOK())
        {
          receiveFrames();
        }     

        if(!_usePredefinedTrocars)
        {
          registerTrocars();
        }

      }
      else
      {
        if(!_usePredefinedTrocars)
        {
          registerTrocars();
        }
      }
    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  // _startThread = false;
  // pthread_join(_thread,NULL);

  // Send zero velocity command to stop the robot
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k] = _q[k];  
    _desiredFootWrench[k].setConstant(0.0f);  
    _stiffness[k] = 0.0f;
    _desiredGripperPosition[k] = 0.0f;
    for(int m = 0; m < 7; m++)
    {
      _ikJoints[k][m] = _currentJoints[k](m);
    }
  }

  _msgGripperInput.ros_dPosition = 0.0f;

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  ros::shutdown();
}


void SurgicalTask::stopNode(int sig)
{
  me->_stop = true;
}


bool SurgicalTask::allSubscribersOK()
{
  bool robotStatus[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatus[k] = !_useRobot[k] || ((_useFranka ||_firstRobotPose[k]) && (_useFranka || _firstRobotTwist[k])
                      // && _firstDampingMatrix[k] 
                      && _firstJointsUpdate[k] 
                      && _firstHumanInput[k]);
                      // && (_useSim || _wrenchBiasOK[k]);

    if(!robotStatus[k])
    {
      std::cerr << k << ": Status: " << "use: " << _useRobot[k] 
                << " pose: " << _firstRobotPose[k] << " twist: " << _firstRobotTwist[k]
                << " damp: " << _firstDampingMatrix[k] << " joints: " << _firstJointsUpdate[k]
                << " human: " << _firstHumanInput[k]
                << " sim/wrench: " << (_useSim || _wrenchBiasOK[k]) << std::endl;
    }
  }

  if(_useOptitrack)
  {
    _optitrackOK = true;
    for(int k = 0; k < NB_TRACKED_OBJECTS-2; k++)
    {
      _optitrackOK = _optitrackOK && _firstOptitrackPose[k];
      if(!_firstOptitrackPose[k])
      {
        std::cerr << "Optirack object: " << k << " missing" << std::endl; 
      }
    }

    if(_optitrackOK && !_optitrackInitialized)
    {
      optitrackInitialization();
    }
  }
  return robotStatus[LEFT] && robotStatus[RIGHT] && (!_useOptitrack || (_optitrackOK && _optitrackInitialized));// && (_useSim ||_firstGripper);
}


bool SurgicalTask::allFramesOK()
{
  bool frameStatus[NB_ROBOTS];
  frameStatus[LEFT] = false;
  frameStatus[RIGHT] = false;

  if(_useSim)
  {
    for(int k = 0; k < NB_ROBOTS; k++)
    {

      bool pillarsStatus = true;
      if(k == 0)
      {
        for(int m = 0; m < _pillarsId.size(); m++)
        {
          pillarsStatus = pillarsStatus && _firstPillarsFrame[m];
        }
      }

      frameStatus[k] = !_useRobot[k] || (_firstRobotBaseFrame[k] && _firstSphericalTrocarFrame[k] && pillarsStatus);

      if(!frameStatus[k])
      {
        std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
                  << " robot base: " << _firstRobotBaseFrame[k] 
                  << " trocar : " << _firstSphericalTrocarFrame[k] << " pillars: " << pillarsStatus << std::endl;
      }
    }

  }

  return !_useSim || (frameStatus[LEFT] && frameStatus[RIGHT]); 
}


void SurgicalTask::receiveFrames()
{

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_useRobot[k])
    {      
      try
      { 
        if(!_firstRobotBaseFrame[k])
        {
          if(k==LEFT)
          {
            if(!_useFranka)
            {
              _lr.waitForTransform("/world", "/left_lwr_base_link", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/left_lwr_base_link", ros::Time(0), _transform);                      
            }
            else
            {
              _lr.waitForTransform("/world", "/left_panda_link0", ros::Time(0), ros::Duration(3.0));
              _lr.lookupTransform("/world", "/left_panda_link0", ros::Time(0), _transform);                                    
            }
          }
          else
          {
            _lr.waitForTransform("/world", "/right_lwr_base_link", ros::Time(0), ros::Duration(3.0));
            _lr.lookupTransform("/world", "/right_lwr_base_link", ros::Time(0), _transform); 
          }
          _xRobotBaseOrigin[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
          _qRobotBaseOrigin[k] << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
          _wRRobotBasis[k] = Utils<float>::quaternionToRotationMatrix(_qRobotBaseOrigin[k]);
          _firstRobotBaseFrame[k] = true;
          std::cerr << "[SurgicalTask]: Robot " << k << " origin received: " << _xRobotBaseOrigin[k].transpose() << std::endl;
        } 
      } 
      catch (tf::TransformException ex)
      {
      }
    }

    if(!_firstSphericalTrocarFrame[k])
    {
      try
      { 

        _lr.waitForTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), _transform);        
        _trocarPosition[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
        Eigen::Vector4f temp;
        temp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstSphericalTrocarFrame[k] = true;
        // _trocarsRegistered[k] = true;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " origin received: " << _trocarPosition[k].transpose() << std::endl;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " orientation received: " << _trocarOrientation[k].transpose() << std::endl;
      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }

  for(int k = 0; k < _pillarsId.size(); k++)
  {
    if(_firstSphericalTrocarFrame[LEFT] && !_firstPillarsFrame[k])
    {    
      try
      { 
        _lr.waitForTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), _transform);        
        _pillarsPosition.row(k) = Eigen::Vector3f(_transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z());
        Eigen::Vector4f q;
        q << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        // _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstPillarsFrame[k] = true;
        std::cerr << "[SurgicalTask]: Pillars " << k << " origin received: " << _pillarsPosition.row(k).transpose() << std::endl;


        ros::ServiceClient client = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        //position
        geometry_msgs::Point linkPosition;
        linkPosition.x = _pillarsPosition(k,0);
        linkPosition.y = _pillarsPosition(k,1);
        linkPosition.z = _pillarsPosition(k,2);
        //orientation
        geometry_msgs::Quaternion linkOrientation;

        Eigen::Vector4f qe, qd;
        Eigen::Vector3f z, zd;
        z = Utils<float>::quaternionToRotationMatrix(q).col(2);
        zd = (_trocarPosition[LEFT]-_pillarsPosition.row(k).transpose()).normalized();
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(z,zd));
        qd = Utils<float>::quaternionProduct(qe,q);

        // Compute final quaternion on plane
        linkOrientation.x = qd(1);
        linkOrientation.y = qd(2);
        linkOrientation.z = qd(3);
        linkOrientation.w = qd(0);

        //pose (Pose + Orientation)
        geometry_msgs::Pose modelPose;
        modelPose.position = linkPosition;
        modelPose.orientation = linkOrientation;

        //ModelState
        gazebo_msgs::ModelState modelState;
        modelState.model_name = (std::string) "target"+std::to_string(k+1);
        modelState.pose = modelPose;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = modelState;

        if(client.call(srv))
        {
            ROS_INFO("Set object pose");
        }
        else
        {
            ROS_ERROR("Reset frame pose! Error msg:%s",srv.response.status_message.c_str());
        }

      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }
}


void SurgicalTask::computeCommand()
{
  humanInputTransformation();

  if(_useOptitrack)
  {
    updateHumanToolPosition();
  }

  for(int r = 0; r <NB_ROBOTS; r++)
  {   
    if(_useRobot[r])
    {
      updateTrocarInformation(r);
      
      selectRobotMode(r);
      
      switch(_robotMode[r])
      {
        case TROCAR_SELECTION:
        {
          trocarSelection(r);
          break;
        }
        case TROCAR_INSERTION:
        {
          trocarInsertion(r);
          break;
        }
        case TROCAR_SPACE:
        {
          trocarSpace(r);
          break;
        }
        default:
        {
          break;
        }
      }
    
      if(_humanInputDevice[r] == FOOT)
      {
        computeHapticFeedback(r);
        computeDesiredFootWrench(r);
      }
    }
  }

}


void SurgicalTask::updateHumanToolPosition()
{
  Eigen::Vector3f toolBasePosition[2];

  // Get tool position on robot basis
  toolBasePosition[LEFT] = _markersPosition.col(LEFT_HUMAN_TOOL)-_markersPosition.col(RIGHT_ROBOT_BASIS);
  toolBasePosition[RIGHT] =  _markersPosition.col(RIGHT_HUMAN_TOOL)-_markersPosition.col(RIGHT_ROBOT_BASIS);
  Eigen::Vector3f offsetWL, offsetWR, offsetOL, offsetOR;


  // Initialize correction offset
  offsetWL.setConstant(0.0f);
  offsetWR.setConstant(0.0f);
  offsetOL.setConstant(0.0f);
  offsetOR.setConstant(0.0f);

  // VALID ONLY WHEN EE TIP matches TOOL TIP
  offsetWL = (_x[LEFT]-toolBasePosition[LEFT]);
  offsetWL(2) -= 0.0025f;
  offsetWR << (_x[LEFT]-toolBasePosition[RIGHT]);
  offsetWR(2) -= 0.0025f;

  offsetOL = (_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(LEFT_HUMAN_TOOL))).inverse()*offsetWL;
  offsetOR = (_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(RIGHT_HUMAN_TOOL))).inverse()*offsetWR;
  _offsetTool = offsetOL;


  // Compute learned correction offset
  Eigen::MatrixXf temp;
  temp.resize(50,1);

  temp = _p[0]*_markersQuaternion.col(LEFT_HUMAN_TOOL)+_p[1];
  for(int k =0; k < temp.rows(); k++)
  {
    if(temp(k)<0.0f)
    {
      temp(k) = 0.0f;
    }
  }
  offsetOL = _p[2]*temp+_p[3];
  // offsetOL.setConstant(0.0f);

  std::cerr << "OL: " << offsetOL.transpose() << std::endl;
  std::cerr << "OR: " << offsetOR.transpose() << std::endl;
  // offsetOL << 0.0124496,   0.025967, -0.0133567;
  offsetOR << -0.0014803, 0.00447115, 0.00305053;

  // Apply correction
  _humanToolPosition[LEFT] = toolBasePosition[LEFT]+_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(LEFT_HUMAN_TOOL))*offsetOL;
  _humanToolPosition[RIGHT] = toolBasePosition[RIGHT]+_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(RIGHT_HUMAN_TOOL))*offsetOR;

  std::cerr << "offset L:" << (_x[LEFT]-toolBasePosition[LEFT]).transpose() << std::endl;
  std::cerr << "offset R:" << (_x[LEFT]-toolBasePosition[RIGHT]).transpose() << std::endl;
}


void SurgicalTask::updateTrocarInformation(int r)
{
  // Compute vector EE to trocar
  _rEETrocar[r] = _trocarPosition[r]-_xEE[r];
  // Compute RCM position
  _xRCM[r] = _xEE[r]+(_trocarPosition[r]-_xEE[r]).dot(_wRb[r].col(2))*_wRb[r].col(2);
  
  // Compute vector EE to RCM
  _rEERCM[r] = _xRCM[r]-_xEE[r];

  // Compute distance RCM tool
  _dRCMTool[r] = (_trocarPosition[r]-_xEE[r]).dot(_wRb[r].col(2))-_toolOffsetFromEE[r];

  _xdEE[r] = _trocarPosition[r]-_rEETrocar[r].dot(_trocarOrientation[r])*_trocarOrientation[r];

  // Linear DS to go to the attractor
  _fxk[r]= 4.0f*(_xdEE[r]-_xEE[r]); 
  std::cerr << "[SurgicalTask]: " << r << ": Distance to attractor: " << (_xdEE[r]-_xEE[r]).norm() << std::endl;
}



void SurgicalTask::selectRobotMode(int r)
{
  _robotMode[r] = TROCAR_INSERTION;
  _alignedWithTrocar[r] = true;
  // std::cerr << "[SurgicalTask]: " << r << ": origin " << _xRobotBaseOrigin[r].transpose() << std::endl;
  std::cerr << "[SurgicalTask]: " << r << ": trocar " << _trocarPosition[r].transpose() << std::endl;
  std::cerr << "[SurgicalTask]: " << r << ": x " << _x[r].transpose() << std::endl;

  if(_alignedWithTrocar[r]==true)
  {
    std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-tool: " << _dRCMTool[r] << std::endl;
    std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-trocar: " << (_trocarPosition[r]-_xRCM[r]).norm() <<std::endl;
    if(_dRCMTool[r] >= 0.01f)
    {
      _robotMode[r] = TROCAR_SELECTION;
    }
    else if(_dRCMTool[r] >= -0.04f && _dRCMTool[r] < 0.01f)
    {
      _robotMode[r] = TROCAR_INSERTION;
    }
    else
    {
      _robotMode[r] = TROCAR_SPACE;
    }
  }
}


void SurgicalTask::trocarSelection(int r)
{
  std::cerr << "[SurgicalTask]: " << r << ": TROCAR SELECTION" << std::endl;


  _fx[r] = _fxk[r];

  // // Find max belief
  // Eigen::MatrixXf::Index indexMax;
  // float bmax = _beliefs[r].array().maxCoeff(&indexMax);


  if(!_alignedWithTrocar[r])
  {
    _vd[r] = _fx[r];
  }
  else
  {
    _vd[r] = _fx[r];
    _vd[r] += 0.1f*_trocarInput[r](2)*_rEETrocar[r].normalized();  
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.3f);
  
  Eigen::Vector4f qe;
  
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

  Eigen::Vector3f axis;  
  float angleErrorToTrocarPosition;
  Utils<float>::quaternionToAxisAngle(qe, axis, angleErrorToTrocarPosition);

  if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                     -MAX_ORIENTATION_ERROR,
                                                                     MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe, _q[r]);

  float angleErrorToTrocarOrientation = std::acos(Utils<float>::bound(_trocarOrientation[r].dot(_wRb[r].col(2)),-1.0f,1.0f));

  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;
  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarOrientation: " << angleErrorToTrocarOrientation << std::endl;
  // std::cerr << "[SurgicalTask]: " << r << ": Distance to attractor: " << _fx[r].norm()/4.0f << std::endl;


  if(std::fabs(angleErrorToTrocarPosition) < 0.05f && 
     std::fabs(angleErrorToTrocarOrientation)< 0.07f && 
     _alignedWithTrocar[r] == false)
  {
    _alignedWithTrocar[r] = true;
  }

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r], _qd[r]);

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _ikJoints[r] = _currentJoints[r];
  _stiffness[r] = 0.0f;
  _desiredGripperPosition[r] = 0.0f;


  std::cerr << "[SurgicalTask]: " << r << ": Aligned with trocar: " << _alignedWithTrocar[r] << std::endl;
}


void SurgicalTask::trocarInsertion(int r)
{
  std::cerr << "[SurgicalTask]: " << r << ": TROCAR INSERTION" << std::endl;

  std::cerr << "[SurgicalTask]: trocar input:  " << _trocarInput[r].transpose() << std::endl;

  if(_linearMapping[r]==POSITION_VELOCITY || _useSim)
  {
    _vd[r] = 0.1f*_trocarInput[r](2)*_rEETrocar[r].normalized(); 
    if(_dRCMTool[r]> -0.005f && (_wRb[r].col(2)).dot(_vd[r])<0.0f)
    {
      _vd[r].setConstant(0.0f);
    }
  }
  else
  {
    _vd[r].setConstant(0.0f);
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.3f);


  if(_controlStrategy[r] == PASSIVE_DS)
  {
    _stiffness[r] = 0.0f;
    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

    Eigen::Vector3f axis;  
    float angleErrorToTrocarPosition;
    Utils<float>::quaternionToAxisAngle(qe, axis, angleErrorToTrocarPosition);

    std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;


    if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
    {
      qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                       -MAX_ORIENTATION_ERROR,
                                                                       MAX_ORIENTATION_ERROR));
    }

    // Compute final quaternion on plane
    _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

    _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);
  }
  else if (_controlStrategy[r] == JOINT_IMPEDANCE && _firstPublish[r])
  {
    if(_linearMapping[r]==POSITION_VELOCITY || _useSim)
    {
      _stiffness[r] = _jointImpedanceStiffnessGain;
      // std::cerr << _xRobotBaseOrigin[r].transpose() << std::endl;
      // std::cerr << _trocarPosition[r].transpose() << " " << _xRCM[r].transpose() << std::endl;
      // std::cerr << _x[r].transpose() << std::endl;
      // std::cerr << _vd[r].transpose() << std::endl;

      _selfRotationCommand[r] =_trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[r](W_SELF_ROTATION);
      if(_currentJoints[r](6)>_trocarSpaceSelfRotationRange && _selfRotationCommand[r]>0.0f)
      {
        _selfRotationCommand[r] = 0.0f;
      }
      else if(_currentJoints[r](6)<-_trocarSpaceSelfRotationRange && _selfRotationCommand[r]<0.0f)
      {
        _selfRotationCommand[r] = 0.0f;
      }
      bool res = _qpSolverRCM[r].step3(_ikJoints[r], _ikJoints[r], _trocarPosition[r],
      _toolOffsetFromEE[r], _vd[r], _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f);
      std::cerr << "[SurgicalTask]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      std::cerr << "[SurgicalTask]: " << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;

    }
    else
    {
      _stiffness[r] = 0.0f;
      _ikJoints[r] = _currentJoints[r];
    }
  }
  else
  {
    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _qd[r] = _q[r];  
    _ikJoints[r] = _currentJoints[r];
    std::cerr << "[SurgicalTask]: " << r << ": CONTROL STRATEGY UNKNOWN" << std::endl;
  }

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _wRb0[r] = _wRb[r];
  _xd0[r] = _x[r];
  if(_linearMapping[r] == POSITION_POSITION)
  {
    _xd0[r](2) -= 0.01f;
  }
  _inputAlignedWithOrigin[r] = false;
  _xdTool[r] = _x[r];

  _desiredGripperPosition[r] = 0.0f;
}


void SurgicalTask::trocarSpace(int r)
{
  std::cerr << "[SurgicalTask]: " << r << ": TROCAR SPACE" << std::endl;

  // Compute desired tool velocity
  computeDesiredToolVelocity(r);

  std::cerr << "[SurgicalTask]: " << r << ": vd tool before: " << _vdTool[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 

  // Scale the desired velocity components normal to the tool depending on the penetration depth
  float depthGain = std::min(std::max((_x[r]-_trocarPosition[r]).dot(_wRb[r].col(2)),0.0f)*3.0f/_toolOffsetFromEE[r],1.0f);
  std::cerr << "[SurgicalTask]: " << r << ": depth gain: " <<  depthGain << std::endl;
  
  Eigen::Matrix3f L;
  L.setIdentity();
  L(0,0) = depthGain;
  L(1,1) = depthGain;
  _vdTool[r] = _wRb[r]*L*_wRb[r].transpose()*_vdTool[r];
  
  // Bound vd tool
  _vdTool[r] = Utils<float>::bound(_vdTool[r], _toolTipLinearVelocityLimit);

  std::cerr << "[SurgicalTask]: " << r << ": vd tool after: " << _vdTool[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 

  // Compute desired gripper position
  _desiredGripperPosition[r] = _gripperRange*(1.0f-std::max(0.0f,_trocarInput[r](EXTRA_DOF)));

  if (_controlStrategy[r] == PASSIVE_DS)
  {
    _stiffness[r] = 0.0f;
    Eigen::Matrix<float,6,6> A;
    A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2))*Eigen::Matrix3f::Identity();
    A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r]);
    A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
    A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_toolOffsetFromEE[r]*_wRb[r].col(2));
    Eigen::Matrix<float,6,1> x, b;
    b.setConstant(0.0f);
    b.segment(3,3) = _vdTool[r];

    x = A.fullPivHouseholderQr().solve(b);
    _vd[r] = x.segment(0,3);
    _omegad[r] = x.segment(3,3);

    // _omegad[r] = (_x[r]-_xEE[r]-_rEERCM[r]).cross(Utils<float>::orthogonalProjector(_wRb[r].col(2))*_vdTool[r])/(_x[r]-_xEE[r]-_rEERCM[r]).squaredNorm();
    // _vd[r] = _vdTool[r]-_omegad[r].cross(_x[r]-_xEE[r]);
    
    _vd[r]+=2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_trocarPosition[r]-_xRCM[r]);

    _vd[r] = Utils<float>::bound(_vd[r],0.4f);


    _omegad[r] = Utils<float>::bound(_omegad[r],3.0f);

    _nullspaceWrench[r].setConstant(0.0f);
    // _nullspaceWrench[r].segment(0,3) = 50.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
    // _nullspaceCommand[r] = 50.0f*jacobianRCM(_currentJoints[r],_trocarPosition[r][indexMax]).transpose()*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);


    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

    Eigen::Vector3f axis;  
    float angleErrorToTrocarPosition;
    Utils<float>::quaternionToAxisAngle(qe,axis,angleErrorToTrocarPosition);

    std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;

    if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
    {
      qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                       -MAX_ORIENTATION_ERROR,
                                                                       MAX_ORIENTATION_ERROR));
    }

    // Compute final quaternion
    _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

    // Add self rotation command
    _omegad[r] += _selfRotationCommand[r]*_wRb[r].col(2);

  }
  else if(_controlStrategy[r] == JOINT_IMPEDANCE && _firstPublish[r])
  {

    _stiffness[r] = _jointImpedanceStiffnessGain;
   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    bool res = _qpSolverRCM[r].step3(_ikJoints[r], _ikJoints[r], _trocarPosition[r],
             _toolOffsetFromEE[r], _vdTool[r], _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cerr << "[SurgicalTask]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
    std::cerr << "[SurgicalTask]: " << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;
  }
  else
  {
    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _ikJoints[r] = _currentJoints[r];
    std::cerr << "[SurgicalTask]: " << r << ": CONTROL STRATEGY UNKNOWN" << std::endl;
  }
}


void SurgicalTask::computeDesiredToolVelocity(int r)
{
  std::cerr << "[SurgicalTask]: trocar input:  " << _trocarInput[r].transpose() << std::endl;

  // Compute IK tip position
  Eigen::Matrix4f Hik;
  Hik = Utils<float>::getForwardKinematics(_ikJoints[r],_robotID);
  _xIK[r] = _xRobotBaseOrigin[r]+_wRRobotBasis[r]*Hik.block(0,3,3,1)+_toolOffsetFromEE[r]*_wRRobotBasis[r]*Hik.block(0,2,3,1);

  std::cerr << "[SurgicalTask]: " << r << " xIK: " << _xIK[r].transpose() << std::endl;

  if(_linearMapping[r] == POSITION_VELOCITY)
  {
    Eigen::Vector3f gains;
    gains << _trocarSpaceVelocityGains[V_UP], _trocarSpaceVelocityGains[V_RIGHT], _trocarSpaceVelocityGains[V_INSERTION];

    _vdTool[r] = _wRb[r]*(gains.cwiseProduct(_trocarInput[r].segment(0,3)));   

    if(_useSafetyLimits)
    {
      Eigen::Vector3f vs;
      vs.setConstant(0.0f);

      Eigen::Vector3f pyramidOffset, pyramidHeight, pyramidCenter;
      pyramidOffset << _trocarSpacePyramidBaseOffset[r](X), _trocarSpacePyramidBaseOffset[r](Y), 0.0f; 
      pyramidHeight << 0.0f, 0.0f, _trocarSpacePyramidBaseOffset[r](Z);
      pyramidCenter = pyramidHeight+pyramidOffset;

      float pyramidAngle = std::atan2(_trocarSpacePyramidBaseSize[r]/2,-pyramidHeight(Z));

      float dotProduct = (pyramidHeight.normalized()).dot((pyramidOffset+pyramidHeight).normalized());
      float theta = std::acos(std::max(std::min(dotProduct,1.0f),-1.0f));  

      float xMin,xMax,yMin,yMax;
      xMax = pyramidCenter(X)-pyramidHeight(Z)*tan(pyramidAngle);
      xMax = std::max(0.0f,xMax);
      xMin = pyramidCenter(X)-pyramidHeight(Z)*tan(-pyramidAngle);
      xMin = std::min(0.0f,xMin);
      yMax = pyramidCenter(Y)-pyramidHeight(Z)*tan(pyramidAngle);
      yMax = std::max(0.0f,yMax);
      yMin = pyramidCenter(Y)-pyramidHeight(Z)*tan(-pyramidAngle);
      yMin = std::min(0.0f,yMin);

      Eigen::Vector3f currentOffset;
      currentOffset = _xIK[r]-_xd0[r];

      if(currentOffset(Z)>pyramidHeight(Z))
      {
        Eigen::Vector3f off;
        if(pyramidOffset.norm()< FLT_EPSILON)
        {
          off.setConstant(0.0f); 
        }
        else
        {
          off = std::tan(theta)*currentOffset(Z)*pyramidOffset.normalized();
        }
        xMax = -currentOffset(Z)*std::tan(pyramidAngle)-off(X);  
        xMin = -currentOffset(Z)*std::tan(-pyramidAngle)-off(X);  
        yMax = -currentOffset(Z)*std::tan(pyramidAngle)-off(Y);  
        yMin = -currentOffset(Z)*std::tan(-pyramidAngle)-off(Y);  
      }

      Eigen::Vector3f vplane, vortho;
      vplane = Utils<float>::orthogonalProjector(_wRb[r].col(2))*_vdTool[r];
      vortho = _vdTool[r]-vplane;

      bool safetyCollison = false;
      if(currentOffset(0) > xMax && _vdTool[r](0) > 0)
      {
        safetyCollison = true;
      }
      else if(currentOffset(0)< xMin && _vdTool[r](0) < 0)
      {
        safetyCollison = true;
      }
      if(currentOffset(1) > yMax && _vdTool[r](1) > 0)
      {
        safetyCollison = true;
      }
      else if(currentOffset(1)< yMin && _vdTool[r](1) < 0)
      {
        safetyCollison = true;
      }
      if(currentOffset(2)< _trocarSpaceMinZOffset[r] && _vdTool[r](2) < 0.0f)
      {
        safetyCollison = true;
      }

      if(safetyCollison)
      {
        if(vortho.dot(_wRb[r].col(2))<0.0f)
        {
          _vdTool[r] = vortho;
        }
        else
        {
          _vdTool[r].setConstant(0.0f);
        }
      }



      // if(currentOffset(2)<-0.02f)
      // {
      //   if(currentOffset(0)> xMax)
      //   {
      //     if(_vdTool[r](0)>0.0f)
      //     {
      //       _vdTool[r].setConstant(0.0f);
      //     }
      //     // vs(0) = _safetyLimitsStiffnessGain*(xMax-currentOffset(0));
      //     // _vdTool[r](1) = 0.0f;
      //     // // if(currentOffset(2)<-0.02f)
      //     // {
      //     //   _vdTool[r](2) = 0.0f;
      //     // }
      //   }
      //   else if(currentOffset(0) < xMin)
      //   {
      //     if(_vdTool[r](0)<0.0f)
      //     {
      //       _vdTool[r].setConstant(0.0f);
      //     }
      //     // vs(0) = _safetyLimitsStiffnessGain*(xMin-currentOffset(0));

      //     // _vdTool[r](1) = 0.0f;
      //     // // if(currentOffset(2)<-0.02f)
      //     // {
      //     //   _vdTool[r](2) = 0.0f;
      //     // }
      //   }

      //   if(currentOffset(1)> yMax)
      //   {
      //     if(_vdTool[r](1)>0.0f)
      //     {
      //       _vdTool[r].setConstant(0.0f);
      //     }
      //     // vs(1) = _safetyLimitsStiffnessGain*(yMax-currentOffset(1));

      //     // _vdTool[r](0) = 0.0f;
      //     // // if(currentOffset(2)<-0.02f)
      //     // {
      //     //   _vdTool[r](2) = 0.0f;
      //     // }
      //   }
      //   else if(currentOffset(1) < yMin)
      //   {
      //     if(_vdTool[r](1)<0.0f)
      //     {
      //       _vdTool[r].setConstant(0.0f);
      //     }
      //     // vs(1) = _safetyLimitsStiffnessGain*(yMin-currentOffset(1));

      //     // _vdTool[r](0) = 0.0f;
      //     // // if(currentOffset(2)<-0.02f)
      //     // {
      //     //   _vdTool[r](2) = 0.0f;
      //     // }
      //   }

      //   if(currentOffset(2) < _trocarSpaceMinZOffset[r])
      //   {
      //     if(_vdTool[r](2)<0.0f)
      //     {
      //       _vdTool[r].setConstant(0.0f);
      //     }
      //     // vs(2) = _safetyLimitsStiffnessGain*(_trocarSpaceMinZOffset[r]-currentOffset(2));

      //     _vdTool[r].segment(0,2).setConstant(0.0f);
      //   }

      // }

      std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(0) << " " << xMin << " " << xMax << std::endl;
      std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(1) << " " << yMin << " " << yMax << std::endl;
      std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(2) << " " << _trocarSpaceMinZOffset[r] << " " << "0.0" << std::endl;
      std::cerr << "[SurgicalTask]: " << r << " vs: " << vs.transpose() << std::endl;
        
      _vdTool[r] += vs;
    }

    _selfRotationCommand[r] = _trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[r](W_SELF_ROTATION);
    _selfRotationCommand[r] = Utils<float>::bound(_selfRotationCommand[r],-_toolTipSelfAngularVelocityLimit,_toolTipSelfAngularVelocityLimit);
    if(_currentJoints[r](6)>_trocarSpaceSelfRotationRange && _selfRotationCommand[r]>0.0f)
    {
      _selfRotationCommand[r] = 0.0f;
    }
    else if(_currentJoints[r](6)<-_trocarSpaceSelfRotationRange && _selfRotationCommand[r]<0.0f)
    {
      _selfRotationCommand[r] = 0.0f;
    }


    if(r==LEFT && _allowTaskAdaptation)
    {
      if(_useTaskAdaptation && _trocarInput[r](EXTRA_DOF)>0.6f)
      {
        _useTaskAdaptation = false;
      }
      if(!_useTaskAdaptation && _trocarInput[r](EXTRA_DOF) <-0.6f)
      {
        initializeBeliefs(r);
        _useTaskAdaptation = true;
      }
      if(_useTaskAdaptation)
      {
        taskAdaptation(r);
        _vdTool[r] = gains(V_INSERTION)*_trocarInput[r](V_INSERTION)*_wRb[r].col(V_INSERTION)+_vda;
      }   
    }

  }
  else if(_linearMapping[r]==POSITION_POSITION)
  {
    Eigen::Vector3f pyramidOffset, pyramidHeight, pyramidCenter;
    pyramidOffset << _trocarSpacePyramidBaseOffset[r](X), _trocarSpacePyramidBaseOffset[r](Y), 0.0f; 
    pyramidHeight << 0.0f, 0.0f, _trocarSpacePyramidBaseOffset[r](Z);
    pyramidCenter = pyramidHeight+pyramidOffset;

    float pyramidAngle = std::atan2(_trocarSpacePyramidBaseSize[r]/2,-pyramidHeight(Z));

    float dotProduct = (pyramidHeight.normalized()).dot((pyramidOffset+pyramidHeight).normalized());
    float theta = std::acos(std::max(std::min(dotProduct,1.0f),-1.0f));  

    float xMin,xMax,yMin,yMax;
    xMax = pyramidCenter(X)-pyramidHeight(Z)*tan(pyramidAngle);
    xMax = std::max(0.0f,xMax);
    xMin = pyramidCenter(X)-pyramidHeight(Z)*tan(-pyramidAngle);
    xMin = std::min(0.0f,xMin);
    yMax = pyramidCenter(Y)-pyramidHeight(Z)*tan(pyramidAngle);
    yMax = std::max(0.0f,yMax);
    yMin = pyramidCenter(Y)-pyramidHeight(Z)*tan(-pyramidAngle);
    yMin = std::min(0.0f,yMin);

    Eigen::Vector3f desiredOffset;

    desiredOffset(Z) = _trocarSpaceMinZOffset[r]*std::max(_trocarInput[r](Z),0.0f);
    if(desiredOffset(Z)>pyramidHeight(Z))
    {
      std::cerr << "[SurgicalTask]: " << "PYRAMID" << std::endl;
      Eigen::Vector3f off;
      if(pyramidOffset.norm()< FLT_EPSILON)
      {
        off.setConstant(0.0f); 
      }
      else
      {
        off = std::tan(theta)*desiredOffset(Z)*pyramidOffset.normalized();
      }
      desiredOffset(X) = -desiredOffset(Z)*std::tan(pyramidAngle*_trocarInput[r](X))-off(X);  
      desiredOffset(Y) = -desiredOffset(Z)*std::tan(pyramidAngle*_trocarInput[r](Y))-off(Y);
    }
    else
    {
      std::cerr << "[SurgicalTask]: " << "SQUARE" << std::endl;
      desiredOffset(X) = (xMin+xMax)/2+_trocarInput[r](X)*(xMax-xMin)/2;        
      desiredOffset(Y) = (yMin+yMax)/2+_trocarInput[r](Y)*(yMax-yMin)/2;
    }

    // Compute real offset from end effector to inital target point
    Eigen::Vector3f currentOffset;
    currentOffset = _x[r]-_xd0[r];

    // Compute desired tip position
    _xd[r] = _xd0[r]+desiredOffset;
    _desiredOffset[r] = desiredOffset;

    std::cerr << "[SurgicalTask]: " << r << ": Desired offset: " << desiredOffset.transpose() << std::endl; 
    std::cerr << "[SurgicalTask]: " << r << ": Current offset: " << currentOffset.transpose() << std::endl; 

    // To start accounting for the human input, the desired and real offset should be close
    // at the beginning
    if((desiredOffset-currentOffset).norm()<0.02f)
    {
      _inputAlignedWithOrigin[r]=true;
    }

    if(_inputAlignedWithOrigin[r]==false)
    {
      desiredOffset.setConstant(0.0f);
      _xd[r] = _xd0[r];
      std::cerr << "[SurgicalTask]: " << "REAL AND DESIRED DO NOT MATCH INITIALLY" << std::endl;
    }
    
    float alpha = _trocarSpaceLinearDSFixedGain+_trocarSpaceLinearDSGaussianGain*std::exp(-(desiredOffset-currentOffset).squaredNorm()/(2.0f*std::pow(_trocarSpaceLinearDSGaussianWidth,2.0f)));  
    
    std::cerr << "[SurgicalTask]: " << r << ": alpha: " << alpha << std::endl;
    if(_controlStrategy[r] == JOINT_IMPEDANCE)
    {
        _vdTool[r] = alpha*(_xd[r]-_xIK[r]);        
    }
    else
    {
      _vdTool[r] = alpha*(_xd[r]-_x[r]);        
    }
    
    if(_humanInputDevice[r] == JOYSTICK)
    {
      _selfRotationCommand[r] = 0.0f;
    }
    else
    {
      if(_selfRotationMapping[r] == POSITION_POSITION)
      {
        _selfRotationCommand[r] = _trocarSpaceSelfRotationGain*(_trocarInput[r](SELF_ROTATION)*_trocarSpaceSelfRotationRange*M_PI/180.0f-_currentJoints[r](6));
        _selfRotationCommand[r]  = Utils<float>::bound(_selfRotationCommand[r] ,-_toolTipSelfAngularVelocityLimit, _toolTipSelfAngularVelocityLimit);        
      }
      else
      {
        _selfRotationCommand[r] =_trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[r](W_SELF_ROTATION);
        if(_currentJoints[r](6)>_trocarSpaceSelfRotationRange && _selfRotationCommand[r]>0.0f)
        {
          _selfRotationCommand[r] = 0.0f;
        }
        else if(_currentJoints[r](6)<-_trocarSpaceSelfRotationRange && _selfRotationCommand[r]<0.0f)
        {
          _selfRotationCommand[r] = 0.0f;
        }
      }
    }
  }
  else
  {
    _vdTool[r].setConstant(0.0f);
  }
}


void SurgicalTask::initializeBeliefs(int r)
{
  // Find closest attractor and set the belief of the corresponding task to 1
  Eigen::VectorXf temp;
  temp.resize(_nbTasks);

  for(int k = 0; k < _nbTasks; k++)
  {
    // if(k == 0)
    // {
      temp(k) = (Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_pillarsPosition.row(k).transpose()-_x[r])).norm();
    // }
    // else if(k==1)
    // {
    //   temp(k) = (Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_humanToolPosition[LEFT]-_x[r])).norm();
    // }
    // else if(k==2)
    // {
    //   temp(k) = (Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_humanToolPosition[RIGHT]-_x[r])).norm(); 
    // }
  }

  Eigen::MatrixXf::Index indexMin;
  float minValue = temp.array().minCoeff(&indexMin);
  _beliefsC.setConstant(0.0f);
  _beliefsC(indexMin)= 1.0f;

  if(_controlStrategy[r] == PASSIVE_DS)
  {
    _d = (_x[r]-_pillarsPosition.row(indexMin).transpose()).norm();
  }
  else if(_controlStrategy[r] == JOINT_IMPEDANCE)
  {
    _d = (_xIK[r]-_pillarsPosition.row(indexMin).transpose()).norm();      
  }
}


void SurgicalTask::taskAdaptation(int r)
{
  ////////////////////////////////////
  // Compute human desired velocity //
  ////////////////////////////////////
  Eigen::Vector3f vH;

  vH = _wRb[r].col(V_UP)*_trocarInput[r](V_UP)+_wRb[r].col(V_RIGHT)*_trocarInput[r](V_RIGHT);
  
  std::cerr << "[SurgicalTask]: " << r << ": Human input: " << vH.transpose() << std::endl; 

  /////////////////////
  // Task adaptation //
  /////////////////////

  // Initialize desired velocity to zero
  _vda.setConstant(0.0f);

  Eigen::MatrixXf vdk;
  vdk.resize(_nbTasks,3);

  Eigen::MatrixXf xAk, errork;
  xAk.resize(_nbTasks,3);
  errork.resize(_nbTasks,3);


  for(int k = 0; k < _nbTasks; k++)
  {
    xAk.row(k) = _pillarsPosition.row(k);
  }

  // Fill task attractors
  // xAk.row(1) = _pillarsPosition.row(1);
  // xAk.row(1) = _humanToolPosition[LEFT];
  // xAk.row(2) = _humanToolPosition[RIGHT];

  // Update offset distance from tool tip to attractor with max belief
  Eigen::MatrixXf::Index indexMax;
  _beliefsC.array().maxCoeff(&indexMax);
  if(std::fabs(1.0f-_beliefsC[indexMax])<FLT_EPSILON && std::fabs(_trocarInput[r](V_INSERTION))>FLT_EPSILON)
  {
    if(_controlStrategy[r] == PASSIVE_DS)
    {
      _d = (_x[r]-xAk.row(indexMax).transpose()).norm();
    }
    else if(_controlStrategy[r] == JOINT_IMPEDANCE)
    {
      _d = (_xIK[r]-xAk.row(indexMax).transpose()).norm();      
    }
  }
  std::cerr << "[SurgicalTask]: " << r << " d: " << _d << " " << std::fabs(1.0f-_beliefsC[indexMax]) << std::endl; 


  // Update task velocities
  for(int k = 0; k < _nbTasks; k++)
  {
    Eigen::Vector3f offset;
    offset = _d*(_trocarPosition[r]-xAk.row(k).transpose()).normalized();

    if(_controlStrategy[r] == PASSIVE_DS)
    {
      errork.row(k) = (xAk.row(k).transpose()+offset-_x[r]).transpose();
    }
    else if(_controlStrategy[r] == JOINT_IMPEDANCE)
    {
      errork.row(k) = (xAk.row(k).transpose()+offset-_xIK[r]).transpose();
    }
    
    // Compute exponential gain
    float alpha = _trocarSpaceLinearDSFixedGain+_trocarSpaceLinearDSGaussianGain*std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(_trocarSpaceLinearDSGaussianWidth,2.0f))); 
    // float alpha = (2.0f+4.0f*std::exp(-errork.row(k).squaredNorm()/(0.03f*0.03f))); 
    // float alpha = 2.0f; 

    vdk.row(k) = alpha*errork.row(k);
    std::cerr << "[SurgicalTask]: " << r << " Target " << k << " alpha: " << alpha << std::endl;

    // Compute desired task adapted velocity
    _vda+=_beliefsC(k)*vdk.row(k).transpose();
  }

  float a, b, c;

  for(int k = 0; k < _nbTasks; k++)
  {
    // Belief update based on human input
    // a = adaptationRate*(vH.dot(temp.normalized());
    // a = 5.0f*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(0.03,2.0f))))*vH.dot(vdk.row(k).normalized());
    a = _taskAdaptationAlignmentGain*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(_taskAdaptationGaussianWidth,2.0f))))*vH.dot(vdk.row(k).normalized());
    // a = (1.0f-_beliefsC(k))*vH.dot(vdk.row(k).normalized());

    // Make belief converging to 0 or 1
    // b = 50.0f*((_beliefsC(k)-0.5f)*vdk.row(k).squaredNorm());
    b = _taskAdaptationConvergenceGain*(_beliefsC(k)-0.5f);
    // b = _beliefsC(k)-0.5f;

    // Belief update to select the closest ine if has same directions
    c = _taskAdaptationProximityGain*std::exp(-_taskAdaptationExponentialGain*(vdk.row(k)).norm()*vH.norm());
    // c = 0.0f;

    _dbeliefsC(k) = _taskAdaptationOverallGain*(a+b+c);

    std::cerr << "[SurgicalTask]: " << r << ": Dbeliefs target" << k << ": " << a << " " << b <<  " " << c << " " << a+b+c << " " << errork.row(k).norm() << std::endl;
  }

  // std::cerr << r << ": a: " << _dbeliefsC.transpose() << std::endl;
  float dbmax = _dbeliefsC.array().maxCoeff(&indexMax);

  if(std::fabs(1.0f-_beliefsC(indexMax))< FLT_EPSILON && std::fabs(_beliefsC.sum()-1)<FLT_EPSILON)
  {
    _dbeliefsC.setConstant(0.0f);
  }
  else
  {
    Eigen::VectorXf temp;
    temp.resize(_nbTasks-1);
    temp.setConstant(0.0f);
    int m = 0;
    for(int k = 0; k < _nbTasks; k++)
    {
      if(k!=indexMax)
      {
        temp(m) = _dbeliefsC(k);
        m++;
      }
    }
    float db2max = temp.array().maxCoeff();

    float z = (dbmax+db2max)/2.0f;
    _dbeliefsC.array() -= z;
    std::cerr << "[SurgicalTask]: " << r << ": Before Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

    float S = 0.0f;
    for(int k = 0; k < _nbTasks; k++)
    {
      // if(fabs(_beliefsC(k))>FLT_EPSILON || _dbeliefsC(k) > 0)
      {
        S += _dbeliefsC(k);
      }
    }
    _dbeliefsC(indexMax)-=S;

  }
  std::cerr << "[SurgicalTask]: " << r << ": After Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

  std::cerr << _dbeliefsC.sum() << std::endl;

  _beliefsC+=_dt*_dbeliefsC;
  for(int k = 0; k < _nbTasks; k++)
  {
    _beliefsC(k) = Utils<float>::bound(_beliefsC(k),0.0f,1.0f);
  }

  _beliefsC /= _beliefsC.sum();

  std::cerr << "[SurgicalTask]: " << r << ": Beliefs: " << _beliefsC.transpose() << std::endl;

  _vda.setConstant(0.0f);
  for(int k = 0; k < _nbTasks; k++)
  {
    _vda += _beliefsC(k)*vdk.row(k);
  }

  // _vda = Utils<float>::bound(_fx[r],0.3f);
}




void* SurgicalTask::startIkLoop(void* ptr)
{
    reinterpret_cast<SurgicalTask *>(ptr)->ikLoop(); 
}

void SurgicalTask::ikLoop()
{
  while(_startThread)
  {

    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_robotMode[r]==TROCAR_SPACE)
      {
          Eigen::VectorXf joints(7);

          std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

          bool res = _qpSolverRCM[r].step(joints, _currentJoints[r], _trocarPosition[r],
          _toolOffsetFromEE[r], _xdTool[r], _currentJoints[r](6), 0.1f, _xRobotBaseOrigin[r]);

          _ikJoints[r] = joints;
          std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

          std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        // std::cerr << "[thread]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
        // std::cerr << "[thread]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
        // std::cerr << "[thread]: " << r <<": vd: " << vdTool.transpose() << std::endl; 
        std::cerr << "[thread]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
        std::cerr << "[thread]: " << r << ": Desired joints: " << joints.transpose() << std::endl;
        std::cerr << "[thread]: " << r << ": Error joints: " << (joints-_currentJoints[r]).norm() << std::endl;
        std::cerr << "[thread]: " << (int) res <<  " Elasped time in [micro s]: " << duration.count() << std::endl;
        std::cerr << "[thread]: " << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
        std::cerr << "[thread]: " <<  "x: " << _x[r].transpose() << std::endl;
        int dt = int(_dt*1e6)-duration.count();
        std::this_thread::sleep_for(std::chrono::microseconds(int(std::max(dt,0))));
      }
      else
      {
        _ikJoints[r] = _currentJoints[r];
      }
    }
  }
  std::cerr << "END thread" << std::endl;
}



void SurgicalTask::humanInputTransformation()
{
  Eigen::Matrix<float,5,5> R;

  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {
      if(_humanInputDevice[r] == FOOT)
      {
        if(_linearMapping[r]==POSITION_POSITION)
        {
          // X, Y, Z, SELF_ROTATION, EXTRA_DOF <= Y, -X, PITCH, -YAW, ROLL
          R << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
               -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
               0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
               0.0f, 0.0f, 0.0f, 1.0f, 0.0f;
          _trocarInput[r] = R*_footPose[r];

          // Scale human input between -1 and 1
          _trocarInput[r](X) = Utils<float>::bound(2*_trocarInput[r](X)/_footInterfaceRange[r][FOOT_Y], -1.0f, 1.0f);
          _trocarInput[r](Y) = Utils<float>::bound(2*_trocarInput[r](Y)/_footInterfaceRange[r][FOOT_X], -1.0f, 1.0f);
          _trocarInput[r](Z) = Utils<float>::bound(2*_trocarInput[r](Z)/_footInterfaceRange[r][FOOT_PITCH], -1.0f, 1.0f);
          // If a linear position-position mapping is desired for the foot we allow for a poosition-position or position-velocity
          // mapping for the self rotation 
          if(_selfRotationMapping[r]==POSITION_POSITION)
          {
            _trocarInput[r](SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[r](SELF_ROTATION)/_footInterfaceRange[r][FOOT_YAW], -1.0f, 1.0f);
          }
          else
          {
            _trocarInput[r](W_SELF_ROTATION) = Utils<float>::deadZone(_trocarInput[r](W_SELF_ROTATION), -_footInterfaceDeadZone[FOOT_YAW], _footInterfaceDeadZone[FOOT_YAW]);
            _trocarInput[r](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[r](W_SELF_ROTATION)/(_footInterfaceRange[r][FOOT_YAW]-2*_footInterfaceDeadZone[FOOT_YAW]), -1.0f, 1.0f);
          }
          _trocarInput[r](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[r](EXTRA_DOF)/_footInterfaceRange[r][FOOT_ROLL], -1.0f, 1.0f);

        }
        else
        {
          // V_UP, V_RIGHT, V_INSERTION, W_SELF_ROTATION <= PITCH, -ROLL, Y, -YAW
          R << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
               0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
               1.0f, 0.0f, 0.0f, 0.0f, 0.0f;
          _trocarInput[r] = R*_footPose[r];

          // Apply deadzone on foot position
          _trocarInput[r](V_UP) = Utils<float>::deadZone(_trocarInput[r](V_UP), -_footInterfaceDeadZone[FOOT_PITCH], _footInterfaceDeadZone[FOOT_PITCH]);
          _trocarInput[r](V_RIGHT) = Utils<float>::deadZone(_trocarInput[r](V_RIGHT), -_footInterfaceDeadZone[FOOT_ROLL], _footInterfaceDeadZone[FOOT_ROLL]);
          _trocarInput[r](V_INSERTION) = Utils<float>::deadZone(_trocarInput[r](V_INSERTION), -_footInterfaceDeadZone[FOOT_Y], _footInterfaceDeadZone[FOOT_Y]);
          _trocarInput[r](W_SELF_ROTATION) = Utils<float>::deadZone(_trocarInput[r](W_SELF_ROTATION), -_footInterfaceDeadZone[FOOT_YAW], _footInterfaceDeadZone[FOOT_YAW]);
          _trocarInput[r](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[r](EXTRA_DOF)/_footInterfaceRange[r][FOOT_X], -1.0f, 1.0f);

          // Scale human input between -1 and 1
          _trocarInput[r](V_UP) = Utils<float>::bound(2*_trocarInput[r](V_UP)/(_footInterfaceRange[r][FOOT_PITCH]-2*_footInterfaceDeadZone[FOOT_PITCH]), -1.0f, 1.0f);
          _trocarInput[r](V_RIGHT) = Utils<float>::bound(2*_trocarInput[r](V_RIGHT)/(_footInterfaceRange[r][FOOT_ROLL]-2*_footInterfaceDeadZone[FOOT_ROLL]), -1.0f, 1.0f);
          _trocarInput[r](V_INSERTION) = Utils<float>::bound(2*_trocarInput[r](V_INSERTION)/(_footInterfaceRange[r][FOOT_Y]-2*_footInterfaceDeadZone[FOOT_Y]), -1.0f, 1.0f);
          _trocarInput[r](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[r](W_SELF_ROTATION)/(_footInterfaceRange[r][FOOT_YAW]-2*_footInterfaceDeadZone[FOOT_YAW]), -1.0f, 1.0f);
          _trocarInput[r](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[r](EXTRA_DOF)/_footInterfaceRange[r][FOOT_X], -1.0f, 1.0f);

          // // V_UP, V_RIGHT, V_INSERTION, W_SELF_ROTATION <= Y, -X, PITCH, -YAW
          // R << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
          //      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
          //      0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
          //      0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
          //      0.0f, 0.0f, 0.0f, 1.0f, 0.0f;
          // _trocarInput[r] = R*_footPose[r];

          // // Apply deadzone on foot position
          // _trocarInput[r](V_UP) = Utils<float>::deadZone(_trocarInput[r](V_UP), -_footInterfaceDeadZone[FOOT_Y], _footInterfaceDeadZone[FOOT_Y]);
          // _trocarInput[r](V_RIGHT) = Utils<float>::deadZone(_trocarInput[r](V_RIGHT), -_footInterfaceDeadZone[FOOT_X], _footInterfaceDeadZone[FOOT_X]);
          // _trocarInput[r](V_INSERTION) = Utils<float>::deadZone(_trocarInput[r](V_INSERTION), -_footInterfaceDeadZone[FOOT_PITCH], _footInterfaceDeadZone[FOOT_PITCH]);
          // _trocarInput[r](W_SELF_ROTATION) = Utils<float>::deadZone(_trocarInput[r](W_SELF_ROTATION), -_footInterfaceDeadZone[FOOT_YAW], _footInterfaceDeadZone[FOOT_YAW]);
          // _trocarInput[r](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[r](EXTRA_DOF)/_footInterfaceRange[r][FOOT_ROLL], -1.0f, 1.0f);

          // // Scale human input between -1 and 1
          // _trocarInput[r](V_UP) = Utils<float>::bound(2*_trocarInput[r](V_UP)/(_footInterfaceRange[r][FOOT_Y]-2*_footInterfaceDeadZone[FOOT_Y]), -1.0f, 1.0f);
          // _trocarInput[r](V_RIGHT) = Utils<float>::bound(2*_trocarInput[r](V_RIGHT)/(_footInterfaceRange[r][FOOT_X]-2*_footInterfaceDeadZone[FOOT_X]), -1.0f, 1.0f);
          // _trocarInput[r](V_INSERTION) = Utils<float>::bound(2*_trocarInput[r](V_INSERTION)/(_footInterfaceRange[r][FOOT_PITCH]-2*_footInterfaceDeadZone[FOOT_PITCH]), -1.0f, 1.0f);
          // _trocarInput[r](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[r](W_SELF_ROTATION)/(_footInterfaceRange[r][FOOT_YAW]-2*_footInterfaceDeadZone[FOOT_YAW]), -1.0f, 1.0f);
          // _trocarInput[r](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[r](EXTRA_DOF)/_footInterfaceRange[r][FOOT_ROLL], -1.0f, 1.0f);

        }
      }
      else
      {

        if(_linearMapping[r] == POSITION_POSITION)
        {
          R <<  0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 1.0f, 0.0f;          
        }
        else
        {
          R <<  0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 0.0f, 1.0f, 0.0f;   
        }
        _trocarInput[r] = R*_footPose[r];
      }
    }
  }
}


void SurgicalTask::computeHapticFeedback(int r)
{
  switch(_linearMapping[r])
  {
    case POSITION_VELOCITY:
    {
      _FdFoot[r].setConstant(0.0f);
      break;
    }
    case POSITION_POSITION:
    {
      _FdFoot[r].setConstant(0.0f);
      if(_robotMode[r] == TROCAR_SPACE && _inputAlignedWithOrigin[r]==false)
      {
        _FdFoot[r] = Utils<float>::bound(200.0f*(_x[r]-(_xd0[r]+_desiredOffset[r])),15.0f)+2.0f*_wRb[r]*_filteredWrench[r].segment(0,3);   
        // _FdFoot[r] = _wRb[r]*_filteredWrench[r].segment(0,3);   
      }
      else if(_robotMode[r] == TROCAR_SPACE && _inputAlignedWithOrigin[r]==true)
      {
        _FdFoot[r] = 2.0f*_wRb[r]*_filteredWrench[r].segment(0,3);
      }
      break;
    }
    default:
    {
      _FdFoot[r].setConstant(0.0f);
      break;
    }
  }
}


void SurgicalTask::computeDesiredFootWrench(int r)
{
  _desiredFootWrench[r].setConstant(0.0f);

  Eigen::Matrix<float,5,3> GammaF;
  Eigen::Matrix3f G;
  G.setIdentity();
  G(2,2) = 0.2;
  Eigen::Matrix<float,5,3> P;
  P <<   0.0f, -1.0f, 0.0f,
         1.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 1.0f,
         0.0f, 0.0f, 0.0f,
         0.0f, 0.0f, 0.0f;

  GammaF = P*G;

  _desiredFootWrench[r] = GammaF*_FdFoot[r];        

  for(int k = 0; k < 2; k++)
  {
    if(_desiredFootWrench[r](k)>15.0f)
    {
      _desiredFootWrench[r](k) = 15.0f;
    }
    else if(_desiredFootWrench[r](k)<-15.0f)
    {
      _desiredFootWrench[r](k) = -15.0f;
    }
  }

  for(int k = 0 ; k < 3; k++)
  {
    if(_desiredFootWrench[r](k+2)>2.0f)
    {
      _desiredFootWrench[r](k+2) = 2.0f;
    }
    else if(_desiredFootWrench[r](k+2)<-2.0f)
    {
      _desiredFootWrench[r](k+2) = -2.0f;
    }
  }
}



void SurgicalTask::logData()
{
 _outputFile << ros::Time::now() << " "
             << _markersPosition.col(LEFT_HUMAN_TOOL).transpose() << " "  
             << _markersQuaternion.col(LEFT_HUMAN_TOOL).transpose() << " "
             << _offsetTool.transpose() << std::endl;
}


void SurgicalTask::publishData()
{
  static int count = 0;
  if(count < 10)
  {
    count++;
  }
  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {
      if(_controlStrategy[r] == JOINT_IMPEDANCE)
      {

        std_msgs::Float64MultiArray msg;
        msg.data.resize(7);
        _msgStiffness.data.resize(7);

        if(!_firstPublish[r])
        {
          for(int m = 0; m < 7; m++)
          {
            msg.data[m] = _currentJoints[r](m);
            _msgStiffness.data[m] = 0.0f;
          }

          if(count==10)
          {
            _firstPublish[r] = true;
          }

        }
        else
        {
          for(int m = 0; m < 7; m++)
          {
            msg.data[m] = _ikJoints[r](m);
            _msgStiffness.data[m] = _stiffness[r];
          }          
        }

      
        _pubDesiredJoints[r].publish(msg);
        _pubStiffness[r].publish(_msgStiffness);
      }
      else if(_controlStrategy[r] == PASSIVE_DS)
      {
        _msgDesiredTwist.linear.x  = _vd[r](0);
        _msgDesiredTwist.linear.y  = _vd[r](1);
        _msgDesiredTwist.linear.z  = _vd[r](2);

        // Convert desired end effector frame angular velocity to world frame
        _msgDesiredTwist.angular.x = _omegad[r](0);
        _msgDesiredTwist.angular.y = _omegad[r](1);
        _msgDesiredTwist.angular.z = _omegad[r](2);

        _pubDesiredTwist[r].publish(_msgDesiredTwist);

        // Publish desired orientation
        _msgDesiredOrientation.w = _qd[r](0);
        _msgDesiredOrientation.x = _qd[r](1);
        _msgDesiredOrientation.y = _qd[r](2);
        _msgDesiredOrientation.z = _qd[r](3);

        _pubDesiredOrientation[r].publish(_msgDesiredOrientation);
      }

      if(_humanInputDevice[r] == FOOT)
      {
        for(int m = 0; m < 5; m++)
        {
          _msgFootInput.ros_effort[m] = _desiredFootWrench[r](m);
          _msgFootInput.ros_filterAxisForce[m] = 1.0f;
        }
        _pubFootInput[r].publish(_msgFootInput);      
      }


      geometry_msgs::Wrench wrench;
      wrench.force.x = _nullspaceWrench[r](0);
      wrench.force.y = _nullspaceWrench[r](1);
      wrench.force.z = _nullspaceWrench[r](2);
      wrench.torque.x = _nullspaceWrench[r](3);
      wrench.torque.y = _nullspaceWrench[r](4);
      wrench.torque.z = _nullspaceWrench[r](5);
      _pubDesiredWrench[r].publish(wrench);

      _msgNullspaceCommand.data.resize(7);
      for(int m = 0; m < 7; m++)
      {
        _msgNullspaceCommand.data[m] = _nullspaceCommand[r](m);
      }

      Eigen::Vector3f F, W;
      F = _wRb[r]*_filteredWrench[r].segment(0,3);
      W = _wRb[r]*_filteredWrench[r].segment(3,3);
      wrench.force.x = F(0);
      wrench.force.y = F(1);
      wrench.force.z = F(2);
      wrench.torque.x = W(0);
      wrench.torque.y = W(1);
      wrench.torque.z = W(2);
      _pubFilteredWrench[r].publish(wrench);


      // _pubNullspaceCommand[r].publish(_msgNullspaceCommand);

      // std_msgs::Float64MultiArray msgRobotData;
      // msgRobotData.data.resize(9);

      // for(int m = 0; m < 3; m++)
      // {
      //   msgRobotData.data[m] = _xRobotBaseOrigin[r](m);
      //   msgRobotData.data[m+3] = 0.0f;
      //   msgRobotData.data[m+6] = _trocarPosition[r](m);
      // }
      // msgRobotData.data[5] = _toolOffsetFromEE[r];
      // _pubRobotData[r].publish(msgRobotData);

      if(r==RIGHT)
      {
        _msgGripperInput.ros_dPosition = _desiredGripperPosition[r];
        _pubGripper.publish(_msgGripperInput);
      }
    }
  }
  _msgSurgicalTaskState.robotMode[LEFT] = _robotMode[LEFT];
  _msgSurgicalTaskState.robotMode[RIGHT] = _robotMode[RIGHT];
  _pubSurgicalTaskState.publish(_msgSurgicalTaskState);
}


void SurgicalTask::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int r)
{
  Eigen::Vector3f temp = _x[r];

  // Update end effecotr pose (position+orientation)
  _xEE[r] << msg->position.x, msg->position.y, msg->position.z;
  _q[r] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[r] = Utils<float>::quaternionToRotationMatrix(_q[r]);
  _x[r] = _xEE[r]+_toolOffsetFromEE[r]*_wRb[r].col(2);

  if((!_useSim && (!_useOptitrack ||  _optitrackInitialized)) ||_firstRobotBaseFrame[r])
  {
    _xEE[r] += _xRobotBaseOrigin[r];
    _x[r] += _xRobotBaseOrigin[r];
  }

  if(!_firstRobotPose[r])
  {
    if((!_useSim && (!_useOptitrack ||  _optitrackInitialized))  || _firstRobotBaseFrame[r])
    {
      _firstRobotPose[r] = true;
      _xd[r] = _x[r];
      _qd[r] = _q[r];
      _vd[r].setConstant(0.0f);
      _trocarPosition[r] = _x[r];
      _trocarOrientation[r] = _wRb[r].col(2);
    }
  }
}


void SurgicalTask::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int r)
{
  _v[r] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[r] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[r])
  {
    _firstRobotTwist[r] = true;
  }
}
 

void SurgicalTask::updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int r)
{
  _footPose[r].setConstant(0.0f);
  // _footPose[r](X) = msg->axes[0];
  // _footPose[r](Y) = msg->axes[1];
  // // _footPose[r](PITCH) = msg->axes[4];
  // // _footPose[r](YAW) = (-msg->axes[5]+1.0f)/2.0f-(-msg->axes[2]+1.0f)/2.0f;
  // _footPose[r](PITCH) = msg->axes[3];
  // _footPose[r](ROLL) = msg->axes[2];
  // _footPose[r](YAW) = (-msg->axes[12]+1.0f)/2.0f-(-msg->axes[13]+1.0f)/2.0f;

  _footPose[r](FOOT_X) = msg->axes[0];
  _footPose[r](FOOT_Y) = msg->axes[1];
  _footPose[r](FOOT_PITCH) = msg->axes[4];
  // _footPose[r](FOOT_YAW) = (-msg->axes[5]+1.0f)/2.0f-(-msg->axes[2]+1.0f)/2.0f;
  _footPose[r](FOOT_PITCH) = msg->axes[5];
  _footPose[r](FOOT_ROLL) = msg->axes[2];
  _footPose[r](FOOT_YAW) = (-msg->axes[3]+1.0f)/2.0f-(-msg->axes[4]+1.0f)/2.0f;

  if(msg->buttons[4] && !msg->buttons[5])
  {
    _msgGripperInput.ros_dPosition = 0;
  }
  else if(!msg->buttons[4] && msg->buttons[5])
  {
    _msgGripperInput.ros_dPosition = 20;
  }

  if(!_firstHumanInput[r])
  {
    _firstHumanInput[r]= true;
  }
}

void SurgicalTask::updateFootOutput(const custom_msgs::FootOutputMsg_v3::ConstPtr& msg, int r)
{
  for(int m = 0; m < 5; m++)
  {
    _footPose[r](m) = msg->platform_position[m];
    _footTwist[r](m) = msg->platform_speed[m];
    _footWrench[r](m) = msg->platform_effortD[m];
  }
  _footPose[r] -= _footOffset[r];
  _footState[r] = msg->platform_machineState;


  if(_firstFootSharedGrasping[r])
  {
    for(int m = 0; m < 5; m++)
    {
      _footPoseFiltered[r](m) = (1.0f-_filterGainFootAxis[r](m))*_footPoseFiltered[r](m)
                              + _filterGainFootAxis[r](m)* _footPose[r](m);
    }
    _footPose[r] = _footPoseFiltered[r];
    // std::cerr << "Non filtered" << _footPose[r].transpose() << std::endl;
    // std::cerr << "Filtered" << _footPoseFiltered[r].transpose() << std::endl;
  }

  if(!_firstHumanInput[r])
  {
    _firstHumanInput[r] = true;
  }
}

void SurgicalTask::updateFootSharedGrasping(const custom_msgs_gripper::SharedGrasping::ConstPtr& msg, int r)
{
  for(int m = 0; m < 5; m++)
  {
    _filterGainFootAxis[r](m) = msg->sGrasp_hFilters[m];

  }

  if(!_firstFootSharedGrasping[r])
  {
    _firstFootSharedGrasping[r] = true;
  }
}


void SurgicalTask::updateGripperOutput(const custom_msgs_gripper::GripperOutputMsg::ConstPtr& msg)
{
  _msgGripperOutput = *msg;

  if(!_firstGripper)
  {
    _firstGripper = true;
  }
}


void SurgicalTask::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int r)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

      // std::cerr << "[SurgicalTask]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;

  if(!_wrenchBiasOK[r] && _firstRobotPose[r])
  {
    Eigen::Vector3f loadForce = _wRb[r].transpose()*_toolMass[r]*_gravity;
    _wrenchBias[r].segment(0,3) -= loadForce;
    _wrenchBias[r].segment(3,3) -= _toolComPositionFromSensor[r].cross(loadForce);
    _wrenchBias[r] += raw; 
    _wrenchCount[r]++;
    if(_wrenchCount[r]==NB_SAMPLES)
    {
      _wrenchBias[r] /= NB_SAMPLES;
      _wrenchBiasOK[r] = true;
      std::cerr << "[SurgicalTask]: Bias " << r << ": " <<_wrenchBias[r].transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK[r] && _firstRobotPose[r])
  {
    _wrench[r] = raw-_wrenchBias[r];
    Eigen::Vector3f loadForce = _wRb[r].transpose()*_toolMass[r]*_gravity;
    _wrench[r].segment(0,3) -= loadForce;
    _wrench[r].segment(3,3) -= _toolComPositionFromSensor[r].cross(loadForce);
    _filteredWrench[r] = _filteredForceGain*_filteredWrench[r]+(1.0f-_filteredForceGain)*_wrench[r];
  }
}


void SurgicalTask::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int r) 
{
  if(!_firstDampingMatrix[r])
  {
    _firstDampingMatrix[r] = true;
  }

  _D[r] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void SurgicalTask::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int r) 
{
  for(int m = 0; m < 7; m++)
  {
    _currentJoints[r](m) = msg->position[m];
  }

  if(_useFranka)
  {
    Eigen::Matrix4f H;
    H = Utils<float>::getForwardKinematics(_currentJoints[r], _robotID);
    _xEE[r] = _wRRobotBasis[r]*H.block(0,3,3,1)+_xRobotBaseOrigin[r];
    _wRb[r] = _wRRobotBasis[r]*H.block(0,0,3,3);
    _q[r] = Utils<float>::rotationMatrixToQuaternion(_wRb[r]);
    _x[r] = _xEE[r]+_toolOffsetFromEE[r]*_wRb[r].col(2);
  }

  if(!_firstJointsUpdate[r])
  {
    _firstJointsUpdate[r] = true;
    _ikJoints[r] = _currentJoints[r];
  }
}


void SurgicalTask::updateOptitrackPose(const geometry_msgs::PoseStamped::ConstPtr& msg, int k) 
{
  if(!_firstOptitrackPose[k])
  {
    _firstOptitrackPose[k] = true;
  }

  _markersSequenceID(k) = msg->header.seq;
  _markersTracked(k) = checkTrackedMarker(_markersPosition.col(k)(0),msg->pose.position.x);
  _markersPosition.col(k) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _markersPosition.col(k) = _Roptitrack*_markersPosition.col(k);  
  _markersQuaternion.col(k) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

  if(k == (int) RIGHT_ROBOT_BASIS || k == (int) LEFT_ROBOT_BASIS)
  {
    _markersPosition.col(k)(2) -= 0.03f;
  }
}


uint16_t SurgicalTask::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}


void SurgicalTask::optitrackInitialization()
{
  if(_optitrackCount< NB_OPTITRACK_SAMPLES)
  {
    if(_markersTracked(RIGHT_ROBOT_BASIS) && _markersTracked(LEFT_ROBOT_BASIS))
    {
      _markersPosition0 = (_optitrackCount*_markersPosition0+_markersPosition)/(_optitrackCount+1);
      _optitrackCount++;
    }
    std::cerr << "[ObjectGrasping]: Optitrack Initialization count: " << _optitrackCount << std::endl;
    if(_optitrackCount == 1)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization starting ...");
    }
    else if(_optitrackCount == NB_OPTITRACK_SAMPLES)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization done !");
      _xRobotBaseOrigin[LEFT] = _markersPosition0.col(LEFT_ROBOT_BASIS)-_markersPosition0.col(RIGHT_ROBOT_BASIS);
    }
  }
  else
  {
    _optitrackInitialized = true;
  }
}


void SurgicalTask::registerTrocars()
{
  if(!_trocarsRegistered[LEFT] || !_trocarsRegistered[RIGHT])
  {
    static struct termios oldSettings, newSettings;
    tcgetattr( STDIN_FILENO, &oldSettings);           // save old settings
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON);                 // disable buffering     
    newSettings.c_cc[VMIN] = 0; 
    newSettings.c_cc[VTIME] = 0; 
    tcsetattr( STDIN_FILENO, TCSANOW, &newSettings);  // apply new settings

    int c = getchar();  // read character (non-blocking)
    int add[NB_ROBOTS];
    add[LEFT] = 'a';
    add[RIGHT] = 'b';
    int finish[NB_ROBOTS];
    finish[LEFT] = 'l';
    finish[RIGHT] = 'r';

    Eigen::VectorXf temp;
    for(int r = 0; r < NB_ROBOTS; r++)
    { 
      if(_useRobot[r])
      {        
        if(c==add[r] && ((!_useFranka && _firstRobotPose[r])|| (_useFranka && _firstJointsUpdate[r])))
        {
          _trocarPosition[r] = _x[r];
          _trocarOrientation[r] = _wRb[r].col(2);
          std::cerr << r << ": Adding trocar: " << _x[r].transpose() << std::endl;         
        }
        else if(c==finish[r])
        {

          if(_trocarPosition[r].norm()>FLT_EPSILON)
          {
            _trocarsRegistered[r] = true;
            std::cerr << r << ": Finished registering" << std::endl;
          }
        }
      }
      else
      {
        _trocarsRegistered[r] = true;
      }    
    }
    tcsetattr( STDIN_FILENO, TCSANOW, &oldSettings);  // restore old settings
  
  }
}


    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      // bool res = _qpSolverRCM[r].step(_ikJoints[r], _currentJoints[r], _trocarPosition[r],
      // _toolOffsetFromEE[r], _x[r], _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r]);

      // _ikJoints[r] = qpSolver(_currentJoints[r], _trocarPosition[r],
      // _toolOffsetFromEE[r], _x[r], _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r]);

      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      // Eigen::VectorXd joints;
      // bool res = _cvxgenSolverRCM.step(joints, _currentJoints[r].cast<double>(), _trocarPosition[r].cast<double>(),
      // (double)_toolOffsetFromEE[r], _x[r].cast<double>(), (double) _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r].cast<double>());
      // _ikJoints[r] = joints.cast<float>();
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // std::cerr << "[SurgicalTask]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": error tool : " << (_x[r]-_xdTool[r]).norm() << std::endl; 
      // std::cerr << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      // std::cerr << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;
      // std::cerr << r << ": Error joints: " << (_ikJoints[r]-_currentJoints[r]).norm() << std::endl;
      // std::cerr << (int) res <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      // std::cerr <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      // std::cerr << (int) res <<  " Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // // std::cerr << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // std::cerr <<  "xd: " << _xdTool[r].transpose() <<  "x: " << _x[r].transpose() << std::endl;


      // Eigen::Matrix<float, 7, 1> joints;
      // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      // bool res = _qpSolverRCM[r].step(_ikJoints[r], _currentJoints[r], _trocarPosition[r],
      // _toolOffsetFromEE[r], _xdTool[r], _currentJoints[r](6), 0.1f, _xRobotBaseOrigin[r]);
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // Eigen::VectorXd joints;
      // bool res = _cvxgenSolverRCM.step(joints, _currentJoints[r].cast<double>(), _trocarPosition[r].cast<double>(),
      // (double)_toolOffsetFromEE[r], _x[r].cast<double>(), (double) _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r].cast<double>());
      // _ikJoints[r] = joints.cast<float>();
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // std::cerr << "[SurgicalTask]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": error tool : " << (_x[r]-_xdTool[r]).norm() << std::endl; 
      // std::cerr << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      // // std::cerr << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;
      // std::cerr << r << ": Error joints: " << (_ikJoints[r]-_currentJoints[r]).norm() << std::endl;
      // std::cerr << (int) res <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      // std::cerr << (int) res <<  " Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // // std::cerr << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // std::cerr <<  "xd: " << _xdTool[r].transpose() <<  "x: " << _x[r].transpose() << std::endl;