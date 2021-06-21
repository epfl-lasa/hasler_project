#include "SurgicalTask.h"


void SurgicalTask::initializeSubscribersAndPublishers()
{
  if(_useRobot[LEFT])
  {
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
      _subRobotExternalWrench[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/left_panda/franka_state_controller/F_ext", 1, boost::bind(&SurgicalTask::updateRobotExternalWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    }
  
    if(_humanInputDevice[LEFT] == JOYSTICK)
    {
      if(_humanInputID[LEFT] == LEFT)
      {
        _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }
      else
      {
        _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }
    }
    else
    {
      if(_humanInputID[LEFT] == LEFT)
      {
        _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
        _subFootInput[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/FI_Input/Left",1, boost::bind(&SurgicalTask::updateFootInput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      
        _subLegState[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/left_leg/leg_joint_publisher/joint_states",1, boost::bind(&SurgicalTask::updateLegState,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootBaseWrench[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench",1, boost::bind(&SurgicalTask::updateFootBaseWrench,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticEfforts[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left/foot_haptic_efforts",1, boost::bind(&SurgicalTask::updateFootHaptics,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootInertiaCoriolisCompensation[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/foot_comp_inertia_coriolis",1, boost::bind(&SurgicalTask::updateFootInertiaCoriolisCompensation,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subLegCompensation[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/leg_comp_platform_effort",1, boost::bind(&SurgicalTask::updateLegCompensation,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootForceSensorModified[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/left_platform/force_sensor_modifier/force_modified",1, boost::bind(&SurgicalTask::updateFootForceSensorModified,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticsData[LEFT] = _nh.subscribe<custom_msgs::FootHapticDataMsg>("/left/foot_haptic_data",1, boost::bind(&SurgicalTask::updateFootHapticsData,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }
      else
      {
        _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
        _subFootInput[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/FI_Input/Right",1, boost::bind(&SurgicalTask::updateFootInput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
        _subLegState[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_leg/leg_joint_publisher/joint_states",1, boost::bind(&SurgicalTask::updateLegState,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootBaseWrench[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/right_leg/leg_joint_publisher/leg_foot_base_wrench",1, boost::bind(&SurgicalTask::updateFootBaseWrench,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticEfforts[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right/foot_haptic_efforts",1, boost::bind(&SurgicalTask::updateFootHaptics,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootInertiaCoriolisCompensation[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/foot_comp_inertia_coriolis",1, boost::bind(&SurgicalTask::updateFootInertiaCoriolisCompensation,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subLegCompensation[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/leg_comp_platform_effort",1, boost::bind(&SurgicalTask::updateLegCompensation,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootForceSensorModified[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/right_platform/force_sensor_modifier/force_modified",1, boost::bind(&SurgicalTask::updateFootForceSensorModified,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticsData[RIGHT] = _nh.subscribe<custom_msgs::FootHapticDataMsg>("/right/foot_haptic_data",1, boost::bind(&SurgicalTask::updateFootHapticsData,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }
    }

    // Publisher definitions
    if(!_useFranka)
    {
      _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/left_lwr/joint_controllers/passive_ds_command_vel", 1);
      _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/left_lwr/joint_controllers/passive_ds_command_orient", 1);
    }
    else
    {
      // _pubDesiredTask[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/passive_ds_controller/taskd", 1);
      _pubDesiredTask[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/surgical_controller/taskd", 1);
    }
    _pubDesiredWrench[LEFT] = _nh.advertise<geometry_msgs::Wrench>("/left_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg>("/left/surgical_task/foot_input", 1);
    _pubToolToFootTorques[LEFT] = _nh.advertise<custom_msgs::FootInputMsg>("/left/surgical_task/tool_to_foot_torques", 1);
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
      
      _pubRobotData[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/passive_ds_robot_data", 1);
    } 

    
    if(!_useFranka)
    {
      _pubStiffness[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/stiffness", 1);
    }
    else
    {
      _pubStiffness[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("/left_panda/joint_impedance_controller/k_gains", 1);
    }

    _pubRobotState[LEFT] = _nh.advertise<surgical_task::RobotStateMsg>("surgical_task/left_robot_state", 1);

  }

  if(_useRobot[RIGHT])
  {
    _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/right_lwr/ee_pose", 1, boost::bind(&SurgicalTask::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/right_lwr/ee_vel", 1, boost::bind(&SurgicalTask::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useFranka)
    {
      _subCurrentJoints[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_lwr/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subCurrentJoints[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_panda/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    }

    _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SurgicalTask::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
   
    if(!_useSim)
    {
      _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&SurgicalTask::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subRobotExternalWrench[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/right_panda/franka_state_controller/F_ext", 1, boost::bind(&SurgicalTask::updateRobotExternalWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

    }
   
    if(_humanInputDevice[RIGHT] == JOYSTICK)
    {
      if(_humanInputID[RIGHT] == LEFT)
      {
        _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }
      else
      {
        _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }    
    }
    else
    {
      if(_humanInputID[RIGHT] == LEFT)
      {
        _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
        _subFootInput[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/FI_Input/Left",1, boost::bind(&SurgicalTask::updateFootInput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      
        _subLegState[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/left_leg/leg_joint_publisher/joint_states",1, boost::bind(&SurgicalTask::updateLegState,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootBaseWrench[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/left_leg/leg_joint_publisher/leg_foot_base_wrench",1, boost::bind(&SurgicalTask::updateFootBaseWrench,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticEfforts[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left/foot_haptic_efforts",1, boost::bind(&SurgicalTask::updateFootHaptics,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootInertiaCoriolisCompensation[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/foot_comp_inertia_coriolis",1, boost::bind(&SurgicalTask::updateFootInertiaCoriolisCompensation,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subLegCompensation[LEFT] = _nh.subscribe<custom_msgs::FootInputMsg>("/left_platform/force_sensor_modifier/leg_comp_platform_effort",1, boost::bind(&SurgicalTask::updateLegCompensation,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootForceSensorModified[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/left_platform/force_sensor_modifier/force_modified",1, boost::bind(&SurgicalTask::updateFootForceSensorModified,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticsData[LEFT] = _nh.subscribe<custom_msgs::FootHapticDataMsg>("/left/foot_haptic_data",1, boost::bind(&SurgicalTask::updateFootHapticsData,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      
      }
      else
      {
        _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
        _subFootInput[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/FI_Input/Right",1, boost::bind(&SurgicalTask::updateFootInput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());      
        _subLegState[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_leg/leg_joint_publisher/joint_states",1, boost::bind(&SurgicalTask::updateLegState,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootBaseWrench[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/right_leg/leg_joint_publisher/leg_foot_base_wrench",1, boost::bind(&SurgicalTask::updateFootBaseWrench,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticEfforts[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right/foot_haptic_efforts",1, boost::bind(&SurgicalTask::updateFootHaptics,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootInertiaCoriolisCompensation[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/foot_comp_inertia_coriolis",1, boost::bind(&SurgicalTask::updateFootInertiaCoriolisCompensation,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subLegCompensation[RIGHT] = _nh.subscribe<custom_msgs::FootInputMsg>("/right_platform/force_sensor_modifier/leg_comp_platform_effort",1, boost::bind(&SurgicalTask::updateLegCompensation,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootForceSensorModified[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/right_platform/force_sensor_modifier/force_modified",1, boost::bind(&SurgicalTask::updateFootForceSensorModified,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());  
        _subFootHapticsData[RIGHT] = _nh.subscribe<custom_msgs::FootHapticDataMsg>("/right/foot_haptic_data",1, boost::bind(&SurgicalTask::updateFootHapticsData,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      }      
    }

    _subGripper = _nh.subscribe("/right/gripperOutput", 1, &SurgicalTask::updateGripperOutput, this, ros::TransportHints().reliable().tcpNoDelay());
    _subGripperAssistance = _nh.subscribe("/right/graspAssistanceOn", 1, &SurgicalTask::updateGripperAssistance, this, ros::TransportHints().reliable().tcpNoDelay());
    _subGripperFeedbackToPlatform = _nh.subscribe("/right/gripperFeedbackToPlatform", 1, &SurgicalTask::updateGripperFeedbackToPlatform, this, ros::TransportHints().reliable().tcpNoDelay());

    _subTaskJoystick = _nh.subscribe("/surgical_task/joy", 1, &SurgicalTask::updateTaskJoystick, this, ros::TransportHints().reliable().tcpNoDelay());

    if(!_useFranka)
    {
      _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/right_lwr/joint_controllers/passive_ds_command_vel", 1);
      _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/right_lwr/joint_controllers/passive_ds_command_orient", 1);
    }
    else
    {
      _pubDesiredTask[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("/right_panda/passive_ds_controller/taskd", 1);
    }
    _pubDesiredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("/right_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg>("/right/surgical_task/foot_input", 1);
    _pubToolToFootTorques[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg>("/right/surgical_task/tool_to_foot_torques", 1);
    _pubNullspaceCommand[RIGHT] = _nh.advertise<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_command_nullspace", 1);

    if(!_useFranka)
    {
      _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/command_joint_pos", 1);
    }
    else
    {
      if(_useSim)
      {
        _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("/right_panda/joint_position_controller/command", 1);
      }
      else
      {
        _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("/right_panda/joint_impedance_controller/qd", 1);
      }
    }   

    if(!_useFranka)
    {
      _pubStiffness[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/stiffness", 1);
    }
    else
    {
      _pubStiffness[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("/right_panda/joint_impedance_controller/k_gains", 1);
    }

    _pubGripper = _nh.advertise<custom_msgs_gripper::GripperInputMsg>("/right/gripperInput", 1);

    _pubRobotState[RIGHT] = _nh.advertise<surgical_task::RobotStateMsg>("surgical_task/right_robot_state", 1);

  }

  _pubSurgicalTaskState = _nh.advertise<surgical_task::SurgicalTaskStateMsg>("surgical_task/state", 1);

  _pubCollisionSpheres = _nh.advertise<visualization_msgs::MarkerArray>("surgical_task/tool_collision", 1);


  if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
  {
    _pubTwoFeetOneTool = _nh.advertise<custom_msgs::TwoFeetOneToolMsg>("mixed_platform/platform_state",1);
  }

  _subOptitrackPose[RIGHT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[RIGHT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

  _subMarkersPosition = _nh.subscribe("/surgical_task/markers_position_transformed", 1, &SurgicalTask::updateMarkersPosition, this, ros::TransportHints().reliable().tcpNoDelay());

  _subTaskManagerState = _nh.subscribe("/task_manager/state", 1, &SurgicalTask::updateTaskManagerState, this, ros::TransportHints().reliable().tcpNoDelay());

}


void SurgicalTask::checkAllSubscribers()
{
  bool robotStatusOK[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatusOK[k] = !_useRobot[k] || (_firstRobotPose[k] &&  _firstRobotTwist[k]
                      // && _firstDampingMatrix[k] 
                      && _firstJointsUpdate[k] 
                      && _firstHumanInput[_humanInputID[k]])
                      && (_useSim || (!_useFTSensor[k] || _wrenchBiasOK[k]))
                      && (_useSim || _wrenchExtBiasOK[k]);

    if(!robotStatusOK[k])
    {
      std::cerr << k << ": Status: " << "use: " << _useRobot[k] 
                << " pose: " << _firstRobotPose[k] << " twist: " << _firstRobotTwist[k]
                << " damp: " << _firstDampingMatrix[k] << " joints: " << _firstJointsUpdate[k]
                << " human: " << _firstHumanInput[_humanInputID[k]]
                << " sim/Ftsensor: " << (_useSim || (!_useFTSensor[k] || _wrenchBiasOK[k]))
                << " sim/Fext: " << (_useSim || _wrenchExtBiasOK[k]) << std::endl;
    }
  }
  
  if(_useSim)
  {
    _trackingOK = true;
  }
  else if(!_useSim && _toolsTracking == OPTITRACK_BASED)
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

    _trackingOK = _optitrackOK && _optitrackInitialized; 
  }
  else if (!_useSim && _toolsTracking == CAMERA_BASED)
  {
    _trackingOK = _firstColorMarkersPosition;

    if(!_trackingOK)
    {
      std::cerr << "First color markers: " << (int) _firstColorMarkersPosition << std::endl;
    }
  }

  if(!_firstTaskManagerState)
  {
    std::cerr << "First task state: " << (int) _firstTaskManagerState << std::endl;
  }

  _allSubscribersOK = robotStatusOK[LEFT] && robotStatusOK[RIGHT] && _trackingOK && _firstTaskManagerState;// && (_useSim ||_firstGripper);
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
            _msgStiffness.data[m] = _stiffness[r](m);
          }          
        }
      
        _pubDesiredJoints[r].publish(msg);
        _pubStiffness[r].publish(_msgStiffness);
      }
      else if(_controlStrategy[r] == PASSIVE_DS)
      {

        if(!_useFranka)
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
        else
        {
          // _msgDesiredTask.data.resize(10);
          // for (int i = 0; i < 3; i++)
          // {
          //   _msgDesiredTask.data[i] = _vd[r](i);
          // }

          // _msgDesiredTask.data[3] = _qd[r](0);

          // for (int i = 0; i < 3; i++)
          // {
          //   _msgDesiredTask.data[4+i] = _qd[r](1+i);
          // }

          // for (int i = 0; i < 3; i++)
          // {
          //   _msgDesiredTask.data[7+i] = _omegad[r](i);
          // }


          _msgDesiredTask.data.resize(13);
          for (int i = 0; i < 3; i++)
          {
            Eigen::Vector3f temp;
            temp = _wRRobotBasis[r].transpose()*_rEERCM[r];
            _msgDesiredTask.data[i] = temp(i);
            temp = _wRRobotBasis[r].transpose()*(_wRb[r]*_toolOffsetFromEE[r]);
            _msgDesiredTask.data[i+3] = temp(i);
            temp = _wRRobotBasis[r].transpose()*(5*(_trocarPosition[r]-_xRCM[r]));
            _msgDesiredTask.data[i+6] = temp(i);
            temp = _wRRobotBasis[r].transpose()*_vdTool[r];
            _msgDesiredTask.data[i+9] = temp(i);
          }

          _msgDesiredTask.data[12] = _selfRotationCommand[r];


          _pubDesiredTask[r].publish(_msgDesiredTask);
        }
      }

      if(_humanInputDevice[_humanInputID[r]] == FOOT)
      {
        for(int m = 0; m < 5; m++)
        {
          _msgFootInput.ros_effort[m] = _desiredFootWrench[_humanInputID[r]](m);
          _msgFootInput.ros_filterAxisForce[m] = 1.0f;
        }
        _pubFootInput[r].publish(_msgFootInput);    

        // for(int m = 0; m < 5; m++)
        // {
        //   _msgToolToFootTorques.ros_effort[m] = _toolToFootTorques[_humanInputID[r]](m);
        //   _msgToolToFootTorques.ros_filterAxisForce[m] = 1.0f;
        // } 
        // _pubToolToFootTorques[r].publish(_msgToolToFootTorques);  

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


      _msgRobotState.controlPhase = _controlPhase[r];
      _msgRobotState.linearMapping = _linearMapping[r];
      _msgRobotState.selfRotationMapping = _selfRotationMapping[r];
      _msgRobotState.dRcmTrocar = (_trocarPosition[r]-_xRCM[r]).norm();
      _msgRobotState.dRcmTip = _dRCMTool[r];
      _msgRobotState.ikRes = _qpResult[r].res;
      _msgRobotState.selfRotationMapping = _selfRotationCommand[r];
      _msgRobotState.eeCollisionConstraintActive = _qpResult[r].eeCollisionConstraintActive;
      _msgRobotState.dEEEE  = _dEECollision[r];
      _msgRobotState.toolCollisionConstraintActive = _qpResult[r].toolCollisionConstraintActive;
      _msgRobotState.dToolTool = _dToolCollision[r];
      _msgRobotState.workspaceCollisionConstraintActive = _qpResult[r].workspaceCollisionConstraintActive;
      _msgRobotState.desiredGripperPosition = _desiredGripperPosition[r];
      _msgRobotState.tankH = _tankH[r];
      _msgRobotState.alphaH = _alphaH[r];

      for(int m = 0; m < 3; m++)
      {
        _msgRobotState.trocarPosition[m] = _trocarPosition[r](m);
        _msgRobotState.tipPosition[m] = _x[r](m);
        _msgRobotState.ikTipPosition[m] = _xIK[r](m);
        _msgRobotState.vdTool[m] = _vdTool[r](m);
        _msgRobotState.currentOffsetFromInsertion[m] = _x[r](m)-_xd0[r](m);
        _msgRobotState.ikOffsetFromInsertion[m] = _xIK[r](m)-_xd0[r](m);
        _msgRobotState.desiredOffsetFromInsertion[m] = _desiredOffsetPPM[r](m);
        Eigen::Vector3f temp;
        temp = -_wRb[r]*_Fext[r];
        _msgRobotState.Fext[m] = _Fext[r](m);
        _msgRobotState.Fm[m] = _Fm[r](m);

      }

      for(int m = 0; m < 5; m++)
      {
        _msgRobotState.humanInput[m] = _trocarInput[_humanInputID[r]](m);
      }

      for(int m = 0; m < 7; m++)
      {
        _msgRobotState.currentJoints[m] = _currentJoints[r](m);
        _msgRobotState.ikJoints[m] = _ikJoints[r](m);
      }

      _pubRobotState[r].publish(_msgRobotState);

    }
  }

  _msgSurgicalTaskState.humanInputMode = _humanInputMode;
  _msgSurgicalTaskState.currentRobot = _currentRobot;
  _msgSurgicalTaskState.useTaskAdaptation = _useTaskAdaptation;
  _msgSurgicalTaskState.beliefsC.resize(_beliefsC.size());
  for(int m = 0; m < _beliefsC.size(); m++)
  {
    _msgSurgicalTaskState.beliefsC[m] = _beliefsC(m);
  }
  _msgSurgicalTaskState.clutching = _clutching;
  _msgSurgicalTaskState.wait = _wait;
  
  for(int m = 0; m < NB_ROBOTS; m++)
  {
    _msgSurgicalTaskState.useRobot[m] = _useRobot[m];
    _msgSurgicalTaskState.tool[m] = _tool[m];
  }

  Eigen::Vector3f retractorPositionC;
  if (_tool[LEFT] == CAMERA)
  {
    retractorPositionC = (_wRb[LEFT]*_eeCameraMapping).transpose()*(_x[RIGHT]-_x[LEFT]);
  }
  else if (_tool[RIGHT] == CAMERA)
  {
    retractorPositionC = (_wRb[RIGHT]*_eeCameraMapping).transpose()*(_x[LEFT]-_x[RIGHT]);
  }

  for(int m = 0 ; m < 3; m++)
  {
    _msgSurgicalTaskState.retractorPositionC[m] = retractorPositionC(m);
  }

  _pubSurgicalTaskState.publish(_msgSurgicalTaskState);

  if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
  {
    if(_currentRobot == LEFT)
    {
      _msgTwoFeetOneTool.currentTool = 2;
    }
    else if(_currentRobot == RIGHT)
    {
      _msgTwoFeetOneTool.currentTool = 1;
    }

    if(_linearMapping[_currentRobot] == POSITION_VELOCITY)
    {
      _msgTwoFeetOneTool.currentControlMode = 1;
    }
    else if(_linearMapping[_currentRobot] == POSITION_POSITION)
    {
      _msgTwoFeetOneTool.currentControlMode = 0;
    }
    _pubTwoFeetOneTool.publish(_msgTwoFeetOneTool);
  }

  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  for(int r =0; r < NB_ROBOTS; r++)
  {
    marker.id = r;
    marker.scale.x = 2.0f*_eeSafetyCollisionRadius;
    marker.scale.y = 2.0f*_eeSafetyCollisionRadius;
    marker.scale.z = 2.0f*_eeSafetyCollisionRadius;
    marker.pose.position.x = _xEE[r](0);
    marker.pose.position.y = _xEE[r](1);
    marker.pose.position.z = _xEE[r](2);
    markerArray.markers.push_back(marker);

    marker.id = r+2;
    marker.scale.x = 2.0f*_toolSafetyCollisionRadius;
    marker.scale.y = 2.0f*_toolSafetyCollisionRadius;
    marker.scale.z = 2.0f*_toolSafetyCollisionRadius;
    Eigen::Vector3f temp;
    temp = _xEE[r]+_toolCollisionOffset[r]+_toolSafetyCollisionRadius*_rToolCollision[r].normalized();
    marker.pose.position.x = temp(0);
    marker.pose.position.y = temp(1);
    marker.pose.position.z = temp(2);
    markerArray.markers.push_back(marker);
  }

  _pubCollisionSpheres.publish(markerArray);


}


void SurgicalTask::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int r)
{
  Eigen::Vector3f temp = _x[r];

  // Update end effecotr pose (position+orientation)
  _xEE[r] << msg->position.x, msg->position.y, msg->position.z;
  _q[r] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[r] = Utils<float>::quaternionToRotationMatrix(_q[r]);
  _x[r] = _xEE[r]+_wRb[r]*_toolOffsetFromEE[r];

  if((!_useSim && _trackingOK) ||_firstRobotBaseFrame[r])
  {
    _xEE[r] += _xRobotBaseOrigin[r];
    _x[r] += _xRobotBaseOrigin[r];
  }

  if(!_firstRobotPose[r])
  {
    if((!_useSim && _trackingOK)  || _firstRobotBaseFrame[r])
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
  if(r==LEFT)
  {
    _footPose[r](FOOT_PITCH) = msg->axes[4];
    _footPose[r](FOOT_YAW) = (-msg->axes[2]+1.0f)/2.0f-(-msg->axes[5]+1.0f)/2.0f;
    _footPose[r](FOOT_ROLL) = msg->axes[3];

  }
  else
  {
    _footPose[r](FOOT_PITCH) = msg->axes[3];
    _footPose[r](FOOT_YAW) = 0.0f;
    _footPose[r](FOOT_ROLL) = msg->axes[2];

  // //   _footPose[r](FOOT_YAW) = (-msg->axes[13]+1.0f)/2.0f-(-msg->axes[14]+1.0f)/2.0f;
  }
  // _footPose[r](FOOT_YAW) = (-msg->axes[5]+1.0f)/2.0f-(-msg->axes[2]+1.0f)/2.0f;

  // if(msg->buttons[4] && !msg->buttons[5])
  // {
  //   _msgGripperInput.ros_desAngle = 0;
  // }
  // else if(!msg->buttons[4] && msg->buttons[5])
  // {
  //   _msgGripperInput.ros_desAngle = 20;
  // }

  if(!_firstHumanInput[r])
  {
    _firstHumanInput[r]= true;
  }
}


void SurgicalTask::updateTaskJoystick(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(msg->buttons[X_BUTTON] == 1)
  {
    _stop = true;
  }

  if(msg->buttons[TRIANGLE_BUTTON] == 1 && !_clutching && _humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
  {
    _clutching = true;
    int r = ((_tool[LEFT] == CAMERA) ? RIGHT : LEFT); 
    _toolClutchingOffset.segment(0,3) = _desiredOffsetPPM[r]; 
    _toolClutchingOffset(SELF_ROTATION) = _desiredAnglePPM[r];
  }
  else if (msg->buttons[TRIANGLE_BUTTON] == 1 && _clutching && _humanInputMode ==  SINGLE_FOOT_SINGLE_ROBOT)
  {
    _clutching = false;
  }
}


void SurgicalTask::updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int r)
{
  for(int m = 0; m < 5; m++)
  {
    _footPose[r](m) = msg->platform_position[m];
    _footTwist[r](m) = msg->platform_speed[m];
    _footWrenchD[r](m) = msg->platform_effortD[m];
    _footWrenchRef[r](m) = msg->platform_effortRef[m];
    _footWrenchM[r](m) = msg->platform_effortM[m];
  }
  _footPose[r] -= _footOffset[r];
  _footState[r] = msg->platform_machineState;

  if(!_firstHumanInput[r])
  {
    _firstHumanInput[r] = true;
  }
}


void SurgicalTask::updateFootInput(const custom_msgs::FootInputMsg::ConstPtr& msg, int r)
{
  for(int m = 0; m < 5; m++)
  {
    _footInputPosition[r](m) = msg->ros_position[m];
    _footInputFilterAxisForce[r](m) = msg->ros_filterAxisForce[m];
    _footInputKp[r](m) = msg->ros_kp[m];
    _footInputKd[r](m) = msg->ros_kd[m];
  }

  if(!_firstFootInput[r])
  {
    _firstFootInput[r] = true;
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


void SurgicalTask::updateRobotExternalWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int r)
{

  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchExtBiasOK[r])
  {
    _wrenchExtBias[r] += raw; 
    _wrenchExtCount[r]++;
    if(_wrenchExtCount[r]==NB_SAMPLES)
    {
      _wrenchExtBias[r] /= NB_SAMPLES;
      _wrenchExtBiasOK[r] = true;
      std::cerr << "[SurgicalTask]: Bias Ext " << r << ": " <<_wrenchExtBias[r].transpose() << std::endl;
    }
  }

  if(_wrenchExtBiasOK[r])
  {
    float alpha = 0.0f;
    _Fext[r] = alpha*_Fext[r]+(1-alpha)*(raw-_wrenchExtBias[r]).segment(0,3);
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
    _currentJointVelocities[r](m) = msg->velocity[m];
    _currentJointTorques[r](m) = msg->effort[m];
  }

  if(_useFranka)
  {
    Eigen::Matrix4f H;
    H = Utils<float>::getForwardKinematics(_currentJoints[r], _robotID);
    _xEE[r] = _wRRobotBasis[r]*H.block(0,3,3,1);
    _wRb[r] = _wRRobotBasis[r]*H.block(0,0,3,3);
    _q[r] = Utils<float>::rotationMatrixToQuaternion(_wRb[r]);
    _x[r] = _xEE[r]+_wRb[r]*_toolOffsetFromEE[r];

    if(!_useSim ||_firstRobotBaseFrame[r])
    {
      _xEE[r] += _xRobotBaseOrigin[r];
      _x[r] += _xRobotBaseOrigin[r];
    }

    if(_trocarsRegistered[r])
    {
      if(!_firstRobotPose[r])
      {
        if(_useFranka)
        {
        _firstRobotPose[r] = true;
        _firstRobotTwist[r] = true;
        _wRb0[r] = _wRb[r];
        _xd0[r] = _x[r];
        _inputAlignedWithOrigin[r] = true;          
        }
      }
    }
  }

  if(!_firstJointsUpdate[r])
  {
    if(!_useSim  || _firstRobotBaseFrame[r])
    {
      _firstJointsUpdate[r] = true;
      // if(_useFranka)
      // {
      //   _firstRobotPose[r] = true;
      //   _firstRobotTwist[r] = true;

      // }
      _ikJoints[r] = _currentJoints[r];
    }
  }
}

void SurgicalTask::updateMarkersPosition(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{

  if(!_firstColorMarkersPosition)
  {
    _nbTasks = (int)(msg->layout.dim[0].size/3);
    _beliefsC.resize(_nbTasks);
    _beliefsC.setConstant(0.0f);
    _beliefsC(0) = 1.0f;
    _dbeliefsC.resize(_nbTasks);
    _dbeliefsC.setConstant(0.0f);

    _colorMarkersPosition.resize(_nbTasks, 3);
    _colorMarkersPosition.setConstant(0.0f);
    _colorMarkersFilteredPosition.resize(_nbTasks, 3);
    _colorMarkersFilteredPosition.setConstant(0.0f);
    _colorMarkersFilteredPosition2.resize(_nbTasks, 3);
    _colorMarkersFilteredPosition2.setConstant(0.0f);
    _colorMarkersStatus.resize(_nbTasks);

    _firstColorMarkersPosition = true;
  }


  if(_firstColorMarkersPosition)
  {
    for(int k = 0; k < _nbTasks; k++)
    {
      _colorMarkersStatus[k] = msg->data[3*k+2];
      if(_colorMarkersStatus[k])
      {
        _colorMarkersPosition.row(k) << msg->data[3*k], msg->data[3*k+1], 0.0f;
        _colorMarkersFilteredPosition2.row(k) = _markerFilterGain*_colorMarkersFilteredPosition2.row(k)+(1.0f-_markerFilterGain)*_colorMarkersPosition.row(k);
        _colorMarkersFilteredPosition.row(k) = _colorMarkersFilteredPosition2.row(k);
        // _colorMarkersFilteredPosition(k,0) = Utils<float>::deadZone(_colorMarkersFilteredPosition2(k,0),-0.3f,0.3f); 
        // _colorMarkersFilteredPosition(k,0) = Utils<float>::bound(_colorMarkersFilteredPosition(k,0)/0.7,-1.0f,1.0f); 
        // _colorMarkersFilteredPosition(k,1) = Utils<float>::deadZone(_colorMarkersFilteredPosition2(k,1),-0.2f,0.2f); 
        // _colorMarkersFilteredPosition(k,1) = Utils<float>::bound(_colorMarkersFilteredPosition(k,1)/0.8,-1.0f,1.0f); 
      }
      else
      {
        _colorMarkersPosition.row(k).setConstant(0.0f);
        _colorMarkersFilteredPosition.row(k).setConstant(0.0f);
        _colorMarkersFilteredPosition2.row(k).setConstant(0.0f);
      }


    }

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


void SurgicalTask::updateLegState(const sensor_msgs::JointState::ConstPtr& msg, int k)
{
  for(int m = 0; m < 7; m++)
  {
    _legJointPositions[k](m) = msg->position[m];
    _legJointVelocities[k](m) = msg->velocity[m];
    _legJointTorques[k](m) = msg->effort[m];
  }
}


void SurgicalTask::updateFootBaseWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  _legFootBaseWrench[k](0) = msg->wrench.force.x;
  _legFootBaseWrench[k](1) = msg->wrench.force.y;
  _legFootBaseWrench[k](2) = msg->wrench.force.z;
  _legFootBaseWrench[k](3) = msg->wrench.torque.x;
  _legFootBaseWrench[k](4) = msg->wrench.torque.y;
  _legFootBaseWrench[k](5) = msg->wrench.torque.z;
}


void SurgicalTask::updateFootHaptics(const custom_msgs::FootInputMsg::ConstPtr& msg, int k)
{
  for(int m = 0; m < 5; m++)
  {
    _footHapticEfforts[k](m) = msg->ros_effort[m];
  }
}


void SurgicalTask::updateFootInertiaCoriolisCompensation(const custom_msgs::FootInputMsg::ConstPtr& msg, int k)
{
  for(int m = 0; m < 5; m++)
  {
    _footInertiaCoriolisCompensationTorques[k](m) = msg->ros_effort[m];
  }
}


void SurgicalTask::updateLegCompensation(const custom_msgs::FootInputMsg::ConstPtr& msg, int k)
{
  for(int m = 0; m < 5; m++)
  {
    _legCompensationTorques[k](m) = msg->ros_effort[m];
  }
}


void SurgicalTask::updateFootForceSensorModified(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  _footWrenchModified[k](0) = msg->wrench.force.x;
  _footWrenchModified[k](1) = msg->wrench.force.y;
  _footWrenchModified[k](2) = msg->wrench.force.z;
  _footWrenchModified[k](3) = msg->wrench.torque.x;
  _footWrenchModified[k](4) = msg->wrench.torque.y;
  _footWrenchModified[k](5) = msg->wrench.torque.z;
}


void SurgicalTask::updateGripperAssistance(const std_msgs::Bool::ConstPtr& msg)
{
  _graspAssistanceOn = msg->data;
}


void SurgicalTask::updateGripperFeedbackToPlatform(const custom_msgs::FootInputMsg::ConstPtr& msg)
{
  _gripperFeedback = msg->ros_effort[FOOT_ROLL];
}


void SurgicalTask::updateTaskManagerState(const surgical_task::TaskManagerStateMsg::ConstPtr& msg)
{
  if(!_firstTaskManagerState)
  {
    _firstTaskManagerState = true;
  }

  _taskStarted = msg->start;
  _taskFinished = msg->finished;
  _stopTime = msg->stopTime;
  _imageId = msg->imageId;
}


void SurgicalTask::updateFootHapticsData(const custom_msgs::FootHapticDataMsg::ConstPtr& msg, int k)
{
  for(int m = 0; m < 7; m++)
  {
    _fh_haptEffLegIn[k](m) = msg->fh_haptEffLegIn[m];
    _fh_jointLimCoeffs[k](m) = msg->fh_jointLimCoeffs[m];
    _fh_bckgndEffLeg[k](m) = msg->fh_bckgndEffLeg[m];
  }

  for(int m = 0; m < 5; m++)
  {
    _fh_haptEffLPF[k](m) = msg->fh_haptEffLPF[m];
    _fh_haptEffLPF_Proj[k](m) = msg->fh_haptEffLPF_Proj[m];
    _fh_haptEffHPF[k](m) = msg->fh_haptEffHPF[m];
    _fh_effFootOut[k](m) = msg->fh_effFootOut[m];
    _fh_vibFB[k](m) = msg->fh_vibFB[m];
    _fh_maxPossGains[k](m) = msg->fh_maxPossGains[m];
  }

  _fh_effGainRaw[k] = msg->fh_effGainRaw;
}
