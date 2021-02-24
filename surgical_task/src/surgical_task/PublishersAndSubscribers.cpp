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
    }
   
    if(_humanInputDevice[RIGHT] == JOYSTICK)
    {
      _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg_v3>("/FI_Output/Right",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
      _subFootSharedGrasping[RIGHT] = _nh.subscribe<custom_msgs_gripper::SharedGraspingMsg>("/right/sharedGrasping",1, boost::bind(&SurgicalTask::updateFootSharedGrasping,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    _subGripper = _nh.subscribe("/right/gripperOutput", 1, &SurgicalTask::updateGripperOutput, this, ros::TransportHints().reliable().tcpNoDelay());

    _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/right_lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/right_lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("/right_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("SurgicalTask/filteredWrenchRight", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg_v5>("/right/surgical_task/foot_input", 1);
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

  }

  _pubSurgicalTaskState = _nh.advertise<surgical_task::SurgicalTaskStateMsg>("surgical_task/state", 1);


  _subOptitrackPose[RIGHT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_ROBOT_BASIS] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_robot/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_ROBOT_BASIS),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[LEFT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/left_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,LEFT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPose[RIGHT_HUMAN_TOOL] = _nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/right_tool/pose", 1, boost::bind(&SurgicalTask::updateOptitrackPose,this,_1,RIGHT_HUMAN_TOOL),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

  _subMarkersPosition = _nh.subscribe("/surgical_task/markers_position_transformed", 1, &SurgicalTask::updateMarkersPosition, this, ros::TransportHints().reliable().tcpNoDelay());

}


void SurgicalTask::checkAllSubscribers()
{
  bool robotStatusOK[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatusOK[k] = !_useRobot[k] || (_firstRobotPose[k] &&  _firstRobotTwist[k]
                      // && _firstDampingMatrix[k] 
                      && _firstJointsUpdate[k] 
                      && _firstHumanInput[k]);
                      // && (_useSim || _wrenchBiasOK[k]);

    if(!robotStatusOK[k])
    {
      std::cerr << k << ": Status: " << "use: " << _useRobot[k] 
                << " pose: " << _firstRobotPose[k] << " twist: " << _firstRobotTwist[k]
                << " damp: " << _firstDampingMatrix[k] << " joints: " << _firstJointsUpdate[k]
                << " human: " << _firstHumanInput[k]
                << " sim/wrench: " << (_useSim || _wrenchBiasOK[k]) << std::endl;
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
  }

  _allSubscribersOK = robotStatusOK[LEFT] && robotStatusOK[RIGHT] && _trackingOK;// && (_useSim ||_firstGripper);
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

  _msgSurgicalTaskState.humanInputMode = _humanInputMode;
  _msgSurgicalTaskState.controlPhase[LEFT] = _controlPhase[LEFT];
  _msgSurgicalTaskState.controlPhase[RIGHT] = _controlPhase[RIGHT];
  _msgSurgicalTaskState.currentRobot = _currentRobot;
  _msgSurgicalTaskState.useTaskAdaptation = _useTaskAdaptation;
  _msgSurgicalTaskState.clutching = _clutching;
  _msgSurgicalTaskState.wait = _wait;
  _msgSurgicalTaskState.eeCollision = false;
  _msgSurgicalTaskState.eeCollision = _qpResult[LEFT].eeCollisionConstraintActive;
  _msgSurgicalTaskState.toolCollision = _qpResult[LEFT].toolCollisionConstraintActive;
  _msgSurgicalTaskState.workspaceCollision = _qpResult[LEFT].workspaceCollisionConstraintActive;

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

  std::cerr << r <<  " " << (int)_firstHumanInput[r] << std::endl;
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

void SurgicalTask::updateFootSharedGrasping(const custom_msgs_gripper::SharedGraspingMsg::ConstPtr& msg, int r)
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
    _xEE[r] = _wRRobotBasis[r]*H.block(0,3,3,1);
    _wRb[r] = _wRRobotBasis[r]*H.block(0,0,3,3);
    _q[r] = Utils<float>::rotationMatrixToQuaternion(_wRb[r]);
    _x[r] = _xEE[r]+_toolOffsetFromEE[r]*_wRb[r].col(2);

/*    std::cerr << _xEE[r].transpose() << std::endl;
    std::cerr << _toolOffsetFromEE[r] << std::endl;
    std::cerr << _wRb[r].col(2).transpose() << std::endl;
    std::cerr << _xRobotBaseOrigin[r].transpose() << std::endl;
    std::cerr << _wRRobotBasis[r] << std::endl;*/
    if(!_useSim ||_firstRobotBaseFrame[r])
    {
      _xEE[r] += _xRobotBaseOrigin[r];
      _x[r] += _xRobotBaseOrigin[r];
    }
  }

  if(!_firstJointsUpdate[r])
  {
    if(!_useSim  || _firstRobotBaseFrame[r])
    {
      _firstJointsUpdate[r] = true;
      if(_useFranka)
      {
        _firstRobotPose[r] = true;
        _firstRobotTwist[r] = true;
        _wRb0[r] = _wRb[r];
        _xd0[r] = _x[r];
        _inputAlignedWithOrigin[r] = true;
      }
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
      }
      else
      {
        _colorMarkersPosition.row(k).setConstant(0.0f);
      }
    }

  }
/*  _humanToolStatus[LEFT] = (int) msg->data[2];
  if(_humanToolStatus[LEFT])
  {
    _humanToolPosition[LEFT] << msg->data[0],  msg->data[1], 0.0f;
  }
  else
  {
    _humanToolPosition[LEFT].setConstant(0.0f);
  }
  _humanToolStatus[RIGHT] = (int) msg->data[5];
  if(_humanToolStatus[RIGHT])
  {
    _humanToolPosition[RIGHT] << msg->data[3],  msg->data[4], 0.0f;
  }
  else
  {
    _humanToolPosition[RIGHT].setConstant(0.0f);
  }
*/
  // _D[r] << msg->data[0],msg->data[1],msg->data[2],
  //          msg->data[3],msg->data[4],msg->data[5],
  //          msg->data[6],msg->data[7],msg->data[8];
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