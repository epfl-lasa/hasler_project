#include "SurgicalTask.h"


bool SurgicalTask::readConfigurationParameters()
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


  if (!_nh.getParam("SurgicalTask/humanInputMode", _humanInputMode))
  {
    ROS_ERROR("Couldn't retrieve the human input mode");
    return false;
  }
  else
  {
    ROS_INFO("Human input mode: %d\n", (int) _humanInputMode);
  }


  if(!_nh.getParam("SurgicalTask/dominantInput", _dominantInputID))
  {
    ROS_ERROR("Couldn't retrieve the dominant human input");
    return false;
  }
  else
  {
    if(_dominantInputID == RIGHT)
    {
      _nonDominantInputID = LEFT;
    }
    else if(_dominantInputID == LEFT)
    {
      _nonDominantInputID = RIGHT;
    }
    else
    {
      ROS_ERROR("Dominant foot selected does not exist");
      return false;
    }
    ROS_INFO("Dominant human input: %d\n", _dominantInputID);
    ROS_INFO("Non dominant human input: %d\n", _nonDominantInputID);
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


  _jointImpedanceStiffnessGain.resize(7);
  if (!_nh.getParam("SurgicalTask/jointImpedanceStiffnessGain", _jointImpedanceStiffnessGain))
  {
    ROS_ERROR("Couldn't retrieve the joint impedance stiffness gain");
    return false;
  }
  else
  {
    ROS_INFO("Joint impedance stiffness gain: %f %f %f %f %f %f %f\n", _jointImpedanceStiffnessGain[0],
                                                                       _jointImpedanceStiffnessGain[1],
                                                                       _jointImpedanceStiffnessGain[2],
                                                                       _jointImpedanceStiffnessGain[3],
                                                                       _jointImpedanceStiffnessGain[4],
                                                                       _jointImpedanceStiffnessGain[5],
                                                                       _jointImpedanceStiffnessGain[6]);
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


  if (!_nh.getParam("SurgicalTask/insertionDistancePVM", _insertionDistancePVM))
  {
    ROS_ERROR("Couldn't retrieve the insertion distance in position velocity mapping");
    return false;
  }
  else
  {
    ROS_INFO("Insertion distance in position velocity mapping: %f %f\n", _insertionDistancePVM[LEFT], _insertionDistancePVM[RIGHT]);
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


  if (!_nh.getParam("SurgicalTask/insertionOffsetPPM", temp))
  {
    ROS_ERROR("Couldn't retrieve the insertion offset for position position mapping");
    return false;
  }
  else
  {
    _insertionOffsetPPM[LEFT] << temp[0], temp[1], temp[2];
    _insertionOffsetPPM[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Insertion offset for position position mapping: LEFT: %f %f %f RIGHT: %f %f %f", _insertionOffsetPPM[LEFT](0), _insertionOffsetPPM[LEFT](1), _insertionOffsetPPM[LEFT](2),
                                                                                               _insertionOffsetPPM[RIGHT](0), _insertionOffsetPPM[RIGHT](1), _insertionOffsetPPM[RIGHT](2));
  }


  if (!_nh.getParam("SurgicalTask/operationOffsetRangePPM", temp))
  {
    ROS_ERROR("Couldn't retrieve the operation phase offset range for position position mapping");
    return false;
  }
  else
  {
    _operationOffsetRangePPM[LEFT] << temp[0], temp[1], temp[2];
    _operationOffsetRangePPM[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Operation phase offset range for position position mapping: LEFT: %f %f %f RIGHT: %f %f %f", _operationOffsetRangePPM[LEFT](0), _operationOffsetRangePPM[LEFT](1), _operationOffsetRangePPM[LEFT](2),
                                                                                           _operationOffsetRangePPM[RIGHT](0), _operationOffsetRangePPM[RIGHT](1), _operationOffsetRangePPM[RIGHT](2));
  }


  if (!_nh.getParam("SurgicalTask/operationMinOffsetPVM", temp))
  {
    ROS_ERROR("Couldn't retrieve the operation phase min offset for position position mapping");
    return false;
  }
  else
  {
    _operationMinOffsetPVM[LEFT] << temp[0], temp[1], temp[2];
    _operationMinOffsetPVM[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Operation phase min offset for position position mapping: LEFT: %f %f %f RIGHT: %f %f %f", _operationMinOffsetPVM[LEFT](0), _operationMinOffsetPVM[LEFT](1), _operationMinOffsetPVM[LEFT](2),
                                                                                                         _operationMinOffsetPVM[RIGHT](0), _operationMinOffsetPVM[RIGHT](1), _operationMinOffsetPVM[RIGHT](2));
  }


  if (!_nh.getParam("SurgicalTask/operationMaxOffsetPVM", temp))
  {
    ROS_ERROR("Couldn't retrieve the operation phase max offset for position position mapping");
    return false;
  }
  else
  {
    _operationMaxOffsetPVM[LEFT] << temp[0], temp[1], temp[2];
    _operationMaxOffsetPVM[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Operation phase max offset for position position mapping: LEFT: %f %f %f RIGHT: %f %f %f", _operationMaxOffsetPVM[LEFT](0), _operationMaxOffsetPVM[LEFT](1), _operationMaxOffsetPVM[LEFT](2),
                                                                                                         _operationMaxOffsetPVM[RIGHT](0), _operationMaxOffsetPVM[RIGHT](1), _operationMaxOffsetPVM[RIGHT](2));
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


  if (!_nh.getParam("SurgicalTask/toolsTracking", _toolsTracking))
  {
    ROS_ERROR("Couldn't retrieve the tools tracking mode");
    return false;
  }
  else
  {
    ROS_INFO("Tools tracking: %d\n", _toolsTracking);
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


  if (!_nh.getParam("SurgicalTask/enableEECollisionAvoidance", _enableEECollisionAvoidance))
  {
    ROS_ERROR("Couldn't retrieve the EE enable collision avoidance boolean");
    return false;
  }
  else
  {
    ROS_INFO("Enable EE collision avoidance: %d\n", (int) _enableEECollisionAvoidance);
  }

  if (!_nh.getParam("SurgicalTask/enableToolCollisionAvoidance", _enableToolCollisionAvoidance))
  {
    ROS_ERROR("Couldn't retrieve the enable Tool collision avoidance boolean");
    return false;
  }
  else
  {
    ROS_INFO("Enable Tool collision avoidance: %d\n", (int) _enableToolCollisionAvoidance);
  }

  if (!_nh.getParam("SurgicalTask/eeSafetyCollisionDistance", _eeSafetyCollisionDistance))
  {
    ROS_ERROR("Couldn't retrieve the EE safety collision distance");
    return false;
  }
  else
  {
    ROS_INFO("EE safety collision distance: %f\n", _eeSafetyCollisionDistance);
  }

  if (!_nh.getParam("SurgicalTask/eeSafetyCollisionRadius", _eeSafetyCollisionRadius))
  {
    ROS_ERROR("Couldn't retrieve the EE safety collision radius");
    return false;
  }
  else
  {
    ROS_INFO("EE safety collision radius: %f\n", _eeSafetyCollisionRadius);
  }

  if (!_nh.getParam("SurgicalTask/toolSafetyCollisionDistance", _toolSafetyCollisionDistance))
  {
    ROS_ERROR("Couldn't retrieve the Tool safety collision distance");
    return false;
  }
  else
  {
    ROS_INFO("Tool safety collision distance: %f\n", _toolSafetyCollisionDistance);
  }

  return true;
}


void SurgicalTask::initializeTaskParameters()
{
  me = this;
  
  _sphericalTrocarId[LEFT] = 14;
  _sphericalTrocarId[RIGHT] = 23;

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

    _controlPhase[r] = INSERTION;
    _ikJoints[r].resize(7);
    _currentJoints[r].resize(7);

    _trocarPosition[r].setConstant(0.0f);
    _wrenchCount[r] = 0;
    _filterGainFootAxis[r].setConstant(1.0f);
    _footOffset[r].setConstant(0.0f);
    _desiredOffsetPPM[r].setConstant(0.0f);
    _desiredAnglePPM[r] = 0.0f;
    _desiredGripperPosition[r] = 0.0f;
    _dRCMTool[r] = 0.0f;

    _stiffness[r].setConstant(0.0f);
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
    _insertionFinished[r] = false;
    _wRRobotBasis[r].setIdentity();
    _humanToolStatus[r] = 0;
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


  _filteredForceGain = 0.9f;

  _currentRobot = LEFT;
  _clutching = false;
  _humanClutchingOffset.setConstant(0.0f);
  _toolClutchingOffset.setConstant(0.0f);
  _attractorOffset.setConstant(0.0f);

  if(!_useSim)
  {
    _xRobotBaseOrigin[LEFT].setConstant(0.0f);
     _xRobotBaseOrigin[LEFT] << 0.0f, 0.0f, 0.0f; // WARNING !!!!!!!!!!!
    _xRobotBaseOrigin[RIGHT].setConstant(0.0f);
    _xRobotBaseOrigin[RIGHT] << 0.0f, 0.79f, 0.0f;
    _wRRobotBasis[LEFT].setIdentity();
    _wRRobotBasis[RIGHT].setIdentity();
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
  _pillarsId <<  1, 3, 10;
  _pillarsPosition.resize(_pillarsId.size(),3);
  _pillarsPosition.setConstant(0.0f);

  if(_useSim)
  {
    _nbTasks = 3;  

    for(int k = 0; k < 3; k++)
    {
      _firstPillarsFrame[k] = false;
    }
  }
  else
  {
    _nbTasks = 2;
  }

  if(_useRobot[RIGHT])
  {
    _nbTasks++;
  }

  _beliefsC.resize(_nbTasks);
  _beliefsC.setConstant(0.0f);
  _beliefsC(0) = 1.0f;
  _dbeliefsC.resize(_nbTasks);
  _dbeliefsC.setConstant(0.0f);

  // Eigen::Vector3f p0;
  // p0 <<-0.413005,  0.443508, 0.0539682;

  // if(!_useSim)
  // {
  //   _pillarsPosition.row(0) = p0;
  //   _pillarsPosition.row(1) << p0(0), p0(1)-0.064, p0(2);
  //   _pillarsPosition.row(2) << p0(0)+0.064, p0(1)-0.064, p0(2);    
  // }

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


  if(_useRobot[LEFT] && _useRobot[RIGHT])
  {
    _qpSolverRCMCollision[LEFT] = new QpSolverRCMCollision(_enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                           _enableToolCollisionAvoidance, _toolSafetyCollisionDistance);  
    _qpSolverRCMCollision[RIGHT] = new QpSolverRCMCollision(_enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                            _enableToolCollisionAvoidance, _toolSafetyCollisionDistance);    
  }
  else
  {
    _qpSolverRCMCollision[LEFT] = new QpSolverRCMCollision();
    _qpSolverRCMCollision[RIGHT] = new QpSolverRCMCollision();
  }

  _qpSolverRCM[LEFT].setRobot(_robotID);
  _qpSolverRCM[RIGHT].setRobot(_robotID);
  _qpSolverRCMCollision[LEFT]->setRobot(_robotID);
  _qpSolverRCMCollision[RIGHT]->setRobot(_robotID);
  _qpSolverRCMCollision2[LEFT].setRobot(_robotID);
  _qpSolverRCMCollision2[RIGHT].setRobot(_robotID);
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
    cfmakeraw(&newSettings);
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