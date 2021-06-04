#include "SurgicalTask.h"


bool SurgicalTask::readConfigurationParameters()
{

  if (!_nh.getParam("SurgicalTask/debug", _debug))
  {
    ROS_ERROR("Couldn't retrieve the use Franka boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use Franka: %d\n", (int) _debug);
  }



  if (!_nh.getParam("SurgicalTask/logData", _logData))
  {
    ROS_ERROR("Couldn't retrieve the log data boolean");
    return false;
  }
  else
  {
    ROS_INFO("Log data: %d\n", (int) _debug);
  }


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

  _tool.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/tool", _tool))
  {
    ROS_ERROR("Couldn't retrieve the tool ID boolean");
    return false;
  }
  else
  {
    ROS_INFO("Tool ID: %d %d\n", (int) _tool[LEFT], (int)_tool[RIGHT]);
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


  _humanInputID.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/humanInputID", _humanInputID))
  {
    ROS_ERROR("Couldn't retrieve the human input ID boolean");
    return false;
  }
  else
  {
    ROS_INFO("Human Input ID: %d %d\n", (int) _humanInputID[LEFT], (int)_humanInputID[RIGHT]);
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


  if (!_nh.getParam("SurgicalTask/switchingAxis", _switchingAxis))
  {
    ROS_ERROR("Couldn't retrieve the switching axis");
    return false;
  }
  else
  {
    ROS_INFO("Switching axis: %d\n", (int) _switchingAxis);
  }


  _switchingThreshold.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/switchingThreshold", _switchingThreshold))
  {
    ROS_ERROR("Couldn't retrieve the switching thresholds ");
    return false;
  }
  else
  {
    ROS_INFO("Switching thresholds: %f %f\n", _switchingThreshold[LEFT], _switchingThreshold[RIGHT]);
  }


  if (!_nh.getParam("SurgicalTask/clutchingAxis", _clutchingAxis))
  {
    ROS_ERROR("Couldn't retrieve the clutching axis");
    return false;
  }
  else
  {
    ROS_INFO("Clutching axis: %d\n", (int) _clutchingAxis);
  }


  if (!_nh.getParam("SurgicalTask/clutchingActivationThreshold", _clutchingActivationThreshold))
  {
    ROS_ERROR("Couldn't retrieve the clutching activation threshold");
    return false;
  }
  else
  {
    ROS_INFO("Clutching activation threshold: %f\n", _clutchingActivationThreshold);
  }


  if (!_nh.getParam("SurgicalTask/clutchingDeactivationThreshold", _clutchingDeactivationThreshold))
  {
    ROS_ERROR("Couldn't retrieve the clutching deactivation threshold");
    return false;
  }
  else
  {
    ROS_INFO("Clutching deactivation threshold: %f\n", _clutchingDeactivationThreshold);
  }


  if (!_nh.getParam("SurgicalTask/gripperControlAxis", _gripperControlAxis))
  {
    ROS_ERROR("Couldn't retrieve the gripper control axis");
    return false;
  }
  else
  {
    ROS_INFO("Gripper control axis: %d\n", (int) _gripperControlAxis);
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


  std::vector<float> temp;
  temp.resize(6);
  
  if (!_nh.getParam("SurgicalTask/toolOffsetFromEE", temp))
  {
    ROS_ERROR("Couldn't retrieve the tool offsets from end effector");
    return false;
  }
  else
  {
    _toolOffsetFromEE[LEFT] << temp[0], temp[1], temp[2];
    _toolOffsetFromEE[RIGHT] << temp[3], temp[4], temp[5];
    ROS_INFO("Tool offset from EE: LEFT: %f %f %f RIGHT: %f %f %f", _toolOffsetFromEE[LEFT](0), _toolOffsetFromEE[LEFT](1), _toolOffsetFromEE[LEFT](2),
                                                                    _toolOffsetFromEE[RIGHT](0), _toolOffsetFromEE[RIGHT](1), _toolOffsetFromEE[RIGHT](2));
  }


  _insertionDistancePVM.resize(NB_ROBOTS);
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


  _footInterfaceMinDeadZone[LEFT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/leftFootInterfaceMinDeadZone", _footInterfaceMinDeadZone[LEFT]))
  {
    ROS_ERROR("Couldn't retrieve the left foot interface min deadzones");
    return false;
  }
  else
  {
    ROS_INFO("Left foot interface min deadzones: %f %f %f %f %f\n", _footInterfaceMinDeadZone[LEFT][FOOT_X], _footInterfaceMinDeadZone[LEFT][FOOT_Y], _footInterfaceMinDeadZone[LEFT][FOOT_PITCH], _footInterfaceMinDeadZone[LEFT][FOOT_ROLL], _footInterfaceMinDeadZone[LEFT][FOOT_YAW]);
  }

  _footInterfaceMaxDeadZone[LEFT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/leftFootInterfaceMaxDeadZone", _footInterfaceMaxDeadZone[LEFT]))
  {
    ROS_ERROR("Couldn't retrieve the left foot interface max deadzones");
    return false;
  }
  else
  {
    ROS_INFO("Left foot interface max deadzones: %f %f %f %f %f\n", _footInterfaceMaxDeadZone[LEFT][FOOT_X], _footInterfaceMaxDeadZone[LEFT][FOOT_Y], _footInterfaceMaxDeadZone[LEFT][FOOT_PITCH], _footInterfaceMaxDeadZone[LEFT][FOOT_ROLL], _footInterfaceMaxDeadZone[LEFT][FOOT_YAW]);
  }

  _footInterfaceMinDeadZone[RIGHT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/rightFootInterfaceMinDeadZone", _footInterfaceMinDeadZone[RIGHT]))
  {
    ROS_ERROR("Couldn't retrieve the right foot interface min deadzones");
    return false;
  }
  else
  {
    ROS_INFO("Right foot interface min deadzones: %f %f %f %f %f\n", _footInterfaceMinDeadZone[RIGHT][FOOT_X], _footInterfaceMinDeadZone[RIGHT][FOOT_Y], _footInterfaceMinDeadZone[RIGHT][FOOT_PITCH], _footInterfaceMinDeadZone[RIGHT][FOOT_ROLL], _footInterfaceMinDeadZone[RIGHT][FOOT_YAW]);
  }

  _footInterfaceMaxDeadZone[RIGHT].resize(NB_DOF_FOOT_INTERFACE);
  if (!_nh.getParam("SurgicalTask/rightFootInterfaceMaxDeadZone", _footInterfaceMaxDeadZone[RIGHT]))
  {
    ROS_ERROR("Couldn't retrieve the right foot interface max deadzones");
    return false;
  }
  else
  {
    ROS_INFO("Right foot interface max deadzones: %f %f %f %f %f\n", _footInterfaceMaxDeadZone[RIGHT][FOOT_X], _footInterfaceMaxDeadZone[RIGHT][FOOT_Y], _footInterfaceMaxDeadZone[RIGHT][FOOT_PITCH], _footInterfaceMaxDeadZone[RIGHT][FOOT_ROLL], _footInterfaceMaxDeadZone[RIGHT][FOOT_YAW]);
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

  temp.resize(6);

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


  _operationMinInsertion.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/operationMinInsertion", _operationMinInsertion))
  {
    ROS_ERROR("Couldn't retrieve the operation phase min insertion");
    return false;
  }
  else
  {
    ROS_INFO("Operation phase min insertion: LEFT: %f RIGHT: %f", _operationMinInsertion[LEFT], _operationMinInsertion[RIGHT]);
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


  if (!_nh.getParam("SurgicalTask/markerFilterGain", _markerFilterGain))
  {
    ROS_ERROR("Couldn't retrieve the marker filter gain");
    return false;
  }
  else
  {
    _markerFilterGain = Utils<float>::bound(_markerFilterGain,0.0f,1.0f);
    ROS_INFO("Marker filter gain: %f\n", _markerFilterGain);
  }


  if (!_nh.getParam("SurgicalTask/taskAdaptationActivationThreshold", _taskAdaptationActivationThreshold))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation activation threshold");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation activation threshold: %f\n", _taskAdaptationActivationThreshold);
  }


  if (!_nh.getParam("SurgicalTask/taskAdaptationDeactivationThreshold", _taskAdaptationDeactivationThreshold))
  {
    ROS_ERROR("Couldn't retrieve the task adaptation deactivation threshold");
    return false;
  }
  else
  {
    ROS_INFO("Task adaptation deactivation threshold: %f\n", _taskAdaptationDeactivationThreshold);
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


  if (!_nh.getParam("SurgicalTask/eeLinearVelocityLimit", _eeLinearVelocityLimit))
  {
    ROS_ERROR("Couldn't retrieve the EE linear velocity Limit");
    return false;
  }
  else
  {
    ROS_INFO("EE linear velocity Limit: %f\n", _eeLinearVelocityLimit);
  }


  if (!_nh.getParam("SurgicalTask/eeAngularVelocityLimit", _eeAngularVelocityLimit))
  {
    ROS_ERROR("Couldn't retrieve the EE angular velocity Limit");
    return false;
  }
  else
  {
    ROS_INFO("EE angular velocity Limit: %f\n", _eeAngularVelocityLimit);
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


  if (!_nh.getParam("SurgicalTask/toolSafetyCollisionRadius", _toolSafetyCollisionRadius))
  {
    ROS_ERROR("Couldn't retrieve the tool safety collision radius");
    return false;
  }
  else
  {
    ROS_INFO("Tool safety collision radius: %f\n", _toolSafetyCollisionRadius);
  }


  if (!_nh.getParam("SurgicalTask/enableWorkspaceCollisionAvoidance", _enableWorkspaceCollisionAvoidance))
  {
    ROS_ERROR("Couldn't retrieve the enable Workspace collision avoidance boolean");
    return false;
  }
  else
  {
    ROS_INFO("Enable Workspace collision avoidance: %d\n", (int) _enableWorkspaceCollisionAvoidance);
  }


  temp.resize(25);
  if (!_nh.getParam("SurgicalTask/footPPMapping", temp))
  {
    ROS_ERROR("Couldn't retrieve the foot PP mapping matrix");
    return false;
  }
  else
  {
    _footPPMapping = Eigen::Map<Eigen::Matrix<float,5,5,Eigen::RowMajor>>(temp.data(),5,5);
    std::cout << "Foot PP mapping matrix: " << std::endl;
    std::cout << _footPPMapping << std::endl;
  }

  if (!_nh.getParam("SurgicalTask/footPVMapping", temp))
  {
    ROS_ERROR("Couldn't retrieve the foot PV mapping matrix");
    return false;
  }
  else
  {
    _footPVMapping = Eigen::Map<Eigen::Matrix<float,5,5,Eigen::RowMajor>>(temp.data(),5,5);
    std::cout << "Foot PV mapping matrix: " << std::endl;
    std::cout << _footPVMapping << std::endl;
  }

  temp.resize(9);
  if (!_nh.getParam("SurgicalTask/eeCameraMapping", temp))
  {
    ROS_ERROR("Couldn't retrieve the EE Camera mapping matrix");
    return false;
  }
  else
  {
    _eeCameraMapping = Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor>>(temp.data(),3,3);
    std::cout << "EE-camera mapping matrix: " << std::endl;
    std::cout << _eeCameraMapping << std::endl;
  }


  if (!_nh.getParam("SurgicalTask/linearForceFeedbackMagnitude", _linearForceFeedbackMagnitude))
  {
    ROS_ERROR("Couldn't retrieve the linear force feedback magnitude");
    return false;
  }
  else
  {
    ROS_INFO("Linear force feedback magnitude: %f\n", _linearForceFeedbackMagnitude);
  }


  if (!_nh.getParam("SurgicalTask/selfRotationTorqueFeedbackMagnitude", _selfRotationTorqueFeedbackMagnitude))
  {
    ROS_ERROR("Couldn't retrieve the self rotation torque feedback magnitude");
    return false;
  }
  else
  {
    ROS_INFO("Self rotation torque feedback magnitude: %f\n", _selfRotationTorqueFeedbackMagnitude);
  }

  _enablePhysicalHumanInteraction.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/enablePhysicalHumanInteraction", _enablePhysicalHumanInteraction))
  {
    ROS_ERROR("Couldn't retrieve the enable physical human interaction boolean");
    return false;
  }
  else
  {
    ROS_INFO("Enable physical human interaction: %d %d\n", (int) _enablePhysicalHumanInteraction[LEFT], (int) _enablePhysicalHumanInteraction[RIGHT]);
  }

  _useFTSensor.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/useFTSensor", _useFTSensor))
  {
    ROS_ERROR("Couldn't retrieve the use force torque sensor boolean");
    return false;
  }
  else
  {
    ROS_INFO("Use force torque sensor boolean: %d %d\n", (int) _useFTSensor[LEFT], (int) _useFTSensor[RIGHT]);
  }

  _externalForcesDeadZones.resize(NB_ROBOTS);
  if (!_nh.getParam("SurgicalTask/externalForcesDeadZones", _externalForcesDeadZones))
  {
    ROS_ERROR("Couldn't retrieve the external forces dead zones");
    return false;
  }
  else
  {
    ROS_INFO("External forces dead zones: %f %f\n", _externalForcesDeadZones[LEFT], _externalForcesDeadZones[RIGHT]);
  }
  return true;
}


void SurgicalTask::initializeTaskParameters()
{
  me = this;
  
  _sphericalTrocarId[LEFT] = 14;
  _sphericalTrocarId[RIGHT] = 23;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolMass[LEFT] = 0.275f;
  _toolComPositionFromSensor[LEFT] << 0.0f,0.0f,0.07f;
  // _toolMass[RIGHT] = 1.7f;
  _toolMass[RIGHT] = 1.2f;
  // _toolComPositionFromSensor[RIGHT] << -0.00281195f,0.00866844f,0.09194155f;
  // _toolComPositionFromSensor[RIGHT] << 0.0f,0.0f,0.09194155f;
  _toolComPositionFromSensor[RIGHT] << 0.0f,0.0f,0.105f;


  for(int r = 0; r < NB_ROBOTS; r++)
  {
    _x[r].setConstant(0.0f);
    _q[r].setConstant(0.0f);
    
    _xd[r].setConstant(0.0f);
    _vd[r].setConstant(0.0f);
    _vdTool[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _qd[r].setConstant(0.0f);
    
    _selfRotationCommand[r] = 0.0f;

    _footPose[r].setConstant(0.0f);
    _footPoseFiltered[r].setConstant(0.0f);
    _footWrenchM[r].setConstant(0.0f);
    _footWrenchD[r].setConstant(0.0f);
    _footWrenchRef[r].setConstant(0.0f);
    _footTwist[r].setConstant(0.0f);
    _trocarInput[r].setConstant(0.0f);
    _desiredFootWrench[r].setConstant(0.0f);
    _toolToFootTorques[r].setConstant(0.0f);
    _xRobotBaseOrigin[r].setConstant(0.0f);
    _qRobotBaseOrigin[r] << 1.0f, 0.0f, 0.0f, 0.0f;
    _D[r].setConstant(0.0f);

    _controlPhase[r] = AUTOMATIC_INSERTION;
    _ikJoints[r].resize(7);
    _currentJoints[r].resize(7);
    _currentJointVelocities[r].resize(7);
    _currentJointTorques[r].resize(7);

    _trocarPosition[r].setConstant(0.0f);
    _wrenchCount[r] = 0;
    _wrenchExtCount[r] = 0;
    _filterGainFootAxis[r].setConstant(1.0f);
    _footOffset[r].setConstant(0.0f);
    _desiredOffsetPPM[r].setConstant(0.0f);
    _desiredAnglePPM[r] = 0.0f;
    _desiredGripperPosition[r] = _gripperRange;
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
    _firstFootInput[r] = false;
    _wrenchBiasOK[r] = false;
    _wrenchExtBiasOK[r] = false;

    _wRRobotBasis[r].setIdentity();
    _humanToolStatus[r] = 0;
    _taud[r] = 0.0f;
    _Fext[r].setConstant(0.0f);
    _vHRef[r].setConstant(0.0f);
    _vHd[r].setConstant(0.0f);
    _vtRef[r].setConstant(0.0f);
    _vtd[r].setConstant(0.0f);
    _tankH[r] = 0.0f;
    _alphaH[r] = 0.0f;
    _depthGain[r] = 0.0f;
    _Fm[r].setConstant(0.0f);
    _toolTipCorrectionOffset[r].setConstant(0.0f);
    _toolDir[r] << 0.0f, 0.0f, -1.0f;
    _toolDirIK[r] << 0.0f, 0.0f, -1.0f;
    _wrenchBias[r].setConstant(0.0f);
    _wrenchExtBias[r].setConstant(0.0f);
  }
  _stop = false;
  _firstGripper = false;
  _optitrackOK = false;
  _usePredefinedTrocars = false;
  _useTaskAdaptation = false;
  _firstColorMarkersPosition = false;

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
  _gripperClutchingOffset = 0.0f;
  _humanGripperClutchingOffset = 0.0f;


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
    _trocarPosition[LEFT] << -0.304, -0.432f, 0.696f-_toolOffsetFromEE[LEFT](2);
    temp << 0.265f,-0.490f,-0.830f;
    temp.normalize();
    _trocarOrientation[LEFT] << temp;

    _trocarPosition[RIGHT] << -0.308f, 0.471f, 0.741f-_toolOffsetFromEE[RIGHT](2);
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
  std::cerr << _beliefsC.size() << std::endl;
  _beliefsC.setConstant(0.0f);
  _beliefsC(0) = 1.0f;
  _dbeliefsC.resize(_nbTasks);
  _dbeliefsC.setConstant(0.0f);

  _vda.setConstant(0.0f);


  if(_toolsTracking == OPTITRACK_BASED)
  {
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

  }

  if(_useFranka)
  {
    _robotID = Utils<float>::ROBOT_ID::FRANKA_PANDA;
  }
  else
  {
    _robotID = Utils<float>::ROBOT_ID::KUKA_LWR;
  }


  if(!(_useRobot[LEFT] && _useRobot[RIGHT]))
  {
    _enableEECollisionAvoidance = false;
    _enableToolCollisionAvoidance = false;
  }

  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_linearMapping[r] == POSITION_VELOCITY)
    {
      _qpSolverRCMCollision[r] = new QpSolverRCMCollision(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance,
                                                          _enableWorkspaceCollisionAvoidance, _operationMinOffsetPVM[r],
                                                          _operationMaxOffsetPVM[r], _operationMinInsertion[r]);        
      _qpSolverRCMCollision2[r] = new QpSolverRCMCollision2(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance,
                                                          _enableWorkspaceCollisionAvoidance, _operationMinOffsetPVM[r],
                                                          _operationMaxOffsetPVM[r]);        
      _qpSolverRCMCollision3[r] = new QpSolverRCMCollision3(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance,
                                                          _enableWorkspaceCollisionAvoidance, _operationMinOffsetPVM[r],
                                                          _operationMaxOffsetPVM[r]);        
    }
    else
    {
      _qpSolverRCMCollision[r] = new QpSolverRCMCollision(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance);
      _qpSolverRCMCollision2[r] = new QpSolverRCMCollision2(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance);
      _qpSolverRCMCollision3[r] = new QpSolverRCMCollision3(_eeLinearVelocityLimit, _eeAngularVelocityLimit,
                                                          _enableEECollisionAvoidance, _eeSafetyCollisionDistance, 
                                                          _enableToolCollisionAvoidance, _toolSafetyCollisionDistance);
    }
  }

  _qpSolverRCM[LEFT].setRobot(_robotID);
  _qpSolverRCM[RIGHT].setRobot(_robotID);
  _qpSolverRCMCollision[LEFT]->setRobot(_robotID);
  _qpSolverRCMCollision[RIGHT]->setRobot(_robotID);
  _qpSolverRCMCollision2[LEFT]->setRobot(_robotID);
  _qpSolverRCMCollision2[RIGHT]->setRobot(_robotID);
  _qpSolverRCMCollision3[LEFT]->setRobot(_robotID);
  _qpSolverRCMCollision3[RIGHT]->setRobot(_robotID);
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
        if(c==add[r] && ((!_useFranka && _firstRobotPose[r])|| (_useFranka && _firstJointsUpdate[r])) && !_trocarsRegistered[r])
        {
          _trocarPosition[r] = _x[r];
          _trocarOrientation[r] = _wRb[r].col(2);
          if(r == RIGHT && _useRobot[LEFT] && _tool[LEFT] == CAMERA && _trocarsRegistered[LEFT])
          {
            Eigen::Vector3f offset, x, delta;
            offset << 0.0f, 0.2f, 0.0f;
            x = _trocarPosition[LEFT]+offset-_x[RIGHT];
            _toolTipCorrectionOffset[r] = _wRb[RIGHT].transpose()*x;
            std::cerr << "Delta:" << _toolTipCorrectionOffset[r].transpose() << std::endl;
            // _trocarPosition[r] = _trocarPosition[LEFT]+offset;
            _trocarPosition[r] = _x[r]+_wRb[r]*_toolTipCorrectionOffset[r];
            // _toolTipCorrectionOffset[r] = _x[LEFT]+offset;
          }
          std::cerr << r << ": Adding trocar: " << _trocarPosition[r].transpose() << std::endl;         
        }
        else if(c==finish[r])
        {

          if(_trocarPosition[r].norm()>FLT_EPSILON)
          {
            _trocarsRegistered[r] = true;
            _toolOffsetFromEE[r] += _toolTipCorrectionOffset[r];
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