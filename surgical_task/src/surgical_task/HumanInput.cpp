#include "SurgicalTask.h"


void SurgicalTask::processHumanInput()
{
  if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
  {
  	dominantFootTwoRobots();
  }
  else if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
  {
  	singleFootSingleRobot();
  }
}


void SurgicalTask::dominantFootTwoRobots()
{

  _oldTrocarInput[_nonDominantInputID] = _trocarInput[_nonDominantInputID];
  _oldTrocarInput[_dominantInputID] = _trocarInput[_dominantInputID];
	
  // Get non dominant foot input
 	_trocarInput[_nonDominantInputID] = _footPose[_nonDominantInputID];

  if(_humanInputDevice[_nonDominantInputID] == FOOT)
  {
   // Scale human input between -1 and 1
    for(int k = 0; k < 5; k++)
    {
      _trocarInput[_nonDominantInputID](k) = Utils<float>::bound(2*_trocarInput[_nonDominantInputID](k)/_footInterfaceRange[_nonDominantInputID][k], -1.0f, 1.0f);
    }  
  }



  if(std::fabs(_trocarInput[_nonDominantInputID](_switchingAxis)) > std::fabs(_switchingThreshold[_currentRobot]) &&
     std::fabs(_oldTrocarInput[_nonDominantInputID](_switchingAxis)) < std::fabs(_switchingThreshold[_currentRobot]) &&
     _trocarInput[_nonDominantInputID](_switchingAxis)*_switchingThreshold[_currentRobot]>0 && !_clutching)
  {
    if(_currentRobot == RIGHT && _linearMapping[LEFT] == POSITION_POSITION)
    {
      _switching = true;
      _wait = false;
      _currentRobot = LEFT;
    }
    else if(_currentRobot == RIGHT && _linearMapping[LEFT] == POSITION_VELOCITY)
    {
      _wait = true;
      _currentRobot = LEFT;
      // _useTaskAdaptation = false;
    }
    else if(_currentRobot == LEFT && _linearMapping[RIGHT] == POSITION_POSITION)
    {
      _switching = true;
      _wait = false;
      _currentRobot = RIGHT;
    }
    else if(_currentRobot == LEFT && _linearMapping[RIGHT] == POSITION_VELOCITY)
    {
      _wait = true;
      _currentRobot = RIGHT;
      // _useTaskAdaptation = false;
    }
  }

  // Check for switching to left robot
  // if(_trocarInput[_nonDominantInputID](X) < -0.7f)
  // {
  // 	// If left robot has position position mapping, we directly switch otherwise we wait for the foot to come back
  // 	// zero => haptic cues might be needed
  //   if(_currentRobot == RIGHT && _linearMapping[LEFT] == POSITION_POSITION)
  //   {
  //     _switching = true;
  //   }
  //   else if(_currentRobot == RIGHT && _linearMapping[LEFT] == POSITION_VELOCITY)
  //   {
  //     _wait = true;
  //   }
  //   _currentRobot = LEFT;
  // }
  // else if(_trocarInput[_nonDominantInputID](X) > 0.7f)
  // {
  //   if(_currentRobot == LEFT && _linearMapping[RIGHT] == POSITION_POSITION)
  //   {
  //     _switching = true;
  //   }
  //   else if(_currentRobot == LEFT && _linearMapping[RIGHT] == POSITION_VELOCITY)
  //   {
  //     _wait = true;
  //   }
  //   _currentRobot = RIGHT;
  // }

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: Tool clutching offset: " << _toolClutchingOffset.transpose() << std::endl;
  }




  // Check for clutching (only for robot with POSITION-POSITION mapping)

  if(std::fabs(_trocarInput[_nonDominantInputID](_clutchingAxis)) > std::fabs(_clutchingDeactivationThreshold) &&
     std::fabs(_oldTrocarInput[_nonDominantInputID](_clutchingAxis)) < std::fabs(_clutchingDeactivationThreshold) &&
     _trocarInput[_nonDominantInputID](_clutchingAxis)*_clutchingDeactivationThreshold>0 &&
     _linearMapping[_currentRobot]== POSITION_POSITION && _clutching)
  {
    _clutching = false;
  }
  else if(std::fabs(_trocarInput[_nonDominantInputID](_clutchingAxis)) > std::fabs(_clutchingActivationThreshold) &&
          std::fabs(_oldTrocarInput[_nonDominantInputID](_clutchingAxis)) < std::fabs(_clutchingActivationThreshold) &&
          _trocarInput[_nonDominantInputID](_clutchingAxis)*_clutchingActivationThreshold>0 &&
          _linearMapping[_currentRobot]== POSITION_POSITION && !_clutching)
  {
    _clutching = true;
    _toolClutchingOffset.segment(0,3) = _desiredOffsetPPM[_currentRobot]; 
    _toolClutchingOffset(SELF_ROTATION) = _desiredAnglePPM[_currentRobot];
    _gripperClutchingOffset = _desiredGripperPosition[_currentRobot];
  }


  if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS && _linearMapping[_currentRobot] == POSITION_POSITION
     && _controlPhase[_currentRobot] == OPERATION)
  {
    _desiredGripperPosition[_currentRobot] = _gripperRange*std::max(0.0f,-_trocarInput[_nonDominantInputID](_gripperControlAxis));

    // if(_clutching)
    // {
    //   _trocarInput[_nonDominantInputID](_gripperControlAxis) = _footPose[_nonDominantInputID](_gripperControlAxis);
    //   _trocarInput[_nonDominantInputID](_gripperControlAxis) = Utils<float>::bound(2*_trocarInput[_nonDominantInputID](_gripperControlAxis)/_footInterfaceRange[_nonDominantInputID][_gripperControlAxis], -2.0f, 2.0f);
    // }

    // if(!_clutching)
    // {

    // _desiredGripperPosition[_currentRobot] = Utils<float>::bound(-_gripperRange*_trocarInput[_nonDominantInputID](_gripperControlAxis)+_gripperClutchingOffset,
    //                                                              0.0, _gripperRange);
    // }
  } 
  else
  {
    _desiredGripperPosition[_currentRobot] = _gripperRange;
  }
  // if(_trocarInput[_nonDominantInputID](Y) < -0.7f && _linearMapping[_currentRobot]== POSITION_POSITION)
  // {
  //   _clutching = false;
  // }
  // else if(_trocarInput[_nonDominantInputID](Y) > 0.7f && _linearMapping[_currentRobot]== POSITION_POSITION)
  // {
  //   _clutching = true;
  // 	_toolClutchingOffset.segment(0,3) = _desiredOffsetPPM[_currentRobot]; 
  // 	_toolClutchingOffset(SELF_ROTATION) = _desiredAnglePPM[_currentRobot];
  // }

  computeTrocarInput(_currentRobot,_dominantInputID);
}


void SurgicalTask::singleFootSingleRobot()
{

  Eigen::Matrix<float,5,5> R;

  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {
      _oldTrocarInput[_humanInputID[r]] = _trocarInput[_humanInputID[r]];
    	computeTrocarInput(r, _humanInputID[r]);
    }
  }
}


void SurgicalTask::computeTrocarInput(int r, int h)
{
 	Eigen::Matrix<float,5,5> R;

  if(_humanInputDevice[h] == FOOT)
  {
    if(_linearMapping[r] == POSITION_POSITION)
    {
      // X, Y, Z, SELF_ROTATION, EXTRA_DOF <= Y, -X, PITCH, -YAW, ROLL
      R = _footPPMapping;
      // R << 0.0f, -1.0f, 0.0f, 0.0f, 0.0f,
      //      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
      //      0.0f, 0.0f, 0.0f, 1.0f, 0.0f;
      // R << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
      //      -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
      //      0.0f, 0.0f, 0.0f, 1.0f, 0.0f;

      _trocarInput[h] = _footPose[h];

      if(_selfRotationMapping[h]==POSITION_VELOCITY)
      {
        _trocarInput[h](FOOT_YAW) = Utils<float>::deadZone(_trocarInput[h](FOOT_YAW), _footInterfaceMinDeadZone[h][FOOT_YAW], _footInterfaceMaxDeadZone[h][FOOT_YAW]);
      }

      _trocarInput[h] = R*_trocarInput[h];

      if(_clutching && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
        _humanClutchingOffset = _trocarInput[h];
      }

      if(_switching  && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
        _switching = false;
        _humanClutchingOffset = _trocarInput[h];
    		_toolClutchingOffset.segment(0,3) = _desiredOffsetPPM[_currentRobot]; 
    		_toolClutchingOffset(SELF_ROTATION) = _desiredAnglePPM[_currentRobot];
        _gripperClutchingOffset = _desiredGripperPosition[_currentRobot];
      }

      if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
      	_trocarInput[h] -= _humanClutchingOffset;
      }

      // Scale human input between -1 and 1
      _trocarInput[h](X) = Utils<float>::bound(2*_trocarInput[h](X)/_footInterfaceRange[h][FOOT_Y], -2.0f, 2.0f);
      _trocarInput[h](Y) = Utils<float>::bound(2*_trocarInput[h](Y)/_footInterfaceRange[h][FOOT_X], -2.0f, 2.0f);
      _trocarInput[h](Z) = Utils<float>::bound(2*_trocarInput[h](Z)/_footInterfaceRange[h][FOOT_PITCH], -2.0f, 2.0f);
      // If a linear position-position mapping is desired for the foot we allow for a poosition-position or position-velocity
      // mapping for the self rotation 
      if(_selfRotationMapping[h]==POSITION_POSITION)
      {
        _trocarInput[h](SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[h](SELF_ROTATION)/_footInterfaceRange[h][FOOT_YAW], -2.0f, 2.0f);
      }
      else
      {
        _trocarInput[h](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[h](W_SELF_ROTATION)/(_footInterfaceRange[h][FOOT_YAW]-_footInterfaceMaxDeadZone[h][FOOT_YAW]+_footInterfaceMinDeadZone[h][FOOT_YAW]), -1.0f, 1.0f);
      }
      _trocarInput[h](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[h](EXTRA_DOF)/_footInterfaceRange[h][FOOT_ROLL], -1.0f, 1.0f);
    }
    else
    {
      // V_UP, V_RIGHT, V_INSERTION, W_SELF_ROTATION <= PITCH, -ROLL, Y, -YAW
      R = _footPVMapping;
      // R << 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
      //      1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
      //      0.0f, 0.0f, 0.0f, 1.0f, 0.0f;
      // R << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
      //      0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
      //      0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
      //      1.0f, 0.0f, 0.0f, 0.0f, 0.0f;

      // Apply deadzone on foot position
      // _trocarInput[h] = R*_footPose[h];
      // _trocarInput[h](V_UP) = Utils<float>::deadZone(_trocarInput[h](V_UP), _footInterfaceMinDeadZone[h][FOOT_Y], _footInterfaceMaxDeadZone[h][FOOT_Y]);
      // _trocarInput[h](V_RIGHT) = Utils<float>::deadZone(_trocarInput[h](V_RIGHT), _footInterfaceMinDeadZone[h][FOOT_X], _footInterfaceMaxDeadZone[h][FOOT_X]);
      // _trocarInput[h](V_INSERTION) = Utils<float>::deadZone(_trocarInput[h](V_INSERTION), _footInterfaceMinDeadZone[h][FOOT_PITCH], _footInterfaceMaxDeadZone[h][FOOT_PITCH]);
      // _trocarInput[h](W_SELF_ROTATION) = Utils<float>::deadZone(_trocarInput[h](W_SELF_ROTATION), _footInterfaceMinDeadZone[h][FOOT_YAW], _footInterfaceMaxDeadZone[h][FOOT_YAW]);


      _trocarInput[h] = _footPose[h];
      for(int k = 0; k < NB_DOF_FOOT_INTERFACE; k++)
      {
        _trocarInput[h](k) = Utils<float>::deadZone(_trocarInput[h](k), _footInterfaceMinDeadZone[h][k], _footInterfaceMaxDeadZone[h][k]);
      }
      _trocarInput[h] = R*_trocarInput[h];


      // Scale human input between -1 and 1
      _trocarInput[h](V_UP) = Utils<float>::bound(2*_trocarInput[h](V_UP)/(_footInterfaceRange[h][FOOT_Y]-_footInterfaceMaxDeadZone[h][FOOT_Y]+_footInterfaceMinDeadZone[h][FOOT_Y]), -1.0f, 1.0f);
      _trocarInput[h](V_RIGHT) = Utils<float>::bound(2*_trocarInput[h](V_RIGHT)/(_footInterfaceRange[h][FOOT_X]-_footInterfaceMaxDeadZone[h][FOOT_X]+_footInterfaceMinDeadZone[h][FOOT_X]), -1.0f, 1.0f);
      _trocarInput[h](V_INSERTION) = Utils<float>::bound(2*_trocarInput[h](V_INSERTION)/(_footInterfaceRange[h][FOOT_PITCH]-_footInterfaceMaxDeadZone[h][FOOT_PITCH]+_footInterfaceMinDeadZone[h][FOOT_PITCH]), -1.0f, 1.0f);
      _trocarInput[h](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[h](W_SELF_ROTATION)/(_footInterfaceRange[h][FOOT_YAW]-_footInterfaceMaxDeadZone[h][FOOT_YAW]+_footInterfaceMinDeadZone[h][FOOT_YAW]), -1.0f, 1.0f);
      _trocarInput[h](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[h](EXTRA_DOF)/_footInterfaceRange[h][FOOT_ROLL], -1.0f, 1.0f);      

      // // Apply deadzone on foot position
      // _trocarInput[h](V_UP) = Utils<float>::deadZone(_trocarInput[h](V_UP), _footInterfaceMinDeadZone[h][FOOT_PITCH], _footInterfaceMaxDeadZone[h][FOOT_PITCH]);
      // _trocarInput[h](V_RIGHT) = Utils<float>::deadZone(_trocarInput[h](V_RIGHT), _footInterfaceMinDeadZone[h][FOOT_ROLL], _footInterfaceMaxDeadZone[h][FOOT_ROLL]);
      // _trocarInput[h](V_INSERTION) = Utils<float>::deadZone(_trocarInput[h](V_INSERTION), _footInterfaceMinDeadZone[h][FOOT_Y], _footInterfaceMaxDeadZone[h][FOOT_Y]);
      // _trocarInput[h](W_SELF_ROTATION) = Utils<float>::deadZone(_trocarInput[h](W_SELF_ROTATION), _footInterfaceMinDeadZone[h][FOOT_YAW], _footInterfaceMaxDeadZone[h][FOOT_YAW]);

      // // Scale human input between -1 and 1
      // _trocarInput[h](V_UP) = Utils<float>::bound(2*_trocarInput[h](V_UP)/(_footInterfaceRange[h][FOOT_PITCH]-2*_footInterfaceMaxDeadZone[h][FOOT_PITCH]), -1.0f, 1.0f);
      // _trocarInput[h](V_RIGHT) = Utils<float>::bound(2*_trocarInput[h](V_RIGHT)/(_footInterfaceRange[h][FOOT_ROLL]-2*_footInterfaceMaxDeadZone[h][FOOT_ROLL]), -1.0f, 1.0f);
      // _trocarInput[h](V_INSERTION) = Utils<float>::bound(2*_trocarInput[h](V_INSERTION)/(_footInterfaceRange[h][FOOT_Y]-2*_footInterfaceMaxDeadZone[h][FOOT_Y]), -1.0f, 1.0f);
      // _trocarInput[h](W_SELF_ROTATION) = Utils<float>::bound(2*_trocarInput[h](W_SELF_ROTATION)/(_footInterfaceRange[h][FOOT_YAW]-2*_footInterfaceMaxDeadZone[h][FOOT_YAW]), -1.0f, 1.0f);
      // _trocarInput[h](EXTRA_DOF) = Utils<float>::bound(2*_trocarInput[h](EXTRA_DOF)/_footInterfaceRange[h][FOOT_X], -1.0f, 1.0f);      

      if(_wait && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
         if(_debug)
        {
          std::cerr <<  "[SurgicalTask]: Wait dominant foot to come back to zero !!!" << std::endl;
        }
        if(_trocarInput[h].segment(0,4).norm()<0.2f)
        {
          _wait = false;
        }
      }
    }
  }
  else
  {
    if(_linearMapping[r] == POSITION_POSITION)
    {
      R <<  0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f;          
    
      _trocarInput[h] = R*_footPose[h];

      if(_clutching  && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
        _humanClutchingOffset = _trocarInput[h];
      }

      if(_switching  && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
        _switching = false;
        _humanClutchingOffset = _trocarInput[h];
    		_toolClutchingOffset.segment(0,3) = _desiredOffsetPPM[_currentRobot]; 
    		_toolClutchingOffset(SELF_ROTATION) = _desiredAnglePPM[_currentRobot];
        _gripperClutchingOffset = _desiredGripperPosition[_currentRobot];
      }

      if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
      	_trocarInput[h] -= _humanClutchingOffset;
      }


      for(int k = 0; k <_trocarInput[h].size(); k++)
      {
        _trocarInput[h](k) = Utils<float>::bound(_trocarInput[h](k),-1.0,1.0);
      }
    }
    else
    {
      R <<  0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
            -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f; 

      _trocarInput[h] = R*_footPose[h];

      if(_wait && _humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
         if(_debug)
        {
          std::cerr <<  "[SurgicalTask]: Wait dominant foot to come back to zero !!!" << std::endl;
        }
        
        if(_trocarInput[h].segment(0,4).norm()<0.2f)
        {
          _wait = false;
        }
      }
    }
  }
}


void SurgicalTask::computeDesiredFootWrench(int r, int h)
{
  _desiredFootWrench[h].setConstant(0.0f);

  Eigen::Matrix4f G;
  G.setIdentity();
  G(2,2) = 0.2f;


  Eigen::Vector4f temp;

  temp(3) = _taud[r];


  switch(_tool[r])
  {
    case CAMERA:
    {
      temp.segment(0,3) = (_wRb[r]*_eeCameraMapping).transpose()*_FdFoot[r];
      _desiredFootWrench[h] = _footPVMapping.block(0,0,4,NB_DOF_FOOT_INTERFACE).transpose()*G*temp;
      temp.setConstant(0.0f);
      temp.segment(0,3) = (_wRb[r]*_eeCameraMapping).transpose()*_Fm[r];
      _toolToFootTorques[h] = _footPVMapping.block(0,0,4,NB_DOF_FOOT_INTERFACE).transpose()*G*temp;

      break;
    }
    case RETRACTOR:
    {
      temp.segment(0,3) = _FdFoot[r];
      _desiredFootWrench[h] = _footPPMapping.block(0,0,4,NB_DOF_FOOT_INTERFACE).transpose()*G*temp;
      temp.setConstant(0.0f);
      temp.segment(0,3) = _Fm[r];
      _toolToFootTorques[h] = _footPPMapping.block(0,0,4,NB_DOF_FOOT_INTERFACE).transpose()*G*temp;
      break;
    }
    default:
    {
      break;
    }
  }

  for(int k = 0; k < 2; k++)
  {
    _desiredFootWrench[h](k) = Utils<float>::bound(_desiredFootWrench[h](k), -15.0f, 15.0f);
    _toolToFootTorques[h](k) = Utils<float>::bound(_toolToFootTorques[h](k), -15.0f, 15.0f);
  }

  for(int k = 0 ; k < 3; k++)
  {
    _desiredFootWrench[h](k+2) = Utils<float>::bound(_desiredFootWrench[h](k+2), -2.0f, 2.0f);
    _toolToFootTorques[h](k+2) = Utils<float>::bound(_toolToFootTorques[h](k+2), -2.0f, 2.0f);
  }
}