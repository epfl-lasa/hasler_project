#include "SurgicalTask.h"


void SurgicalTask::robotControlStep(int r, int h)
{
  // Update trocar information
  updateRobotTaskState(r);

  // Select robot mode
  updateControlPhase(r);

  switch(_controlPhase[r])
  {
    case AUTOMATIC_INSERTION:
    {
      automaticInsertionStep(r, h);
      break;
    }
    case OPERATION:
    {
      if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT || (_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS && !_clutching && ! _wait))
      {
        if((_humanInputDevice[r] == FOOT && _footState[h] == 2 &&  _subFootOutput[h].getNumPublishers() > 0) || _humanInputDevice[r] == JOYSTICK)
        {
          operationStep(r, h);
        }
      }
      break;
    }
    default:
    {
      break;
    }
  }

  if(_humanInputDevice[r] == FOOT && !_useSim)
  {
    // Compute haptic feedback in robot world frame
    computeHapticFeedback(r);

    // Compute haptic foot torques
    computeDesiredFootTorques(r, h);
  }
}


void SurgicalTask::updateRobotTaskState(int r)
{
  // Compute vector EE to trocar
  _rEETrocar[r] = _trocarPosition[r]-_xEE[r];

  // Compute tool direction
  _toolDir[r] = (_wRb[r]*_toolOffsetFromEE[r]).normalized();

  // Compute RCM position
  _xRCM[r] = _xEE[r]+(_trocarPosition[r]-_xEE[r]).dot(_toolDir[r])*_toolDir[r];
  
  // Compute vector EE to RCM
  _rEERCM[r] = _xRCM[r]-_xEE[r];

  // Compute distance RCM tool
  _dRCMTool[r] = (_xRCM[r]-_x[r]).dot(_toolDir[r]);

  // Compute IK EE/tip position and orientation
  Eigen::Matrix4f Hik;
  Hik = Utils<float>::getForwardKinematics(_ikJoints[r],_robotID); 
  _xEEIK[r] =  _xRobotBaseOrigin[r]+_wRRobotBasis[r]*Hik.block(0,3,3,1);
  _wRbIK[r] = _wRRobotBasis[r]*Hik.block(0,0,3,3);
  _xIK[r] = _xEEIK[r]+_wRbIK[r]*_toolOffsetFromEE[r];
  _toolDirIK[r] = (_wRbIK[r]*_toolOffsetFromEE[r]).normalized();

  // Compute depth gain
  if(_controlStrategy[r] == PASSIVE_DS)
  {
    _depthGain[r] = std::min(std::max((_x[r]-_trocarPosition[r]).dot(_wRb[r].col(2)),0.0f)*4.0f/_toolOffsetFromEE[r].norm(),1.0f);
  }
  else
  {
    _depthGain[r] = std::min(std::max((_xIK[r]-_trocarPosition[r]).dot(_wRbIK[r].col(2)),0.0f)*4.0f/_toolOffsetFromEE[r].norm(),1.0f);    
  }

  if(_debug)
  {
  	std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-tool: " << _dRCMTool[r] << std::endl;
  	std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-trocar: " << (_trocarPosition[r]-_xRCM[r]).norm() <<std::endl;    
    std::cerr << "[SurgicalTask]: " << r << ": depth gain: " <<  _depthGain[r] << std::endl;  
    std::cerr << "[SurgicalTask]: " << r << " xIK: " << _xIK[r].transpose() << std::endl;
  }

  if(_useRobot[LEFT] && _useRobot[RIGHT])
  {
    computeToolCollision(r);
  }
}


void SurgicalTask::computeToolCollision(int r)
{
  Eigen::Vector3f r21, e1, e2;
  if(r == LEFT)
  {
    if(_controlStrategy[r] == PASSIVE_DS)
    {
      r21 = _xEE[r]-_xEE[RIGHT];
      e1 = _toolDir[r];
      e2 = _toolDir[RIGHT];
    }
    else
    {
      r21 = _xEEIK[r]-_xEEIK[RIGHT];
      e1 = _toolDirIK[r];
      e2 = _toolDirIK[RIGHT];
    }
  }
  else if (r == RIGHT)
  {
    if(_controlStrategy[r] == PASSIVE_DS)
    {

      r21 = _xEE[r]-_xEE[LEFT];
      e1 = _toolDir[r];
      e2 = _toolDir[LEFT];
    }
    else
    {
      r21 = _xEEIK[r]-_xEEIK[LEFT];
      e1 = _toolDirIK[r];
      e2 = _toolDirIK[LEFT];
    }
  }

  _rEECollision[r] = (r21.norm()-2.0f*_eeSafetyCollisionRadius)*r21.normalized();
  _nEECollision[r] = r21.normalized();
  _dEECollision[r] = r21.norm()-2.0f*_eeSafetyCollisionRadius;

  float l1 = 0.0f, l2 = 0.0f;
  float den = 1.0f-std::pow(e1.dot(e2),2.0f);

  if(den > FLT_EPSILON)
  {
    l1 = -(r21.dot(e1)-e1.dot(e2)*r21.dot(e2))/den;
    l2 = (r21.dot(e2)-e1.dot(e2)*r21.dot(e1))/den;

    float l2Max = (r == LEFT) ? _toolOffsetFromEE[RIGHT].norm() : _toolOffsetFromEE[LEFT].norm();

    if (l1 > _toolOffsetFromEE[r].norm())
    {
      l1 = _toolOffsetFromEE[r].norm();
      l2 = r21.dot(e2)+l1*e1.dot(e2);
    }

    if (l2 > l2Max)
    {
      l2 = l2Max;
      l1 = l2*e1.dot(e2)-r21.dot(e1);
    }

    l1 = std::max(0.0f, std::min(l1, _toolOffsetFromEE[r].norm()));
    l2 = std::max(0.0f, std::min(l2, l2Max));

    _rToolCollision[r] = r21+l1*e1-l2*e2;
    _toolCollisionOffset[r] = l1*e1-_rToolCollision[r].normalized()*_toolSafetyCollisionRadius;
  }
  else
  {
    _rToolCollision[r] = r21;
    _toolCollisionOffset[r] = _toolOffsetFromEE[r].norm()*e1-_rToolCollision[r].normalized()*_toolSafetyCollisionRadius;
  }

  _nToolCollision[r] = _rToolCollision[r].normalized();
  _dToolCollision[r] = _rToolCollision[r].norm()-2*_toolSafetyCollisionRadius;

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": Distance EE-Robot: " << _rEECollision[r].norm() << " " << _rEECollision[r].norm() <<std::endl;
    std::cerr << "[SurgicalTask]: " << r << ": Distance Tool-Tool: " << _rToolCollision[r].transpose() << _rToolCollision[r].norm() << " " << l1 << " " << l2 <<std::endl;    
  }
}


void SurgicalTask::updateControlPhase(int r)
{
  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": trocar " << _trocarPosition[r].transpose() << std::endl;
    std::cerr << "[SurgicalTask]: " << r << ": x " << _x[r].transpose() << std::endl;    
  }

  if(_controlPhase[r] == AUTOMATIC_INSERTION)
  {
    if(_insertionFinished[r])
    {
      _controlPhase[r] = OPERATION;
      _wRb0[r] = _wRb[r];
      if(_controlStrategy[r] == PASSIVE_DS)
      {
        _xd0[r] = _x[r];
      }
      else
      {
        _xd0[r] = _xIK[r];
      }
      if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
      	_clutching = true;
      	computeTrocarInput(_currentRobot,_dominantInputID);
      	_clutching = false;
      }
    }
  }
}


void SurgicalTask::automaticInsertionStep(int r, int h)
{

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": AUTOMATIC INSERTION" << std::endl;
  }

  Eigen::Vector3f x;
  Eigen::Matrix3f wRb;
  float alpha = 1.0f;
  if(_controlStrategy[r] == PASSIVE_DS)
  {  
    x = _x[r];
    wRb = _wRb[r];
    alpha = 5.0f;
  }
  else
  {
    x = _xIK[r];
    wRb = _wRbIK[r];
  }

  // Compute attractor position
  if(_linearMapping[r] == POSITION_VELOCITY)
  {
    _xd[r] = _xd0[r]+_wRb0[r].col(2)*(-_insertionDistancePVM[r]);
  }
  else
  {
    _xd[r] = _xd0[r]+_insertionOffsetPPM[r];
  }

  // Compute reference task velocity
  _vtRef[r] = alpha*(_xd[r]-x); 

  if(_vtRef[r].norm()/alpha<0.005)
  {
    _insertionFinished[r] = true;
  }

  // Scale reference task velocity to take into account fulcrum effect
  _vtd[r] = fulcrumEffectScaling(r, _vtRef[r]);
 
  // Bound task velocity
  _vtd[r] = Utils<float>::bound(_vtd[r],_toolTipLinearVelocityLimit);
  
  // Compute final desired tool tip velocity
  _vdTool[r] = _vtd[r];

  if(_controlStrategy[r] == PASSIVE_DS)
  {

    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    // _stiffness[r].setConstant(0.0f);

    // Eigen::Matrix<float,6,6> A;
    // A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2))*Eigen::Matrix3f::Identity();
    // A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r]);
    // A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
    // A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_toolOffsetFromEE[r]*_wRb[r].col(2));
    // Eigen::Matrix<float,6,1> x, b;
    // b.setConstant(0.0f);
    // b.segment(3,3) = _vdTool[r];

    // x = A.fullPivHouseholderQr().solve(b);
    // _vd[r] = x.segment(0,3);
    // _omegad[r] = x.segment(3,3);

    
    // _vd[r]+=10.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_trocarPosition[r]-_xRCM[r]);

    // _vd[r] = Utils<float>::bound(_vd[r],0.4f);

    // _omegad[r] = Utils<float>::bound(_omegad[r],3.0f);

    // std::cerr << _toolOffsetFromEE[r] << std::endl;
    // std::cerr << _omegad[r].transpose() << std::endl;
    // _omegad[r] = Utils<float>::bound(_omegad[r],1.0f);
    // std::cerr << _rEERCM[r].transpose() << std::endl;
    // std::cerr << _vdTool[r].transpose() << std::endl;
    // std::cerr << A << std::endl;

    // _nullspaceWrench[r].setConstant(0.0f);

    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(wRb.col(2),_rEETrocar[r].normalized()));

    Eigen::Vector3f axis;  
    float angleErrorToTrocarPosition;
    Utils<float>::quaternionToAxisAngle(qe,axis,angleErrorToTrocarPosition);

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;
    }

    if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
    {
      qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                       -MAX_ORIENTATION_ERROR,
                                                                       MAX_ORIENTATION_ERROR));
    }

    // Compute final quaternion
    _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

    _selfRotationCommand[r] = 0.0f;
  }
  else if (_controlStrategy[r] == JOINT_IMPEDANCE && _firstPublish[r])
  {
    _stiffness[r] = Eigen::Map<Eigen::Matrix<float, 7, 1> >(_jointImpedanceStiffnessGain.data());
  
    _selfRotationCommand[r] = 0.0f;

    _qpResult[r] = _qpSolverRCMCollision[r]->step(_ikJoints[r], _ikJoints[r], _currentJoints[r], _trocarPosition[r], _toolOffsetFromEE[r], _vdTool[r],
                                                  _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f, _nEECollision[r], 
                                                  _dEECollision[r], -_eeSafetyCollisionRadius*_rEECollision[r].normalized(), _nToolCollision[r],
                                                  _dToolCollision[r], _toolCollisionOffset[r]);

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      std::cerr << "[SurgicalTask]: " << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;      
    }
  }
  else
  {
    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _qd[r] = _q[r];  
    _ikJoints[r] = _currentJoints[r];
  }

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _inputAlignedWithOrigin[r] = false;

  _desiredGripperPosition[r] = _gripperRange;
}


void SurgicalTask::operationStep(int r, int h)
{
  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": OPERATION" << std::endl;
  }

  // Compute desired tool tip task velocity
  computeDesiredToolVelocity(r, h);

  // Compute admittance velocity if enable human physical interaction
  if(_enablePhysicalHumanInteraction[r])
  {
    computeAdmittanceVelocity(r);
  }
  
  // Compute final desired tool tip velocity
  if(_enablePhysicalHumanInteraction[r])
  {
    _vdTool[r] = (1-_alphaH[r])*_vtd[r]+ _vHd[r];
  }
  else
  {
    _vdTool[r] = _vtd[r];
  }


  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": vtd: " << _vtd[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 
    std::cerr << "[SurgicalTask]: " << r << ": vd tool: " << _vdTool[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 
  }

  // Compute desired gripper position
  if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
  {
    _desiredGripperPosition[r] = _gripperRange*(std::max(0.0f,_trocarInput[h](EXTRA_DOF)));
  }

  if (_controlStrategy[r] == PASSIVE_DS)
  {
    _stiffness[r].setConstant(0.0f);
    Eigen::Matrix<float,6,6> A;
    A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_toolDir[r])*Eigen::Matrix3f::Identity();
    A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_toolDir[r])*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r]);
    A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
    A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_wRb[r]*_toolOffsetFromEE[r]);
    Eigen::Matrix<float,6,1> x, b;
    b.setConstant(0.0f);
    b.segment(3,3) = _vdTool[r];

    x = A.fullPivHouseholderQr().solve(b);

    _vd[r] = x.segment(0,3);
    _omegad[r] = x.segment(3,3);

    
    _vd[r]+=10.0f*Utils<float>::orthogonalProjector(_toolDir[r])*(_trocarPosition[r]-_xRCM[r]);

    _vd[r] = Utils<float>::bound(_vd[r],0.4f);


    _omegad[r] = Utils<float>::bound(_omegad[r],3.0f);

    _nullspaceWrench[r].setConstant(0.0f);

    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_toolDir[r],_rEETrocar[r].normalized()));

    Eigen::Vector3f axis;  
    float angleErrorToTrocarPosition;
    Utils<float>::quaternionToAxisAngle(qe,axis,angleErrorToTrocarPosition);

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;
    }

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

    _stiffness[r] = Eigen::Map<Eigen::Matrix<float, 7, 1> >(_jointImpedanceStiffnessGain.data());

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    _qpResult[r] = _qpSolverRCMCollision[r]->step(_ikJoints[r], _ikJoints[r], _currentJoints[r], _trocarPosition[r], _toolOffsetFromEE[r], _vdTool[r],
                                                  _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f, _nEECollision[r], 
                                                  _dEECollision[r], -_eeSafetyCollisionRadius*_rEECollision[r].normalized(), _nToolCollision[r],
                                                  _dToolCollision[r], _toolCollisionOffset[r], true, _xIK[r]-_xd0[r]);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      std::cerr << "[SurgicalTask]: " << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;      
    }
  }
  else
  {
    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _ikJoints[r] = _currentJoints[r];
  }
}


void SurgicalTask::computeDesiredToolVelocity(int r, int h)
{

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: trocar input:  " << _trocarInput[h].transpose() << std::endl;    
  }

  Eigen::Matrix3f wRb;
  if(_controlStrategy[r] == PASSIVE_DS)
  {
    wRb = _wRb[r];
  }
  else
  {
    wRb = _wRbIK[r];
  }

  if(_linearMapping[r] == POSITION_VELOCITY)
  {
    Eigen::Vector3f gains;
    gains << _trocarSpaceVelocityGains[V_UP], _trocarSpaceVelocityGains[V_RIGHT], _trocarSpaceVelocityGains[V_INSERTION];

    _vtRef[r] = wRb*_eeCameraMapping*(gains.cwiseProduct(_trocarInput[h].segment(0,3)));   


    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << " Current offset: " << (_x[r]-_xd0[r]).transpose() << std::endl;
    }

    _selfRotationCommand[r] = _trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[h](W_SELF_ROTATION);
    _selfRotationCommand[r] = Utils<float>::bound(_selfRotationCommand[r],-_toolTipSelfAngularVelocityLimit,_toolTipSelfAngularVelocityLimit);
    if(_ikJoints[r](6)>_trocarSpaceSelfRotationRange*M_PI/180.0f && _selfRotationCommand[r]>0.0f)
    {
      _selfRotationCommand[r] = 0.0f;
    }
    else if(_ikJoints[r](6)<-_trocarSpaceSelfRotationRange*M_PI/180.0f && _selfRotationCommand[r]<0.0f)
    {
      _selfRotationCommand[r] = 0.0f;
    }

    if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS && r != _currentRobot && _tool[r]==CAMERA)
    {
      _selfRotationCommand[r] = 0.0f;
    }

    if(_tool[r]==CAMERA && _allowTaskAdaptation)
    {
      if(std::fabs(_trocarInput[h](EXTRA_DOF))>std::fabs(_taskAdaptationDeactivationThreshold) &&
         std::fabs(_oldTrocarInput[h](EXTRA_DOF))<std::fabs(_taskAdaptationDeactivationThreshold) &&
         _trocarInput[h](EXTRA_DOF)*_taskAdaptationDeactivationThreshold>0 && _useTaskAdaptation && r == _currentRobot)
      {
        _useTaskAdaptation = false;
      }
      else if(std::fabs(_trocarInput[h](EXTRA_DOF))>std::fabs(_taskAdaptationActivationThreshold) &&
              std::fabs(_oldTrocarInput[h](EXTRA_DOF))<std::fabs(_taskAdaptationActivationThreshold) &&
              _trocarInput[h](EXTRA_DOF)*_taskAdaptationActivationThreshold>0 && !_useTaskAdaptation && r == _currentRobot)
      {
        initializeBeliefs(r);
        _useTaskAdaptation = true;
      }


      if(_useTaskAdaptation)
      {

        taskAdaptation(r, h);

        if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT || (_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS && r == _currentRobot))
        {
          _vtRef[r] = gains(V_INSERTION)*_trocarInput[h](V_INSERTION)*wRb.col(V_INSERTION);
        }
        else
        {
          _vtRef[r].setConstant(0.0f);
        }
        
        if(!_useSim && _toolsTracking == CAMERA_BASED)
        {
          _vtRef[r] += wRb*_eeCameraMapping*_vda;

        }
        else
        {
          _vtRef[r] += _vda;
        }
      }   
    }

    // if(_dRCMTool[r]> _insertionDistancePVM[r] && (wRb.col(2)).dot(_vtRef[r])<0.0f)
    // {
    //   _vtRef[r] = Utils<float>::orthogonalProjector(wRb.col(2))*_vtRef[r];
    // }

  }
  else if(_linearMapping[r]==POSITION_POSITION)
  {

    float xMin,xMax,yMin,yMax;

    xMin = -_operationOffsetRangePPM[r](0)/2.0;
    xMax = _operationOffsetRangePPM[r](0)/2.0;
    yMin = -_operationOffsetRangePPM[r](1)/2.0;
    yMax = _operationOffsetRangePPM[r](1)/2.0;

    _desiredOffsetPPM[r](Z) = Utils<float>::bound(_operationOffsetRangePPM[r](2)*_trocarInput[h](Z)+_toolClutchingOffset(Z),
                                                  -_operationOffsetRangePPM[r](2),0.0);

    _desiredOffsetPPM[r](X) = Utils<float>::bound((xMin+xMax)/2+_trocarInput[h](X)*(xMax-xMin)/2+_toolClutchingOffset(X),
    	                                            (xMin+xMax)/2-(xMax-xMin)/2,(xMin+xMax)/2+(xMax-xMin)/2);


    _desiredOffsetPPM[r](Y) = Utils<float>::bound((yMin+yMax)/2+_trocarInput[h](Y)*(yMax-yMin)/2+_toolClutchingOffset(Y),
    	                                            (yMin+yMax)/2-(yMax-yMin)/2,(yMin+yMax)/2+(yMax-yMin)/2);

    // Compute real offset from end effector to inital target point
    Eigen::Vector3f currentOffset;
    currentOffset = _x[r]-_xd0[r];

    // Compute desired tip position
    _xd[r] = _xd0[r]+_desiredOffsetPPM[r];

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": Desired offset: " << _desiredOffsetPPM[r].transpose() << std::endl; 
      std::cerr << "[SurgicalTask]: " << r << ": Current offset: " << currentOffset.transpose() << std::endl;       
    }

    // To start accounting for the human input, the desired and real offset should be close
    // at the beginning
    if((_desiredOffsetPPM[r]-(_xIK[r]-_xd0[r])).norm()<0.005f)
    {
      _inputAlignedWithOrigin[r]=true;
      _wait = false;
    }

    if(_inputAlignedWithOrigin[r]==false)
    {
      _xd[r] = _xd0[r];
      _wait = true;
        
      if(_debug)
      {
        std::cerr << "[SurgicalTask]: " << r << "Input should be align with origin" << std::endl;
      }
    }
    
    float alpha = _trocarSpaceLinearDSFixedGain+_trocarSpaceLinearDSGaussianGain*std::exp(-(_xd[r]-_xIK[r]).squaredNorm()/(2.0f*std::pow(_trocarSpaceLinearDSGaussianWidth,2.0f)));  
    
    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": alpha: " << alpha << std::endl;
    }

    if(_controlStrategy[r] == JOINT_IMPEDANCE)
    {
      _vtRef[r] = alpha*(_xd[r]-_xIK[r]);        
      // _vtRef[r] = alpha*(_xd[r]-_x[r]);        
      // if(_colorMarkersStatus[2] == 0 && _toolsTracking == CAMERA_BASED && !_useSim)
      // {
      //   _vtRef[r].setConstant(0.0f);
      // }      
    }
    else
    {
      _vtRef[r] = alpha*(_xd[r]-_x[r]); 
    }
    
    if(_humanInputDevice[r] == JOYSTICK)
    {
      _selfRotationCommand[r] = 0.0f;
    }
    else
    {
      if(_selfRotationMapping[r] == POSITION_POSITION)
      {
      	_desiredAnglePPM[r] = Utils<float>::bound(_trocarInput[h](SELF_ROTATION)*_trocarSpaceSelfRotationRange+_toolClutchingOffset(SELF_ROTATION),
      		                                       -_trocarSpaceSelfRotationRange,_trocarSpaceSelfRotationRange);

        _selfRotationCommand[r] = _trocarSpaceSelfRotationGain*(_desiredAnglePPM[r]*M_PI/180.0f-_ikJoints[r](6));
        _selfRotationCommand[r]  = Utils<float>::bound(_selfRotationCommand[r] ,-_toolTipSelfAngularVelocityLimit, _toolTipSelfAngularVelocityLimit);        
      }
      else
      {
        _selfRotationCommand[r] =_trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[h](W_SELF_ROTATION);
        if(_ikJoints[r](6)>_trocarSpaceSelfRotationRange*M_PI/180.0f && _selfRotationCommand[r]>0.0f)
        {
          _selfRotationCommand[r] = 0.0f;
        }
        else if(_ikJoints[r](6)<-_trocarSpaceSelfRotationRange*M_PI/180.0f && _selfRotationCommand[r]<0.0f)
        {
          _selfRotationCommand[r] = 0.0f;
        }
      }
    }
  }
  else
  {
    _vtRef[r].setConstant(0.0f);
  }

  // Scale reference task velocity to take into account fulcrum effect
  _vtd[r] = fulcrumEffectScaling(r, _vtRef[r]);

  // Bound task velocity
  _vtd[r] = Utils<float>::bound(_vtd[r], _toolTipLinearVelocityLimit);
}


void SurgicalTask::computeAdmittanceVelocity(int r)
{
  Eigen::Vector3f Ft;
  if(_useFTSensor[r])
  {
    Ft = _wRb[r]*_wrench[r].segment(0,3);

    if(_debug)
    {
      std::cerr << "FT:  " << Ft.transpose() << std::endl;
      std::cerr << "Fext:  " << (-_wRb[r]*_Fext[r]).transpose() << std::endl;
      std::cerr << "Fext-FT:  " << (-_wRb[r]*_Fext[r]-Ft).transpose() << std::endl;      
    }
  }
  else
  {
    Ft.setConstant(0.0f);
  }

  Eigen::Matrix3f S, P;
  P = Utils<float>::orthogonalProjector(_toolDir[r]);
  S = P*Utils<float>::getSkewSymmetricMatrix((_trocarPosition[r]-_xEE[r]).dot(_toolDir[r])*_toolDir[r]);    
  Eigen::Vector3f omegaEEd, vEEdir, vTooldir;
  vEEdir = (-_wRb[r]*_Fext[r]-Ft).normalized();
  Eigen::Vector3f b;
  b = P*vEEdir;
  omegaEEd = S.fullPivHouseholderQr().solve(b);
  vTooldir = (vEEdir+omegaEEd.cross(_wRb[r]*_toolOffsetFromEE[r])).normalized();

  Eigen::Vector3f Fh;
  Fh.setConstant(0.0f);

  float forceDeadZone;

  if(_tool[r] == RETRACTOR && _msgGripperOutput.gripper_position >  15.0f)
  {
    forceDeadZone = 12.0f;
    std::cerr << "INCREASE DEADZONE" << std::endl;
  }
  else
  {
    forceDeadZone = _externalForcesDeadZones[r];
  }

  Fh = Utils<float>::deadZone((-_wRb[r]*_Fext[r]-Ft).norm(),0.0f,forceDeadZone)*vTooldir; 
    
  if(_useFTSensor[r])
  {
    _Fm[r] = Utils<float>::deadZone(Ft.norm(),0.0f,5.0f)*Ft.normalized(); 
    _Fm[r](0) *= 0.2f;
    _Fm[r](1) *= 0.2f; 
  }
  else
  {
    _Fm[r].setConstant(0.0f);
  }

  Eigen::Vector3f D;
  D << 200, 200, 200;

  float mass = 5.0f;
  _vHRef[r] += _dt*(-_wRbIK[r]*D.asDiagonal()*_wRbIK[r].transpose()*_vHRef[r]+ Fh)/mass;

  _vHd[r] = fulcrumEffectScaling(r, _vHRef[r]);
  _vHd[r] = Utils<float>::bound(_vHd[r], 0.1f);

  float pin, pout, pd;
  pin = 1000*Fh.dot(_vHd[r]);
  pd = 2.0f;

  _tankH[r] += _dt*(pin-(1.2f-_alphaH[r])*pd);
  _tankH[r] = Utils<float>::bound(_tankH[r],0,1.0f);

  _alphaH[r] = Utils<float>::smoothRise(_tankH[r],0.0,0.8f);

  if(_debug)
  {
    std::cerr << r << " Dir: " << vTooldir.transpose() << " tank: " << _tankH[r] << " alpha: " << _alphaH[r] <<  " vH: " << _vHRef[r].norm() << " Fh: "<<  Fh.norm() << std::endl;
  }
}


Eigen::Vector3f SurgicalTask::fulcrumEffectScaling(int r, Eigen::Vector3f vIn)
{
  Eigen::Matrix3f B;
  B.setIdentity();
  B(0,0) = _depthGain[r];
  B(1,1) = _depthGain[r];

  Eigen::Vector3f vOut;
  if(_controlStrategy[r] == PASSIVE_DS)
  {
    vOut = _wRb[r]*B*_wRb[r].transpose()*vIn; 
  }
  else
  {
    vOut = _wRbIK[r]*B*_wRbIK[r].transpose()*vIn;
  }
  return vOut;
}


void SurgicalTask::computeHapticFeedback(int r)
{
  _FdFoot[r].setConstant(0.0f);
  
  if(_controlPhase[r] == OPERATION)
  {

    Eigen::Vector3f vdEE, omegadEE;
    if(_controlStrategy[r] == PASSIVE_DS)
    {
      getExpectedDesiredEETwist(r, vdEE, omegadEE, _vdTool[r], _wRb[r]*_toolOffsetFromEE[r]);
    }
    else
    {
      getExpectedDesiredEETwist(r, vdEE, omegadEE, _vdTool[r],_wRbIK[r]*_toolOffsetFromEE[r]);
    }
    
    float toolCollisionVel = _nToolCollision[r].transpose()*(vdEE+omegadEE.cross(_toolCollisionOffset[r]));
    float eeCollisionVel = _nEECollision[r].transpose()*(vdEE+omegadEE.cross(-_eeSafetyCollisionRadius*_nEECollision[r]));


    if(_useRobot[LEFT] && _useRobot[RIGHT])
    {
      float safetyToolCollisionGain = Utils<float>::smoothFall(_dToolCollision[r],_toolSafetyCollisionDistance, _toolSafetyCollisionDistance+0.01f); 
      getExpectedDesiredEETwist(r, vdEE, omegadEE, _nToolCollision[r], _toolCollisionOffset[r]);
      _FdFoot[r] += _linearForceFeedbackMagnitude*safetyToolCollisionGain*(vdEE+omegadEE.cross(_wRbIK[r]*_toolOffsetFromEE[r])).normalized();


      float safetyEECollisionGain = Utils<float>::smoothFall(_dEECollision[r],_eeSafetyCollisionDistance, _eeSafetyCollisionDistance+0.01f); 
      getExpectedDesiredEETwist(r, vdEE, omegadEE, _nEECollision[r], -_eeSafetyCollisionRadius*_rEECollision[r].normalized());
      _FdFoot[r] += _linearForceFeedbackMagnitude*safetyEECollisionGain*(vdEE+omegadEE.cross(_wRbIK[r]*_toolOffsetFromEE[r])).normalized();

    }

    if(_tool[r] == CAMERA)
    {
      Eigen::Vector3f dir;
      Eigen::Vector3f currentOffset = _xIK[r]-_xd0[r];
      dir << 0.0f,0.0f,1.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(currentOffset(2)-_operationMinOffsetPVM[r](2),0, 0.01f)*dir; 

      dir << 0.0f,0.0f,-1.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(_operationMaxOffsetPVM[r](2)-currentOffset(2),0, 0.01f)*dir; 

      dir << 1.0f,0.0f,0.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(currentOffset(0)-_operationMinOffsetPVM[r](0),0, 0.01f)*dir; 

      
      dir << -1.0f,0.0f,0.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(_operationMaxOffsetPVM[r](0)-currentOffset(0),0, 0.01f)*dir; 

      dir << 0.0f,1.0f,0.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(currentOffset(1)-_operationMinOffsetPVM[r](1),0, 0.01f)*dir; 


      dir << 0.0f,-1.0f,0.0f;
      _FdFoot[r]+= _linearForceFeedbackMagnitude*Utils<float>::smoothFall(_operationMaxOffsetPVM[r](1)-currentOffset(1),0, 0.01f)*dir;  
    }


    if(_tool[r] == RETRACTOR && _inputAlignedWithOrigin[r]==false)
    {
      _FdFoot[r] += Utils<float>::bound(200.0f*(-_desiredOffsetPPM[r]),15.0f);   
    }


    _FdFoot[r] += _Fm[r];


    _taud[r] = _selfRotationTorqueFeedbackMagnitude*(Utils<float>::smoothFall(_currentJoints[r](6)+_trocarSpaceSelfRotationRange*M_PI/180.0f,0, 0.2f)
                    -Utils<float>::smoothFall(_trocarSpaceSelfRotationRange*M_PI/180.0f-_currentJoints[r](6),0, 0.2f));
    // _currentJoints[r](6)<-_trocarSpaceSelfRotationRange

    // if(_tool[r] == RETRACTOR && _colorMarkersStatus[2] == 0 && _toolsTracking == CAMERA_BASED)
    // {
    //   _FdFoot[r] += Utils<float>::bound(200.0f*(_xIK[r]-(_xd0[r]+_desiredOffsetPPM[r])),15.0f);   
    //   // _FdFoot[r] = _wRb[r]*_filteredWrench[r].segment(0,3);   
    // }
  }
}


void SurgicalTask::getExpectedDesiredEETwist(int r, Eigen::Vector3f &vdEE, Eigen::Vector3f &omegadEE, Eigen::Vector3f vdk, Eigen::Vector3f rk)
{
  Eigen::Matrix<float,6,6> A;

  if(_controlStrategy[r] == PASSIVE_DS)
  {
    A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_toolDir[r])*Eigen::Matrix3f::Identity();
    A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_toolDir[r])*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r]);    
  }
  else
  {
    A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_toolDirIK[r])*Eigen::Matrix3f::Identity();
    A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_toolDirIK[r])*Utils<float>::getSkewSymmetricMatrix((_trocarPosition[r]-_xEEIK[r]).dot(_toolDirIK[r])*_toolDirIK[r]);    
  }
  A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rk);
  Eigen::Matrix<float,6,1> x, b;
  b.setConstant(0.0f);
  b.segment(3,3) = vdk;

  x = A.fullPivHouseholderQr().solve(b);
  vdEE = x.segment(0,3);
  omegadEE = x.segment(3,3);
}