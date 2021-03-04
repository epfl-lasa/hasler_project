#include "SurgicalTask.h"


void SurgicalTask::robotControlStep(int r, int h)
{
  // Update trocar information
  updateTrocarInformation(r);

  // Select robot mode
  updateControlPhase(r);

  switch(_controlPhase[r])
  {
    case INSERTION:
    {
      insertionStep(r, h);
      break;
    }
    case OPERATION:
    {
      if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT || (_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS && !_clutching && ! _wait))
      {
        operationStep(r, h);
      }
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
    computeDesiredFootWrench(r, h);
  }
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


  if(_debug)
  {
  	std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-tool: " << _dRCMTool[r] << std::endl;
  	std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-trocar: " << (_trocarPosition[r]-_xRCM[r]).norm() <<std::endl;    
  }

  Eigen::Vector3f r21, e1, e2;
  if(r == LEFT)
  {
    _rEECollision[r] = _xEE[r]-_xEE[RIGHT];
    e1 = _wRb[r].col(2);
    e2 = _wRb[RIGHT].col(2);
  }
  else if (r == RIGHT)
  {
    _rEECollision[r] = _xEE[r]-_xEE[LEFT];
    e1 = _wRb[r].col(2);
    e2 = _wRb[LEFT].col(2);
  }

  r21 = _rEECollision[r];


  float l1 = 0.0f, l2 = 0.0f;
  float den = 1.0f-std::pow(e1.dot(e2),2.0f);


  if(den > FLT_EPSILON)
  {
    l1 = -(r21.dot(e1)-e1.dot(e2)*r21.dot(e2))/den;
    l1 = std::max(0.0f, std::min(l1, _toolOffsetFromEE[r]));
    l2 = (r21.dot(e2)-e1.dot(e2)*r21.dot(e1))/den;
    if(r == LEFT)
    {
      l2 = std::max(0.0f, std::min(l2, _toolOffsetFromEE[RIGHT]));
    }
    else
    {
      l2 = std::max(0.0f, std::min(l2, _toolOffsetFromEE[LEFT]));
    }

    _rToolCollision[r] = r21+l1*e1-l2*e2;
    _toolCollisionOffset[r] = l1;
  }
  else
  {
    _rToolCollision[r] = r21;
    _toolCollisionOffset[r] = _toolOffsetFromEE[r];
  }

  if(_debug)
  {
  	std::cerr << "[SurgicalTask]: " << r << ": Distance EE-Robot: " << _rEECollision[r].norm() << " " << _rEECollision[r].norm()-2*0.1f <<std::endl;
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

  if(_controlPhase[r] == INSERTION)
  {
    if(_insertionFinished[r])
    {
      _controlPhase[r] = OPERATION;
      _wRb0[r] = _wRb[r];
      _xd0[r] = _x[r];
      if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
      {
      	_clutching = true;
      	computeTrocarInput(_currentRobot,_dominantInputID);
      	_clutching = false;
      }
    }
  }
}


void SurgicalTask::insertionStep(int r, int h)
{

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": AUTOMATIC INSERTION" << std::endl;
  }


  if(_linearMapping[r] == POSITION_VELOCITY)
  {
    _vd[r] = _xd0[r]+_wRb0[r].col(2)*(-_insertionDistancePVM[r])-_x[r]; 
  }
  else
  {
    _vd[r] = _xd0[r]+_insertionOffsetPPM[r]-_x[r];
  }

  if(_vd[r].norm()<0.005)
  {
    _insertionFinished[r] = true;
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.05f);

//   if(_linearMapping[r]==POSITION_VELOCITY || _useSim)
//   {
//     _vd[r] = 0.1f*_trocarInput[h](2)*_rEETrocar[r].normalized(); 
//     if(_dRCMTool[r]> -0.005f && (_wRb[r].col(2)).dot(_vd[r])<0.0f)
//     {
//       _vd[r].setConstant(0.0f);
//     }
//   }
//   else
//   {
//     _vd[r].setConstant(0.0f);
//   }

//   _vd[r] = Utils<float>::bound(_vd[r],0.3f);

  if(_controlStrategy[r] == PASSIVE_DS)
  {
    _stiffness[r].setConstant(0.0f);
    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_wRb0[r].col(2)));

    Eigen::Vector3f axis;  
    float angleErrorToTrocarPosition;
    Utils<float>::quaternionToAxisAngle(qe, axis, angleErrorToTrocarPosition);

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

    // Compute final quaternion on plane
    _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

    _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);
  }
  else if (_controlStrategy[r] == JOINT_IMPEDANCE && _firstPublish[r])
  {
    _stiffness[r] = Eigen::Map<Eigen::Matrix<float, 7, 1> >(_jointImpedanceStiffnessGain.data());
  
    _selfRotationCommand[r] = 0.0f;

    _qpResult[r] = _qpSolverRCMCollision[r]->step(_ikJoints[r], _ikJoints[r], _trocarPosition[r], _toolOffsetFromEE[r], _vd[r],
                                             _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f,
                                             (_rEECollision[r].norm()-2.0f*_eeSafetyCollisionRadius)*_rEECollision[r].normalized(), _rToolCollision[r],
                                             _toolCollisionOffset[r]);
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

  // Compute desired tool velocity
  computeDesiredToolVelocity(r, h);

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": vd tool before: " << _vdTool[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 
  }

  // Scale the desired velocity components normal to the tool depending on the penetration depth
  float depthGain = std::min(std::max((_x[r]-_trocarPosition[r]).dot(_wRb[r].col(2)),0.0f)*3.0f/_toolOffsetFromEE[r],1.0f);

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": depth gain: " <<  depthGain << std::endl;
  }
  
  Eigen::Matrix3f L;
  L.setIdentity();
  L(0,0) = depthGain;
  L(1,1) = depthGain;
  _vdTool[r] = _wRb[r]*L*_wRb[r].transpose()*_vdTool[r];
  
  // Bound vd tool
  _vdTool[r] = Utils<float>::bound(_vdTool[r], _toolTipLinearVelocityLimit);

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": vd tool after: " << _vdTool[r].transpose() << " Self rotation: " << _selfRotationCommand[r] << std::endl; 
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

    
    _vd[r]+=2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_trocarPosition[r]-_xRCM[r]);

    _vd[r] = Utils<float>::bound(_vd[r],0.4f);


    _omegad[r] = Utils<float>::bound(_omegad[r],3.0f);

    _nullspaceWrench[r].setConstant(0.0f);

    Eigen::Vector4f qe;
    qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

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

    _qpResult[r] = _qpSolverRCMCollision[r]->step(_ikJoints[r], _ikJoints[r], _trocarPosition[r], _toolOffsetFromEE[r], _vdTool[r],
                                              _selfRotationCommand[r], _dt, _xRobotBaseOrigin[r], _wRRobotBasis[r], 1.0f,
                                              (_rEECollision[r].norm()-2.0f*_eeSafetyCollisionRadius)*_rEECollision[r].normalized(), _rToolCollision[r],
                                             _toolCollisionOffset[r], true, _xIK[r]-_xd0[r]);

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

  // Compute IK tip position
  Eigen::Matrix4f Hik;
  Hik = Utils<float>::getForwardKinematics(_ikJoints[r],_robotID);
  _xIK[r] = _xRobotBaseOrigin[r]+_wRRobotBasis[r]*Hik.block(0,3,3,1)+_toolOffsetFromEE[r]*_wRRobotBasis[r]*Hik.block(0,2,3,1);

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << " xIK: " << _xIK[r].transpose() << std::endl;
  }

  if(_linearMapping[r] == POSITION_VELOCITY)
  {
    Eigen::Vector3f gains;
    gains << _trocarSpaceVelocityGains[V_UP], _trocarSpaceVelocityGains[V_RIGHT], _trocarSpaceVelocityGains[V_INSERTION];

    _vdTool[r] = _wRb[r]*_eeCameraMapping*(gains.cwiseProduct(_trocarInput[h].segment(0,3)));   

    if(_useSafetyLimits)
    {
      // Eigen::Vector3f vs;
      // vs.setConstant(0.0f);

      // Eigen::Vector3f pyramidOffset, pyramidHeight, pyramidCenter;
      // pyramidOffset << _trocarSpacePyramidBaseOffset[r](X), _trocarSpacePyramidBaseOffset[r](Y), 0.0f; 
      // pyramidHeight << 0.0f, 0.0f, _trocarSpacePyramidBaseOffset[r](Z);
      // pyramidCenter = pyramidHeight+pyramidOffset;

      // float pyramidAngle = std::atan2(_trocarSpacePyramidBaseSize[r]/2,-pyramidHeight(Z));

      // float dotProduct = (pyramidHeight.normalized()).dot((pyramidOffset+pyramidHeight).normalized());
      // float theta = std::acos(std::max(std::min(dotProduct,1.0f),-1.0f));  

      // float xMin,xMax,yMin,yMax;
      // xMax = pyramidCenter(X)-pyramidHeight(Z)*tan(pyramidAngle);
      // xMax = std::max(0.0f,xMax);
      // xMin = pyramidCenter(X)-pyramidHeight(Z)*tan(-pyramidAngle);
      // xMin = std::min(0.0f,xMin);
      // yMax = pyramidCenter(Y)-pyramidHeight(Z)*tan(pyramidAngle);
      // yMax = std::max(0.0f,yMax);
      // yMin = pyramidCenter(Y)-pyramidHeight(Z)*tan(-pyramidAngle);
      // yMin = std::min(0.0f,yMin);

      Eigen::Vector3f currentOffset;
      currentOffset = _xIK[r]-_xd0[r];

      // if(currentOffset(Z)>pyramidHeight(Z))
      // {
      //   Eigen::Vector3f off;
      //   if(pyramidOffset.norm()< FLT_EPSILON)
      //   {
      //     off.setConstant(0.0f); 
      //   }
      //   else
      //   {
      //     off = std::tan(theta)*currentOffset(Z)*pyramidOffset.normalized();
      //   }
      //   xMax = -currentOffset(Z)*std::tan(pyramidAngle)-off(X);  
      //   xMin = -currentOffset(Z)*std::tan(-pyramidAngle)-off(X);  
      //   yMax = -currentOffset(Z)*std::tan(pyramidAngle)-off(Y);  
      //   yMin = -currentOffset(Z)*std::tan(-pyramidAngle)-off(Y);  
      // }

      Eigen::Vector3f vplane, vortho;
      vplane = Utils<float>::orthogonalProjector(_wRb[r].col(2))*_vdTool[r];
      vortho = _vdTool[r]-vplane;

      bool safetyCollison = false;
      if(currentOffset(0) > _operationMaxOffsetPVM[r](0) && _vdTool[r](0) > 0)
      {
        safetyCollison = true;
      }
      else if(currentOffset(0)< _operationMinOffsetPVM[r](0) && _vdTool[r](0) < 0)
      {
        safetyCollison = true;
      }
      if(currentOffset(1) > _operationMaxOffsetPVM[r](1) && _vdTool[r](1) > 0)
      {
        safetyCollison = true;
      }
      else if(currentOffset(1)< _operationMinOffsetPVM[r](1) && _vdTool[r](1) < 0)
      {
        safetyCollison = true;
      }
      if(currentOffset(2)< _operationMinOffsetPVM[r](2) && _vdTool[r](2) < 0.0f)
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

      if(_debug)
      {
        std::cerr << "[SurgicalTask]: " << r << " Safety collision: " << (int) safetyCollison << std::endl; 
        std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(0) << " " << _operationMinOffsetPVM[r](0) << " " << _operationMaxOffsetPVM[r](0) << std::endl;
        std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(1) << " " << _operationMinOffsetPVM[r](1) << " " << _operationMaxOffsetPVM[r](1) << std::endl;
        std::cerr << "[SurgicalTask]: " << r << " Offset: " << currentOffset(2) << " " << _operationMinOffsetPVM[r](2) << " " << _operationMaxOffsetPVM[r](2) << std::endl;                
      }
    }

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << " Current offset: " << (_xIK[r]-_xd0[r]).transpose();
    }

    _selfRotationCommand[r] = _trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[h](W_SELF_ROTATION);
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
      if(std::fabs(_trocarInput[h](EXTRA_DOF))>std::fabs(_taskAdaptationDeactivationThreshold) &&
         std::fabs(_oldTrocarInput[h](EXTRA_DOF))<std::fabs(_taskAdaptationDeactivationThreshold) &&
         _trocarInput[h](EXTRA_DOF)*_taskAdaptationDeactivationThreshold>0 && _useTaskAdaptation)
      {
        _useTaskAdaptation = false;
      }
      else if(std::fabs(_trocarInput[h](EXTRA_DOF))>std::fabs(_taskAdaptationActivationThreshold) &&
              std::fabs(_oldTrocarInput[h](EXTRA_DOF))<std::fabs(_taskAdaptationActivationThreshold) &&
              _trocarInput[h](EXTRA_DOF)*_taskAdaptationActivationThreshold>0 && !_useTaskAdaptation)
      {
        initializeBeliefs(r);
        _useTaskAdaptation = true;
      }
      // if(_useTaskAdaptation && _trocarInput[h](EXTRA_DOF)>0.6f)
      // {
      //   _useTaskAdaptation = false;
      // }
      // if(!_useTaskAdaptation && _trocarInput[h](EXTRA_DOF) <-0.6f)
      // {
      //   initializeBeliefs(r);
      //   _useTaskAdaptation = true;
      // }
      if(_useTaskAdaptation)
      {
        taskAdaptation(r, h);
        _vdTool[r] = gains(V_INSERTION)*_trocarInput[h](V_INSERTION)*_wRb[r].col(V_INSERTION)+_vda;
      }   
    }

    if(_dRCMTool[r]> _insertionDistancePVM[r] && (_wRb[r].col(2)).dot(_vdTool[r])<0.0f)
    {
      _vdTool[r].setConstant(0.0f);
    }

  }
  else if(_linearMapping[r]==POSITION_POSITION)
  {

    float xMin,xMax,yMin,yMax;
    xMin = -_trocarSpacePyramidBaseSize[r]/2.0;

    xMin = -_operationOffsetRangePPM[r](0)/2.0;
    xMax = _operationOffsetRangePPM[r](0)/2.0;
    yMin = -_operationOffsetRangePPM[r](1)/2.0;
    yMax = _operationOffsetRangePPM[r](1)/2.0;

    _desiredOffsetPPM[r](Z) = Utils<float>::bound(-_operationOffsetRangePPM[r](2)*_trocarInput[h](Z)+_toolClutchingOffset(Z),
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
    if((_desiredOffsetPPM[r]-currentOffset).norm()<0.02f)
    {
      _inputAlignedWithOrigin[r]=true;
    }

    if(_inputAlignedWithOrigin[r]==false)
    {
      _desiredOffsetPPM[r].setConstant(0.0f);
      _xd[r] = _xd0[r];
  
      if(_debug)
      {
        std::cerr << "[SurgicalTask]: " << r << "Input should be align with origin" << std::endl;
      }
    }
    
    float alpha = _trocarSpaceLinearDSFixedGain+_trocarSpaceLinearDSGaussianGain*std::exp(-(_desiredOffsetPPM[r]-currentOffset).squaredNorm()/(2.0f*std::pow(_trocarSpaceLinearDSGaussianWidth,2.0f)));  
    
    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": alpha: " << alpha << std::endl;
    }

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
      	_desiredAnglePPM[r] = Utils<float>::bound(_trocarInput[h](SELF_ROTATION)*_trocarSpaceSelfRotationRange+_toolClutchingOffset(SELF_ROTATION),
      		                                       -_trocarSpaceSelfRotationRange,_trocarSpaceSelfRotationRange);

        _selfRotationCommand[r] = _trocarSpaceSelfRotationGain*(_desiredAnglePPM[r]*M_PI/180.0f-_currentJoints[r](6));
        _selfRotationCommand[r]  = Utils<float>::bound(_selfRotationCommand[r] ,-_toolTipSelfAngularVelocityLimit, _toolTipSelfAngularVelocityLimit);        
      }
      else
      {
        _selfRotationCommand[r] =_trocarSpaceVelocityGains[W_SELF_ROTATION]*_trocarInput[h](W_SELF_ROTATION);
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
      // if(_controlPhase[r] == OPERATION && _inputAlignedWithOrigin[r]==false)
      // {
      //   _FdFoot[r] = Utils<float>::bound(200.0f*(_x[r]-(_xd0[r]+_desiredOffsetPPM[r])),15.0f)+2.0f*_wRb[r]*_filteredWrench[r].segment(0,3);   
      //   // _FdFoot[r] = _wRb[r]*_filteredWrench[r].segment(0,3);   
      // }
      // else if(_controlPhase[r] == OPERATION && _inputAlignedWithOrigin[r]==true)
      // {
      //   _FdFoot[r] = 2.0f*_wRb[r]*_filteredWrench[r].segment(0,3);
      // }
      break;
    }
    default:
    {
      _FdFoot[r].setConstant(0.0f);
      break;
    }
  }
}