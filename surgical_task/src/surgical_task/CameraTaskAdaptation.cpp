#include "SurgicalTask.h"

void SurgicalTask::initializeBeliefs(int r)
{
  Eigen::VectorXf error;

  Eigen::MatrixXf::Index indexMin;

  _beliefsC.setConstant(0.0f);

  if(_useSim || (!_useSim && _toolsTracking == OPTITRACK_BASED))
  {
  	error.resize(_nbTasks);
  	Eigen::MatrixXf xAk;
  	xAk.resize(_nbTasks,3);

  	for(int k = 0; k < _nbTasks; k++)
  	{
  		if(_useSim && !(_useRobot[RIGHT] && k == _nbTasks-1))
  		{
				xAk.row(k) = _pillarsPosition.row(k);
  		}
  		else if(!_useSim && !(_useRobot[RIGHT] && k == _nbTasks-1))
  		{
				xAk.row(k) = _humanToolPosition[k].transpose();
  		}
  		else
  		{
				xAk.row(k) = _x[RIGHT];
  		}

			error(k) = (Utils<float>::orthogonalProjector(_wRb[r].col(2))*(xAk.row(k).transpose()-_x[r])).norm();
  	}

	  float minValue = error.array().minCoeff(&indexMin);
	  _beliefsC.setConstant(0.0f);
	  _beliefsC(indexMin)= 1.0f;

	  if(_controlStrategy[r] == PASSIVE_DS)
	  {
    	_d = (_x[r]-xAk.row(indexMin).transpose()).norm();
	  }
	  else if(_controlStrategy[r] == JOINT_IMPEDANCE)
	  {
	    _d = (_xIK[r]-xAk.row(indexMin).transpose()).norm();      
	  }
  }
  else if(!_useSim && _toolsTracking == CAMERA_BASED)
  {
		std::vector<int> tempID;
	  for(int k = 0; k < _nbTasks; k++)
	  {
	    if(_colorMarkersStatus[k])
	    {
	      tempID.push_back(k);
	    }
	  }

	  if(tempID.size()>0)
	  {
	    error.resize(tempID.size());

	    for(int k = 0; k < tempID.size(); k++)
	    {
	      error(k) = _colorMarkersPosition.row(tempID[k]).norm();
	    }
      
      float minValue = error.array().minCoeff(&indexMin);
  		_beliefsC(tempID[indexMin])= 1.0f;
    }
    else
    {
	    _beliefsC(0) = 1.0f;
    }
  }
  else
  {
  	_beliefsC(0) = 1.0f;
  }
}


void SurgicalTask::taskAdaptation(int r, int h)
{
  ////////////////////////////////////
  // Compute human desired velocity //
  ////////////////////////////////////
  Eigen::Vector3f vH;

  vH = _wRb[r].col(V_UP)*_trocarInput[h](V_UP)+_wRb[r].col(V_RIGHT)*_trocarInput[h](V_RIGHT);
  
  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": Human input: " << vH.transpose() << std::endl; 
  }

  /////////////////////
  // Task adaptation //
  /////////////////////

  // Initialize desired velocity to zero
  _vda.setConstant(0.0f);


  Eigen::MatrixXf vdk, errork;
  vdk.resize(_nbTasks,3);
  errork.resize(_nbTasks,3);
	  
  Eigen::MatrixXf::Index indexMax;

  if(_useSim || (!_useSim && _toolsTracking == OPTITRACK_BASED))
  {
	  Eigen::MatrixXf xAk;
	  xAk.resize(_nbTasks,3);

  	// Set attractor for all tasks
	 	for(int k = 0; k < _nbTasks; k++)
  	{
  		if(_useSim && !(_useRobot[RIGHT] && k == _nbTasks-1))
  		{
				xAk.row(k) = _pillarsPosition.row(k);
  		}
  		else if(!_useSim && !(_useRobot[RIGHT] && k == _nbTasks-1))
  		{
				xAk.row(k) = _humanToolPosition[k].transpose();
  		}
  		else
  		{
				xAk.row(k) = _x[RIGHT];
  		}
  	}

	  // Update offset distance from tool tip to attractor with max belief
	  _beliefsC.array().maxCoeff(&indexMax);
	  if(std::fabs(1.0f-_beliefsC[indexMax])<FLT_EPSILON && std::fabs(_trocarInput[h](V_INSERTION))>FLT_EPSILON)
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
	  
    if(_debug)
    {
  	 std::cerr << "[SurgicalTask]: " << r << " d: " << _d << " " << std::fabs(1.0f-_beliefsC[indexMax]) << std::endl; 
    }
  	
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

      if(_debug)
      {
        std::cerr << "[SurgicalTask]: " << r << " Target " << k << " alpha: " << alpha << std::endl;
      }

	    // Compute desired task adapted velocity
	    _vda+=_beliefsC(k)*vdk.row(k).transpose();
	  }
  }
  else if(!_useSim && _toolsTracking == CAMERA_BASED)
  {
	  for(int k = 0; k < _nbTasks; k++)
	  {
	    float alpha = 0.05f; 

	    errork.row(k) = (_wRb[r]*_colorMarkersPosition.row(k).transpose()).transpose();
	    vdk.row(k) = alpha*errork.row(k);
      
      if(_debug)
      {
        std::cerr << "[SurgicalTask]: " << r << " Target " << k << " alpha: " << alpha << std::endl;
      }

	    // Compute desired task adapted velocity
	    _vda+=_beliefsC(k)*vdk.row(k).transpose();
	  }
  }
  else
  {
  	_vda.setConstant(0.0f);
  	return;
  }


  float a, b, c;

  for(int k = 0; k < _nbTasks; k++)
  {
    // Belief update based on human input
    // a = adaptationRate*(vH.dot(temp.normalized());
    // a = 5.0f*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(0.03,2.0f))))*vH.dot(vdk.row(k).normalized());
		if(_useSim || (!_useSim && _toolsTracking == OPTITRACK_BASED))
		{
	    a = _taskAdaptationAlignmentGain*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(_taskAdaptationGaussianWidth,2.0f))))*vH.dot(vdk.row(k).normalized());
    	b = _taskAdaptationConvergenceGain*(_beliefsC(k)-0.5f);
    	c = _taskAdaptationProximityGain*std::exp(-_taskAdaptationExponentialGain*(vdk.row(k)).norm()*vH.norm());
		}
		else
		{
	    a = _taskAdaptationAlignmentGain*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(_taskAdaptationGaussianWidth,2.0f))))*vH.dot(errork.row(k));
  	  b = _taskAdaptationConvergenceGain*(_beliefsC(k)-0.5f);
    	c = _taskAdaptationProximityGain*std::exp(-_taskAdaptationExponentialGain*(errork.row(k)).norm()*vH.norm());
		}

    _dbeliefsC(k) = _taskAdaptationOverallGain*(a+b+c);

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": Dbeliefs target" << k << ": " << a << " " << b <<  " " << c << " " << a+b+c << " " << errork.row(k).norm() << std::endl;
    }
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

    if(_debug)
    {
      std::cerr << "[SurgicalTask]: " << r << ": Before Dbeliefs: " << _dbeliefsC.transpose() << std::endl;
    }

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

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": After Dbeliefs: " << _dbeliefsC.transpose() << std::endl;
    std::cerr << _dbeliefsC.sum() << std::endl;
  }


  _beliefsC+=_dt*_dbeliefsC;
  for(int k = 0; k < _nbTasks; k++)
  {
    _beliefsC(k) = Utils<float>::bound(_beliefsC(k),0.0f,1.0f);
  }

  _beliefsC /= _beliefsC.sum();

  if(_debug)
  {
    std::cerr << "[SurgicalTask]: " << r << ": Beliefs: " << _beliefsC.transpose() << std::endl;
  }

  _vda.setConstant(0.0f);
  for(int k = 0; k < _nbTasks; k++)
  {
    _vda += _beliefsC(k)*vdk.row(k);
  }

  // _vda = Utils<float>::bound(_fx[r],0.3f);
}



// void SurgicalTask::taskAdaptation(int r, int h)
// {
//   ////////////////////////////////////
//   // Compute human desired velocity //
//   ////////////////////////////////////
//   Eigen::Vector3f vH;

//   vH = _wRb[r].col(V_UP)*_trocarInput[h](V_UP)+_wRb[r].col(V_RIGHT)*_trocarInput[h](V_RIGHT);
  
//   std::cerr << "[SurgicalTask]: " << r << ": Human input: " << vH.transpose() << std::endl; 

//   /////////////////////
//   // Task adaptation //
//   /////////////////////

//   // Initialize desired velocity to zero
//   _vda.setConstant(0.0f);

//   Eigen::MatrixXf vdk, errork;
//   vdk.resize(_nbTasks,3);
//   errork.resize(_nbTasks,3);


//   // Update task velocities
//   for(int k = 0; k < _nbTasks; k++)
//   {
    
//     float alpha = 0.05f; 

//     errork.row(k) = (_wRb[LEFT]*_humanToolPosition[k]).transpose();
//     vdk.row(k) = alpha*errork.row(k);
//     std::cerr << "[SurgicalTask]: " << r << " Target " << k << " alpha: " << alpha << std::endl;

//     // Compute desired task adapted velocity
//     _vda+=_beliefsC(k)*vdk.row(k).transpose();
//   }

//   float a, b, c;

//   for(int k = 0; k < _nbTasks; k++)
//   {
//     // Belief update based on human input
//     // a = adaptationRate*(vH.dot(temp.normalized());
//     // a = 5.0f*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(0.03,2.0f))))*vH.dot(vdk.row(k).normalized());
//     a = _taskAdaptationAlignmentGain*(1-std::exp(-errork.row(k).squaredNorm()/(2.0f*std::pow(_taskAdaptationGaussianWidth,2.0f))))*vH.dot(errork.row(k));
//     // a = (1.0f-_beliefsC(k))*vH.dot(vdk.row(k).normalized());

//     // Make belief converging to 0 or 1
//     // b = 50.0f*((_beliefsC(k)-0.5f)*vdk.row(k).squaredNorm());
//     b = _taskAdaptationConvergenceGain*(_beliefsC(k)-0.5f);
//     // b = _beliefsC(k)-0.5f;

//     // Belief update to select the closest ine if has same directions
//     c = _taskAdaptationProximityGain*std::exp(-_taskAdaptationExponentialGain*(errork.row(k)).norm()*vH.norm());
//     // c = 0.0f;

//     _dbeliefsC(k) = _taskAdaptationOverallGain*(a+b+c);

//     std::cerr << "[SurgicalTask]: " << r << ": Dbeliefs target" << k << ": " << a << " " << b <<  " " << c << " " << a+b+c << " " << errork.row(k).norm() << std::endl;
//   }

//   // std::cerr << r << ": a: " << _dbeliefsC.transpose() << std::endl;

//   Eigen::MatrixXf::Index indexMax;

//   float dbmax = _dbeliefsC.array().maxCoeff(&indexMax);

//   if(std::fabs(1.0f-_beliefsC(indexMax))< FLT_EPSILON && std::fabs(_beliefsC.sum()-1)<FLT_EPSILON)
//   {
//     _dbeliefsC.setConstant(0.0f);
//   }
//   else
//   {
//     Eigen::VectorXf temp;
//     temp.resize(_nbTasks-1);
//     temp.setConstant(0.0f);
//     int m = 0;
//     for(int k = 0; k < _nbTasks; k++)
//     {
//       if(k!=indexMax)
//       {
//         temp(m) = _dbeliefsC(k);
//         m++;
//       }
//     }
//     float db2max = temp.array().maxCoeff();

//     float z = (dbmax+db2max)/2.0f;
//     _dbeliefsC.array() -= z;
//     std::cerr << "[SurgicalTask]: " << r << ": Before Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

//     float S = 0.0f;
//     for(int k = 0; k < _nbTasks; k++)
//     {
//       // if(fabs(_beliefsC(k))>FLT_EPSILON || _dbeliefsC(k) > 0)
//       {
//         S += _dbeliefsC(k);
//       }
//     }
//     _dbeliefsC(indexMax)-=S;

//   }
//   std::cerr << "[SurgicalTask]: " << r << ": After Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

//   std::cerr << _dbeliefsC.sum() << std::endl;

//   _beliefsC+=_dt*_dbeliefsC;
//   for(int k = 0; k < _nbTasks; k++)
//   {
//     _beliefsC(k) = Utils<float>::bound(_beliefsC(k),0.0f,1.0f);
//   }

//   _beliefsC /= _beliefsC.sum();

//   std::cerr << "[SurgicalTask]: " << r << ": Beliefs: " << _beliefsC.transpose() << std::endl;

//   _vda.setConstant(0.0f);
//   for(int k = 0; k < _nbTasks; k++)
//   {
//     _vda += _beliefsC(k)*vdk.row(k);
//   }

//   // _vda = Utils<float>::bound(_fx[r],0.3f);
// }