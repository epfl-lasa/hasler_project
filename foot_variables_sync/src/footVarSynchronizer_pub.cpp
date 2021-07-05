#include "footVarSynchronizer.h"


void footVarSynchronizer::processAllPublishers()
{
	_msgTotalDesiredFootInput.ros_effort.fill(0.0f);
	_msgTotalDesiredFootInput.ros_speed.fill(0.0f);
	_msgTotalDesiredFootInput.ros_position.fill(0.0f);
	_msgTotalDesiredFootInput.ros_kp.fill(0.0f);
	_msgTotalDesiredFootInput.ros_ki.fill(0.0f);
	_msgTotalDesiredFootInput.ros_kd.fill(0.0f);
	_msgTotalDesiredFootInput.ros_filterAxisForce.fill(1.0f);
	//int nonZeroFilterAxisForce = 0; 

	for (size_t j=0; j<NB_PLATFORM_AXIS; j++)
	{
		for (size_t i=0; i< (size_t) _nbDesiredFootInputPublishers; i++)
		{		
			if (_flagDesiredFootInputsRead[i] && _subDesiredFootInput[i].getNumPublishers()>0)
			{	
				
				if (_platform_controllerType==POSITION_CTRL)
				{
					_msgTotalDesiredFootInput.ros_position[j] += _msgDesiredFootInput[i].ros_position[j];
				}else if (_platform_controllerType==SPEED_CTRL)
				{
					_msgTotalDesiredFootInput.ros_speed[j] += _msgDesiredFootInput[i].ros_speed[j];
				}
				if (_platform_machineState==TELEOPERATION)
				{
					if (fabs(_msgDesiredFootInput[i].ros_kp[j])>FLT_EPSILON
						|| fabs(_msgDesiredFootInput[i].ros_ki[j])>FLT_EPSILON ||
						fabs(_msgDesiredFootInput[i].ros_kd[j])>FLT_EPSILON)
						{
							//_flagPIDGainsByInput=true;
							_msgTotalDesiredFootInput.ros_kp[j] += _msgDesiredFootInput[i].ros_kp[j];			
							_msgTotalDesiredFootInput.ros_ki[j] += _msgDesiredFootInput[i].ros_ki[j];			
							_msgTotalDesiredFootInput.ros_kd[j] += _msgDesiredFootInput[i].ros_kd[j];			
						}
				}
				_msgTotalDesiredFootInput.ros_effort[j] += _msgDesiredFootInput[i].ros_effort[j];
				//nonZeroFilterAxisForce += _msgDesiredFootInput[i].ros_filterAxisForce[j] > FLT_EPSILON ? 1 : 0;
				_msgTotalDesiredFootInput.ros_filterAxisForce[j] *= _msgDesiredFootInput[i].ros_filterAxisForce[j];
			}
		}
		
		//_msgTotalDesiredFootInput.ros_filterAxisForce[j] /= nonZeroFilterAxisForce > 0 ? (float) nonZeroFilterAxisForce : 1.0 ;	
		//msgTotalDesiredFootInput.ros_filterAxisForce[j] = Utils_math<float::>nonZeroFilterAxisForce > 0 ? (float) nonZeroFilterAxisForce : 1.0
			
	}
}


void footVarSynchronizer::publishFootInput(bool* flagVariableOnly_) {
  //! Keep send the same valuest that the platform is broadcasting
  processAllPublishers();	
 // _mutex.lock();
 if (!_stop)
 {
  for (int k = 0; k < NB_PLATFORM_AXIS; k++) {
    _msgFootInput.ros_position[rosAxis[k]] = _ros_position[k] + _msgTotalDesiredFootInput.ros_position[rosAxis[k]];
    _msgFootInput.ros_speed[rosAxis[k]] = _ros_speed[k] + _msgTotalDesiredFootInput.ros_speed[rosAxis[k]] ;
	if (_flagControlZeroEffort) {
		_msgFootInput.ros_effort[rosAxis[k]] =  0.0f;
	}else{
		_msgFootInput.ros_effort[rosAxis[k]] = _ros_effort[k] + ( _flagHumanOnPlatform ?  Utils_math<float>::bound(_msgTotalDesiredFootInput.ros_effort[rosAxis[k]],-effortLims[rosAxis[k]], effortLims[rosAxis[k]]) : 0.0f);
	}
    float divFilterAxisForce = (_ros_filterAxisFS[k] > FLT_EPSILON ? 1.0 : 0.0 ) + (_msgTotalDesiredFootInput.ros_filterAxisForce[rosAxis[k]] > FLT_EPSILON ? 1.0 : 0.0);
	_msgFootInput.ros_filterAxisForce[rosAxis[k]] = _ros_filterAxisFS[k] * _msgTotalDesiredFootInput.ros_filterAxisForce[rosAxis[k]] ;
	// _msgFootInput.ros_filterAxisForce[rosAxis[k]] /= divFilterAxisForce > FLT_EPSILON ? divFilterAxisForce : 1.0;
	_msgFootInput.ros_filterAxisForce[rosAxis[k]] = _msgFootInput.ros_filterAxisForce[rosAxis[k]] > 1.0 ? 1.0 : _msgFootInput.ros_filterAxisForce[rosAxis[k]];
	
		if (_platform_controllerType==POSITION_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_ros_posP[k] + _msgTotalDesiredFootInput.ros_kp[rosAxis[k]],0.0,_ros_posP_Max[rosAxis[k]]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_ros_posI[k] + _msgTotalDesiredFootInput.ros_ki[rosAxis[k]],0.0,_ros_posI_Max[rosAxis[k]]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_ros_posD[k] + _msgTotalDesiredFootInput.ros_kd[rosAxis[k]],0.0,_ros_posD_Max[rosAxis[k]]);
		} else if (_platform_controllerType==SPEED_CTRL)
		{
			_msgFootInput.ros_kp[rosAxis[k]] = Utils_math<float>::bound(_ros_speedP[k] + _msgTotalDesiredFootInput.ros_kp[rosAxis[k]],0.0,_ros_speedP_Max[rosAxis[k]]);
			_msgFootInput.ros_ki[rosAxis[k]] = Utils_math<float>::bound(_ros_speedI[k] + _msgTotalDesiredFootInput.ros_ki[rosAxis[k]],0.0,_ros_speedI_Max[rosAxis[k]]);
			_msgFootInput.ros_kd[rosAxis[k]] = Utils_math<float>::bound(_ros_speedD[k] + _msgTotalDesiredFootInput.ros_kd[rosAxis[k]],0.0,_ros_speedD_Max[rosAxis[k]]);
		}

  }
  if(_subTorquesModified.getNumPublishers()>0)
  {
	  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
	  {
		  _msgFootInput.ros_effortM[i] = _ros_effortM(rosAxis[i]);
	  }
  }else
  {
	_ros_effortM.setZero();
	_msgFootInput.ros_effortM.fill(0.0f);
  }



 }
 else
 {
	 _msgFootInput.ros_effort.fill(0.0f);
	 _msgFootInput.ros_filterAxisForce.fill(0.0f);
	 _msgFootInput.ros_effortM.fill(0.0f);
	 _msgFootInput.ros_kp.fill(0.0f);
	 _msgFootInput.ros_ki.fill(0.0f);
	 _msgFootInput.ros_kd.fill(0.0f);
	 for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
	 {
		 _msgFootInput.ros_position[i] = _platform_position[i];
	 }
	 _msgFootInput.ros_speed.fill(0.0f);
	 _msgFootInput.ros_effort.fill(0.0f);
 }

  _pubFootInput.publish(_msgFootInput);
  if (flagVariableOnly_!=NULL)
  {
  	*flagVariableOnly_ = true;
  }
  //_mutex.unlock();
}