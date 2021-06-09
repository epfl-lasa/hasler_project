#include "footVarSynchronizer.h"



void footVarSynchronizer::updateConfigAfterParamsChanged()
	{
		_flagUpdateConfig = false;

	

		if (_flagPositionOnlyPublished && _flagControlThisPosition)
		{
			_config.send_this_position = false;
			_flagControlThisPosition = false;
			_flagUpdateConfig = true;
		}

		if (_flagEffortOnlyPublished && _flagControlZeroEffort) {
			_config.send_zero_effort = false;
			_flagControlZeroEffort = false;
			_flagUpdateConfig = true;
		}

                if (_flagUpdateConfig)
		{
			_mutex.lock();
			_dynRecServer.updateConfig(_config);
			_mutex.unlock();
			_flagUpdateConfig = false;
		}

	}

void footVarSynchronizer::updateConfigAfterPlatformChanged()
	{
		_flagUpdateConfig = true;

		if (_flagUpdateConfig)
		{
			_mutex.lock();
			_dynRecServer.updateConfig(_config);
			_mutex.unlock();
			_flagUpdateConfig = false;
		}
	}


void footVarSynchronizer::resetDesiredPositionToCurrent(int axis_)
{
  if (axis_==-1){
    for (int k=0; k<NB_AXIS; k++ )
    {
      resetDesiredPositionToCurrent(k);
    }
  }

  else{
    switch(axis_){
        case(X): {_config.desired_Position_X=_platform_position[X];}
        case(Y): {_config.desired_Position_Y=_platform_position[Y];}
        case(PITCH): {_config.desired_Position_PITCH=_platform_position[PITCH];}
        case(ROLL): {_config.desired_Position_ROLL=_platform_position[ROLL];}
        case(YAW): {_config.desired_Position_YAW=_platform_position[YAW];}
      }
  }
}
