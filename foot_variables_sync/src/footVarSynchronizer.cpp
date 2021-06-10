#include "footVarSynchronizer.h"


#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes

char const *Platform_Names[]{"none", "right", "left"};

void footVarSynchronizer::stopNode(int sig)
{
    me->_stop= true;
	me->publishFootInput(NULL);
}

void footVarSynchronizer::run()
{
  while (!_stop) 
  {	
	if (_flagPlatformOutCommStarted && _subFootOutput.getNumPublishers()>0)
	{
		if ((_platform_id!=(uint8_t) _platform_name)&&(_platform_id!=UNKNOWN))
		{
			ROS_ERROR("[%s footVarSync]: This node for variables synchronization is acting on the wrong platform",Platform_Names[_platform_name]);
			break;
		}
		else
		{
			if (!_flagInitialConfig)
			{
				controlGainsDefault(-1);
				_config.machine_state = (uint8_t)_platform_machineState;
				_ros_newState = _config.machine_state;
				_config.controller_type = (uint8_t)_platform_controllerType;
				_ros_controllerType = _config.controller_type;
				_config.use_default_gains=true;
				_config.send_pid_gains=false;
				_config.load_param_pid_gains=false;
				_ros_defaultControl=true;
				_ros_position.setZero();
				_config.controlled_axis=-1;
				_dynRecServer.setConfigDefault(_config);
				_mutex.lock();
				_dynRecServer.updateConfig(_config);
				_mutex.unlock();
				_configPrev = _config;
				_msgFootOutputPrev=_msgFootOutput;
				_flagInitialConfig=true;
				ROS_INFO("[%s footVarSync]: Updating default parameters from the platform in the rqt_reconfig...",Platform_Names[_platform_name]);
				ros::spinOnce();
				
			} else if(_subFootOutput.getNumPublishers()>0)
			{	
				if (_flagOutputMessageReceived)
				{
					processFootOutput();
					changedPlatformCheck();
					_flagPlatformActionsTaken= false;
					requestDoActionsPlatform();
					if (_flagPlatformActionsTaken) {
						updateConfigAfterPlatformChanged();
						}
					_flagOutputMessageReceived = false;
				}

				checkWhichPIDGainsToUse();
				if (!_ros_defaultControl && _flagLoadPIDGains)
				{ 
					controlGainsDefault(-1);
					_flagUpdateConfig = true;
				}
				if (_flagWasDynReconfCalled)
				{
					updateInternalVariables();
					changeParamCheck();
					_flagParamsActionsTaken = false;
					requestDoActionsParams();
					if (_flagParamsActionsTaken)
						{ updateConfigAfterParamsChanged();}
					_flagWasDynReconfCalled = false;
				}
								

				if (_flagParamsActionsTaken)
				{
					_configPrev = _config;
					_flagParamsActionsTaken= false;
				}
				if (_flagPlatformActionsTaken)
				{
					_msgFootOutputPrev = _msgFootOutput;
					_flagPlatformActionsTaken= false;
				}


				if (_flagForceModifiedConnected)
				{ 
					correctForceForLegCompensation();

					if (abs(_ros_forceModified.segment(0,3).norm()) > HUMAN_ON_PLATFORM_THRESHOLD) {
						if(!_flagHumanOnPlatform)
						{ 
							ROS_INFO("[%s footVarSync]: Probably there is a human on the platform",Platform_Names[_platform_name]); 
							_flagHumanOnPlatform=true;
						}
					}
					else
					{
						if(_flagHumanOnPlatform)
						{ 
							ROS_INFO("[%s footVarSync]: Probably there is NOT a human on the platform",Platform_Names[_platform_name]); 
							_flagHumanOnPlatform=false;
						}
					}
					}
					
					_ros_effort.setZero();

					if (_flagHumanOnPlatform && _flagCompensateLeg && _subLegGravCompTorques.getNumPublishers()>0)
					{	
						_ros_effort+= _leg_grav_comp_effort;
					}
			}
			

		}
	}
	else
	{
		_flagInitialConfig=false;
	}
          publishFootInput(&_flagPositionOnlyPublished);
		  
        ros::spinOnce();
	_loopRate.sleep();
  }
  ROS_INFO("[%s footVarSync]: Parameters setting and variables synchronization stoped",Platform_Names[_platform_name]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}