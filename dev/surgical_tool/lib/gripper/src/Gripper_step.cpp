#include "Gripper.h"
#include <string>


void Gripper::step()
{
  //_gripperMutex.lock();
 // memset(_logMsg, 0, sizeof(_logMsg)); //! Flush the char

  if ((_ros_state == RESET_UC) || ((_gripper_state == RESET_UC)) || (_recoveringFromError))
  {
    //sprintf(_logMsg, "%s : ABOUT TO RESTART THE PLATFORM CONTROLLER", Gripper_Names[PLATFORM_ID]);
    //_nh.loginfo(_logMsg);
    NVIC_SystemReset();
    _stop = true;
    ThisThread::sleep_for(5000);
    return;
  }

  getMotion(); //! SPI
  
  if (_flagEmergencyCalled)
  {
    _gripper_state = EMERGENCY;
    _recoveringFromError=true;
  }

  switch (_gripper_state)
  {

  case STANDBY:{ 
        if (!_enterStateOnceFlag[STANDBY]){
          // TODO
          //sprintf(_logMsg, "%s : MOVING TO STATE STANDBY", Gripper_Names[PLATFORM_ID]);
          //_nh.loginfo(_logMsg);
          _enableMotors->write(0);
          _enterStateOnceFlag[STANDBY]=true;
          //sprintf(_logMsg, "Timestep: %f milliseconds", ((float)_timestep * 1e-3));
          //_nh.loginfo(_logMsg);
        }
//        totalEffortDClear();
        break;
        }    //Do nothing

    case HOMING:
        {
          // Init
          if(!_enterStateOnceFlag[HOMING])
          {           
            _gripper_controllerType = SPEED_CTRL;
            _pidSpeed->reset();
            loadDefaultPIDGains();
            speedCtrlLimitsSet();
            _enableMotors->write(1);
            limitSwitchesClear();
            _enterStateOnceFlag[HOMING]=true;

            _speedD = SPEED_D_HOMING;         // m/s
          }

          speedControl();

          // Definition of the transition rule to the next state
          if ((_switchesState == 1))
          {
            if(!_tic){
              _toc = _innerTimer.read_us();
              _tic=true;
              }
    
            //  After 1.5 second move to next state       
          
            if ((_innerTimer.read_us() - _toc) > 1500000)
            {
                
                positionReset(); // has to be before ClearlastState
                clearLastState(); 
                _tic=false;
                _toc=false;
                _gripper_state = STANDBY;    
            }
          }

          break;
          }



    case GRIPPER_STATE_CONTROL: //There is a bug here, please fix
    {
          // Init State
        
        if (!_enterStateOnceFlag[GRIPPER_STATE_CONTROL])
        {
          _gripper_controllerType=POSITION_CTRL;
          _pidPosition->reset();  _posDesiredFilter.reset();   
          _pidSpeed->reset();

          posCtrlLimitsSet(); // for constrains
          speedCtrlLimitsSet(); // for constrains
          _enterStateOnceFlag[GRIPPER_STATE_CONTROL] = true;
          _enableMotors->write(1);
        }

        // Main state

          if (flagPositionInControl()) {
            if (_flagInputReceived[MSG_POSITION]) {
                _positionD=_ros_position;
                _flagInputReceived[MSG_POSITION] = false;
            }
          

                gotoPoint(_positionD);
              
            }
            
          else if (flagSpeedInControl())
          {
            if (_flagInputReceived[MSG_SPEED]) {
                _speedD = _ros_speed;
              }
            _flagInputReceived[MSG_SPEED] = false;
          } 
            //! Add speed controller here!  
          

          break;
    }
        
    case EMERGENCY:
    {
          _enableMotors->write(0);
          if(!_enterStateOnceFlag[EMERGENCY]){
          _pidPosition->reset();  _posDesiredFilter.reset();   
          _pidSpeed->reset();
          _enterStateOnceFlag[EMERGENCY]=true;
          }
            releaseGripper();
            break;
    }
        case RESET_UC:
        {
          //defined upstairs
          break;
        }
  }
  
  setSpeed(_pwmSpeedOut);
  //readActualEffort();             //! Using the ESCON 50/5 Analog Output
  
  //! Update the gripper with ROS variables
  updateGripperFromRos();

  _timestep = float(_innerTimer.read_us() - _timestamp);
  _timestamp=_innerTimer.read_us();
  //_gripperMutex.unlock();
}

bool Gripper::flagPositionInControl() {
  return (_gripper_controllerType == POSITION_CTRL);
}

bool Gripper::flagSpeedInControl() {
  return (_gripper_controllerType == SPEED_CTRL);
}