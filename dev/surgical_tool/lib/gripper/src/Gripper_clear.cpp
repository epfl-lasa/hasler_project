#include "Gripper.h"



void Gripper::positionReset()
{
  if (_switchesState == 1)
  {
    _positionOffset = HOMING_OFFSET;
    _spi->lock();
      _encoder->QEC_clear(_spi);
    _spi->unlock();
  }
}


void Gripper::limitSwitchesClear()
{
    _switchesState = 0;
}


void Gripper::clearLastState()
{
  switch(_gripper_state)
    {
      case(HOMING):
      {
        _enterStateOnceFlag[HOMING]=false;   
        limitSwitchesClear();       
        speedCtrlClear();
        break;
      }

      case(GRIPPER_STATE_CONTROL):
      {
        _enterStateOnceFlag[GRIPPER_STATE_CONTROL]=false;
        positionCtrlClear();
        speedCtrlClear();
        break;
      }


      case(EMERGENCY):{
        _enterStateOnceFlag[EMERGENCY]=false;
        break;
      }
      case(STANDBY): {_enterStateOnceFlag[STANDBY]=false;
        //sprintf(_logMsg, "%s : LEAVING STANDBY STATE", Platform_Names[PLATFORM_ID]);
        //_nh.loginfo(_logMsg);
        break;
      }
      case(RESET_UC):{break;}
    }
}



void Gripper::resetControllers(Controller controllerType)
{
  switch (controllerType) {
    
    case(POSITION_CTRL):
      {
      _pidPosition->reset();  _posDesiredFilter.reset();   
        break;
      }
    case (SPEED_CTRL): {
        _pidSpeed->reset();
        break;
      }
  }
}

void Gripper::positionCtrlClear() {
    _positionD=_position;  
    _positionCtrlOut=0.0f;
}



void Gripper::speedCtrlClear()
{
    _speedD=_speed;
    _speedCtrlOut=0.0f; 
}
  