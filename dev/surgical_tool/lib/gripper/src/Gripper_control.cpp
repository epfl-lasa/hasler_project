#include "Gripper.h"

void Gripper::positionControl() {
  if ((_gripper_controllerType == POSITION_CTRL)) {
    if (_gripper_state==GRIPPER_STATE_CONTROL)
    {
        posInterpolator();
    }
    else
    {
        _positionD_filtered = _positionD;
    }
      _pidPosition->compute();
      _pwmSpeedOut = _positionCtrlOut;
  }
}

//! 3
void Gripper::speedControl() {
  if ((_gripper_controllerType == SPEED_CTRL)) {
    _pidSpeed->compute();
    _pwmSpeedOut = _speedCtrlOut;
  }
}


//! 5
void Gripper::gotoPoint(float point)
{
  _positionD=point;
  positionControl();
}

//! 7
void Gripper::gotoPointGainsDefault()
{
  _gripper_kpPosition = GT_KP_POSITION;
  _gripper_kiPosition = GT_KI_POSITION;
  _gripper_kdPosition = GT_KD_POSITION;
}

void Gripper::speedPIDGainsDefault()
{

  _gripper_kpSpeed = GT_KP_SPEED;
  _gripper_kiSpeed = GT_KI_SPEED;
  _gripper_kdSpeed = GT_KD_SPEED;

}

void Gripper::posCtrlLimitsSet() {    
  _pidPosition->setOutputLimits(-SPEED_LIMIT_GRIPPER_DEFAULT,SPEED_LIMIT_GRIPPER_DEFAULT);
}


void Gripper::speedCtrlLimitsSet() {
  _pidSpeed->setOutputLimits(-SPEED_LIMIT_GRIPPER_DEFAULT,SPEED_LIMIT_GRIPPER_DEFAULT);
}

void Gripper::posInterpolator(){

     if (fabs(_positionD - _position) > 0.005) //! Only interpolate
                                                           //! if greater than 5
                                                           //! millimiters /
                                                           //! millidegrees
     {
       _positionD_filtered =
           _posDesiredFilter.update(_positionD);
       _positionD_filtered = clip(_positionD_filtered,
                                        -C_WS_LIMITS, C_WS_LIMITS);
     } 
     else 
     {
       _positionD_filtered = _positionD;
     }
   }
   

void Gripper::loadDefaultPIDGains()
{
  gotoPointGainsDefault();
  speedPIDGainsDefault();
  setPIDGains();
}

void Gripper::loadROSPIDGains()
{

    _gripper_kpPosition=_ros_kpPosition;
    _gripper_kiPosition = _ros_kiPosition;
    _gripper_kdPosition = _ros_kdPosition;
    _gripper_kpSpeed = _ros_kpSpeed;
    _gripper_kiSpeed = _ros_kiSpeed;
    _gripper_kdSpeed = _ros_kdSpeed;
  setPIDGains();
}

void Gripper::setPIDGains()
{

    _pidPosition->setTunings(_gripper_kpPosition,
                                  _gripper_kiPosition,
                                  _gripper_kdPosition);

    _pidSpeed->setTunings(_gripper_kpSpeed,
                               _gripper_kiSpeed,
                               _gripper_kdSpeed);
}