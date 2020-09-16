#include "Gripper.h"


//! 1
void Gripper::emergencyCallback()
{
  me->_flagEmergencyCalled=true;
}

//! 2
void Gripper::releaseGripper()
{
    positionCtrlClear();
    _speedFilter.reset();
}
