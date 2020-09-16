#include "Gripper.h"

Gripper::~Gripper()
{
  _innerTimer.~Timer();

    _pidPosition->~PID();
    delete (_pidPosition);
    _pidSpeed->~PID();
    delete (_pidSpeed);
    _encoder->~QEC_1X();
    delete (_encoder);
    delete (_motor);
    delete (_limitSwitch);
  delete(_enableMotors);
  delete (_spi); 
  delete (_pubGripperOutput);
}