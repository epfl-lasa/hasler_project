#include "Gripper.h"



void Gripper::setSpeed(float speed)
{
  float driver_speed;
  driver_speed = clip(speed, -NOMINAL_SPEED_GEARBOX_GRIPPER, NOMINAL_SPEED_GEARBOX_GRIPPER);
  setPWM(driver_speed);
}

//! 3
void Gripper::setPWM(float pwm)
{
  float driver_PWM = map(MOTORSIGN_GRIPPER*pwm, -NOMINAL_SPEED_GEARBOX_GRIPPER, NOMINAL_SPEED_GEARBOX_GRIPPER, 0.0f, 1.0f); //! 
  _motor->write(driver_PWM);
}

