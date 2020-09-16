#include "Gripper.h"

//! #1


void Gripper::getMotion()
{
  getPosition();
  getSpeed();
  getAcceleration();
}




void Gripper::getPosition()
{
  if ((_timestamp - _posSamplingStamp) >= (uint32_t)POSITION_PID_SAMPLE_P)
  {
    float encoder_out = 0.0f;
    _spi->lock();

    encoder_out = _encoder->QEC_getPosition(_spi);

    _spi->unlock();
    // Adapt roll and yaw angles due to differential mechanism
      _position = encoder_out + _positionOffset ;
    _posSamplingStamp = _timestamp;
  }
}

void Gripper::getSpeed()
{
  if ((_timestamp-_speedSamplingStamp)>=(uint32_t)VELOCITY_PID_SAMPLE_P)
  {
    _speed = (_position - _positionPrev) * invSpeedSampT;
    _speed = _speedFilter.update(_speed);

    _positionPrev = _position;
    _speedSamplingStamp = _timestamp;
  }
}

//! #5
void Gripper::getAcceleration()
{
  if ((_timestamp - _accSamplingStamp) >= (uint32_t)ACC_SAMPLE_P)
  {
    _acceleration = (_speed - _speedPrev) * invAccSampT;
  
    _acceleration = _accFilter.update(_acceleration);

    _speedPrev = _speed;
    _accSamplingStamp = _timestamp;
  }
}
