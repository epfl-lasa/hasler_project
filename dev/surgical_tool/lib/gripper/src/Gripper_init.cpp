#include "Gripper.h"

void Gripper::init()
{
    
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
   
    _motor->period_us(20); // PWM (to ESCON) PERIOD 20 us-> 50kHz    
    _pidPosition->setSampleTime(POSITION_PID_SAMPLE_P); //! [us]
    _pidSpeed->setSampleTime(VELOCITY_PID_SAMPLE_P);

    loadDefaultPIDGains();

    _spi->lock();
    _encoder->QEC_init(0, ENCODERSCALE_GRIPPER, ENCODERSIGN_GRIPPER, _spi);

  _spi->unlock(); 
  std::string GRIPPER_SUBSCRIBER_NAME = _gripperName == RIGHT_GRIPPER ? GRIPPER_SUBSCRIBER_NAME_RIGHT : GRIPPER_SUBSCRIBER_NAME_LEFT;
  std::string GRIPPER_PUBLISHER_NAME = _gripperName == RIGHT_GRIPPER ? GRIPPER_PUBLISHER_NAME_RIGHT : GRIPPER_PUBLISHER_NAME_LEFT;
  std::string SERVICE_CHANGE_STATE_NAME = _gripperName == RIGHT_GRIPPER ? SERVICE_CHANGE_STATE_NAME_RIGHT : SERVICE_CHANGE_STATE_NAME_LEFT;
  std::string SERVICE_CHANGE_CTRL_NAME = _gripperName == RIGHT_GRIPPER ? SERVICE_CHANGE_CTRL_NAME_RIGHT : SERVICE_CHANGE_CTRL_NAME_LEFT;
  
  _subGripperInput = new ros::Subscriber<custom_msgs_gripper::GripperInputMsg>(GRIPPER_SUBSCRIBER_NAME.c_str(), updateGripperInput);
  _servChangeState = new ros::ServiceServer<custom_msgs_gripper::gripperSetStateSrv::Request,custom_msgs_gripper::gripperSetStateSrv::Response>(SERVICE_CHANGE_STATE_NAME.c_str(), updateState);
  _servChangeCtrl = new ros::ServiceServer<custom_msgs_gripper::gripperSetControllerSrv::Request,custom_msgs_gripper::gripperSetControllerSrv::Response>(SERVICE_CHANGE_CTRL_NAME.c_str(), updateController);
  
  _pubGripperOutput = new ros::Publisher(GRIPPER_PUBLISHER_NAME.c_str(), &_msgGripperOutput);

  _nh.initNode();
   ThisThread::sleep_for(10);
  _nh.advertise(*_pubGripperOutput);
  _nh.advertiseService(*_servChangeState);
  _nh.advertiseService(*_servChangeCtrl);
  _nh.subscribe(*_subGripperInput);
   ThisThread::sleep_for(10);
   _timestamp = _innerTimer.read_us();
   _timestep = CTRL_LOOP;
  //_timestamp = _innerTimer.read_us();
  _posSamplingStamp = _timestamp;
  _speedSamplingStamp=_timestamp;
  _accSamplingStamp=_timestamp;
  _analogReadStamp=_timestamp;
  _enableMotors->write(0);
}


//******************************LIMIT-SWITCHES-CALLBACKS****************

//! WATCH OUT-> THE DEFINITION OF THE INTERRUPTION CALLBACKS HAVE TO BE IN THE SAME C++

void Gripper::switchCallback()
{
  if ((me->_switchesState == 0) )
  {
    me->_switchesState = 1;
  }
}