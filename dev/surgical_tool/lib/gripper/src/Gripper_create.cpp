#include "Gripper.h"



Gripper *Gripper::me = NULL;

Gripper::Gripper(GrippersId name_):
_gripperName(name_)
{
  me = this;
  _stop=false;
  _innerTimer.start(); // Start Running the Timer -> I moved it to the constructor
  _flagEmergencyCalled = false;
    
  _positionD=0.0f;
  _positionCtrlOut=0.0f;
  _speedCtrlOut=0.0f;
  _positionD_filtered=0.0f;
  _position=0.0f;
  _positionPrev=0.0f;
  _positionOffset=0.0f;
  _speed=0.0f;
  _speedD=0.0f;
  _speedPrev=0.0f;
  _acceleration=0.0f;


     positionCtrlClear();
    _gripper_kpPosition = 0.0f;
    _gripper_kiPosition = 0.0f;
    _gripper_kdPosition = 0.0f;

    speedCtrlClear();
    _gripper_kpSpeed = 0.0f;
    _gripper_kiSpeed = 0.0f;
    _gripper_kdSpeed = 0.0f;

    _posDesiredFilter.setAlpha(0.5f);

    limitSwitchesClear();

    _ros_position=0.0f;
    _ros_speed=0.0f;

    _pidPosition = new PID(&_innerTimer, &_position, &_positionCtrlOut, &_positionD_filtered, _gripper_kpPosition, _gripper_kiPosition, _gripper_kdPosition, DIRECT, POS_PID_FILTER_GAINS);
    _pidPosition->setMode(AUTOMATIC);
    _pidSpeed = new PID(&_innerTimer, &_speed, &_speedCtrlOut, &_speedD, _gripper_kpSpeed, _gripper_kiSpeed, _gripper_kdSpeed,DIRECT, VEL_PID_FILTER_GAINS);
    _pidSpeed->setMode(AUTOMATIC);
    _flagInWsConstrains = false;

  _speedFilter.setAlpha(VEL_PID_FILTER_GAINS);

  _accFilter.setAlpha(0.96);

  _innerCounterADC=0;

  _ros_controllerType=SPEED_CTRL;
  _gripper_controllerType=_ros_controllerType;
  _flagClearLastState=false;
  _flagControllerTypeChanged=false;
  _flagDefaultCtrlNew = false;
  _flagCtrlGainsNew = false;

  _ros_flagDefaultControl=true;
  _gripper_flagDefaultControl = true;

  for(int c = 0; c<NB_GI_CATEGORY; c++)
  {
    _flagInputReceived[c] = false; //! To be used specially for the telemanipulation state
  }

  _tic=false;
  _ros_state = EMERGENCY;
  _gripper_state=_ros_state;
  
    
  // Reset the flags that acknowledge when the state is entered for the first time 
  _enterStateOnceFlag[HOMING]=false;
  _enterStateOnceFlag[STANDBY]=false;
  _enterStateOnceFlag[GRIPPER_STATE_CONTROL]=false;

  /*******DESIGNATIONS OF PINS IN THE MICROCONTROLLER NUCLEO L476RG */

  _csPin = PB_10;  //! CS2  -> Dorsi/Plantar Flexion NOT as PWMX/XN
  
  _motorPin = PB_3; // D3  PWM2/2

  _limitSwitchPin = PC_5; // NOT as PWMX/XN
  

  _spi = new SPI(PA_7, PA_6, PA_5); // mosi, miso, sclk https://os.mbed.com/grippers/ST-Nucleo-L476RG/
  // _spi->format(8,0); // Default
  // _spi->frequency(1000000); // Default
  /************************************************************* */

    ThisThread::sleep_for(10); //! Wait a bit after the SPI starts

    _encoder = new QEC_1X(_csPin);
    _motor = new PwmOut(_motorPin);  


    _limitSwitch = new InterruptIn(_limitSwitchPin);
    _limitSwitch->mode(PullUp);

    _limitSwitch=NULL;
      
  _timestamp=0; // We don't read the timer until the gripper is initialized

  _recoveringFromError=false;

}