#include "Utils_math.h"
#include "sharedControlGrasp.h"
#include "tf_conversions/tf_eigen.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


const double MIN_THRESHOLD = 3.0; // [DEG]
const double MAX_THRESHOLD = 7.0; // [DEG]
#define STD 0.5

// #define ListofToolAxes(enumeration, names) names,
// char const *Tool_Axis_Names[]{TOOL_AXES};
// #undef ListofToolAxes

char const *Tool_Names[]{"none", "right", "left"};

#define DELAY_SEC 3.0


const float Axis_Limits[] =  {0.090,0.0975,27.5*DEG_TO_RAD,120.0*DEG_TO_RAD};

const float SAMPLING_TIME = 10; // 100Hz

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const double scaleFoot[] = {Axis_Limits[0]/0.15,Axis_Limits[1]/0.15,(1.5*Axis_Limits[2])/(2*0.15),35.0*DEG_TO_RAD/(1.5*M_PI),20.5*DEG_TO_RAD/30*DEG_TO_RAD}; // X, Y, Z, YAW, ROLL;

const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};

sharedControlGrasp *sharedControlGrasp::me = NULL;

  double magnitude_vib = 2.0;
  double decayRate_vib = 3.0;
  double frequency_vib = 32.6;

sharedControlGrasp::sharedControlGrasp(ros::NodeHandle &n_1, double frequency, std::string toolName_)
    : _n(n_1), _loopRate(frequency), _dt(1.0f / frequency){

   me = this;
  _stop = false;


   if (toolName_.compare("right") == 0) {
      _myID = sharedControlGrasp::RIGHT_TOOL;
    } else if (toolName_.compare("left") == 0) {
      _myID = sharedControlGrasp::LEFT_TOOL;
    } else {
      ROS_ERROR("You didn't enter a toolID left or right");
      _stop=true;
    }
  
   
  Eigen::Vector3d alphaPosition;
  alphaPosition.setConstant(0.6);

  Eigen::Vector3d alphaSpeed;
  alphaSpeed.setConstant(0.95);

  Eigen::Vector3d alphaAcc;
  alphaAcc.setConstant(0.95);

  Eigen::Vector3d alphaJerk;
  alphaJerk.setConstant(0.95);


  _vibrationGrasping = 0.0;
  _impedanceGrasping = 0.0;
  _hapticTorques.setZero();
  _precisionAng=0.0;
  _precisionPos=0.0;  
  _precisionGrasp=0.0; 
  _hapticAxisFilterPos=1.0; 

  _graspingAngle=0.0;
  _graspingAngleSpeed=0.0;
    // _toolJointSpeed.setZero();  
    _platformJointPosition.setZero();
    _platformJointPositionOffset.setZero();
    _platformJointVelocity.setZero();
    _platformJointEffort.setZero();
   
    // _flagToolJointsConnected=false;

    _thresholds.setZero();

    _kpPosition.setZero();
    _kiPosition.setZero();
    _kdPosition.setZero();
    
    _posCtrlRef.setZero();
    _posCtrlIn.setZero();
    _posCtrlInPrev.setZero();
    _posCtrlOut.setZero();

    _kpGrasping=0.0;
    _kiGrasping=0.0;
    _kdGrasping=0.0;

    _graspCtrlRef=0.0;
    _graspCtrlIn=0.0;
    _graspCtrlOut=0.0;
    _thresholdFilter.setAlpha(0.9);

    _aStateNext=A_POSITIONING;
    _aState=A_POSITIONING;


   for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
   {
      _pidPosition[i] = new PIDd(&_posCtrlIn(i), &_posCtrlOut(i), &_posCtrlRef(i),
      _kpPosition(i), _kiPosition(i), _kdPosition(i), P_ON_E , DIRECT, 0.9);
      _pidPosition[i]->setMode(AUTOMATIC);
      _pidPosition[i]->setSampleTime(SAMPLING_TIME);
      _kpPositionFilter[i].setAlpha(0.99);
      _kiPositionFilter[i].setAlpha(0.99);
      _kdPositionFilter[i].setAlpha(0.99);
   }
    
    _pidGrasping = new PIDd(&_graspCtrlIn, &_graspCtrlOut, &_graspCtrlRef,
    _kpGrasping, _kiGrasping, _kdGrasping, P_ON_E , DIRECT, 0.9);
    _pidGrasping->setMode(AUTOMATIC);
    _pidGrasping->setSampleTime(SAMPLING_TIME);

    _pidPosition[Axis_Mod[p_x]]->setOutputLimits(-17.0,17.0);
    _pidPosition[Axis_Mod[p_y]]->setOutputLimits(-17.0,17.0);
    _pidPosition[Axis_Mod[p_pitch]]->setOutputLimits(-2.0,2.0);
    _pidPosition[Axis_Mod[p_yaw]]->setOutputLimits(-2.0,2.0);
    
    _pidGrasping->setOutputLimits(-0.5,0.5);

  std::string toolName="right";

  if (!_n.getParam("id", toolName))
  { 
      ROS_ERROR("No indicaton of tool id (right or left) was done"); 
  }


   if (!_n.getParam("hapticGraspOn", _flagHapticGrasping))
  { 
      ROS_ERROR("No indicaton of the subject ID (e.g. sXX) was done"); 
      _flagHapticGrasping = false;
  }
  
     if (!_n.getParam("sharedGraspOn", _flagSharedGrasping))
  { 
      ROS_ERROR("No indicaton of the subject ID (e.g. sXX) was done"); 
      _flagSharedGrasping = false;
  }
    
  if (!_n.getParam("magnitudeVibration", magnitude_vib))
  { 
      ROS_ERROR("No indication of the magnitude of the vibration was given"); 
  }

  if (!_n.getParam("decayRateVibration", decayRate_vib))
  { 
      ROS_ERROR("No indication of the decayRate of the vibration was given"); 
  }

  if (!_n.getParam("frequencyVibration", frequency_vib))
  { 
      ROS_ERROR("No indication of the frequency of the vibration was given"); 
  }

    // _myVibrator = new vibrator(&_toolJointSpeed(tool_wrist_open_angle), &_vibrationGrasping,magnitude_vib,decayRate_vib,frequency_vib,0.0);
    _myVibrator = new vibrator(&_graspingAngleSpeed, &_vibrationGrasping,magnitude_vib,decayRate_vib,frequency_vib,0.0);
    _mySmoothSignals = new smoothSignals(smoothSignals::SMOOTH_RISE, &_hapticAxisFilterPos,3.0); 


}

sharedControlGrasp::~sharedControlGrasp() { me->_n.shutdown(); }

bool sharedControlGrasp::init() 
{
    _hapticTorques.setZero();
    _pubFootInput = _n.advertise<custom_msgs::FootInputMsg_v5>("/"+std::string(Tool_Names[_myID])+"/shared_control_publisher/foot_input",0);
    _pubSharedGrasp = _n.advertise<custom_msgs_gripper::SharedGrasping>("/"+std::string(Tool_Names[_myID])+"/sharedGrasping",0);
    _subToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myID])+"/tool_joint_state_publisher/joint_states"
    , 1, boost::bind(&sharedControlGrasp::readToolState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay()); 
    _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myID])+"/platform_joint_publisher/joint_states"
    , 1, boost::bind(&sharedControlGrasp::readPlatformState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());                  

  // Subscriber definitions
  signal(SIGINT, sharedControlGrasp::stopNode);

  if (_n.ok()) {
    srand(ros::Time::now().toNSec());
    ros::spinOnce();
    ROS_INFO("The shared control for grasping node is about to start ");
    return true;
  }
  else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void sharedControlGrasp::stopNode(int sig) { me->_stop = true;  }

void sharedControlGrasp::run() {

  while (!_stop) {
    _hapticTorques.setZero();
      
      estimateActionState();
      if (_flagSharedGrasping)
      {
        doSharedControl();
      }
      
      _myVibrator->run(ros::Time::now());
      _mySmoothSignals->run(ros::Time::now());
      _hapticTorques(p_roll) = Utils_math<double>::bound(_vibrationGrasping,-2.5, 2.5);
  
      publishFootInput();
      publishSharedGrasp();

    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("The sharedControl spawner stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void sharedControlGrasp::estimateActionState()
{

  _graspingAngle=_toolJointPosition(tool_wrist_open_angle);
  for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
  {
    double speed_comp = Utils_math<double>::bound(1.1*exp(abs(_platformJointVelocity(Axis_Mod[i])))-1.0,0.0,0.5);
    _thresholds(i) = (speed_comp + 1-(1/(STD*sqrt(2*M_PI)))*exp(-0.5*(pow(((_platformJointPosition(Axis_Mod[i])/(0.5*Axis_Limits[i])))/STD,2)))) * (MAX_THRESHOLD-5.0);
  }
  
  _myThreshold = _thresholds.norm();
  
  _myThreshold = _thresholdFilter.update(Utils_math<double>::bound(_myThreshold,MIN_THRESHOLD,MAX_THRESHOLD));
  // if ( (_toolJointPosition(tool_wrist_open_angle)) < (30.0 - _myThreshold) * DEG_TO_RAD)
  if ( _graspingAngle < (30.0 - _myThreshold) * DEG_TO_RAD)
  {
    if (_myVibrator->finished())
      {
        _aStateNext = A_GRASPING;
      }
  }
  else
  {
    if (_myVibrator->finished())
      {
      _aStateNext = A_POSITIONING;
     }
  }

  
  if (_aState!=_aStateNext)
  {
    if (_flagSharedGrasping)
    {
      _myVibrator->reset();
      _mySmoothSignals->reset();
      if (_aStateNext==A_POSITIONING)
      {
        _myVibrator->changeParams(1.0*magnitude_vib,0.7*decayRate_vib,5.0*frequency_vib);
        _mySmoothSignals->changeParams(smoothSignals::SMOOTH_RISE,2.0);
      }
      else
      {
        _myVibrator->changeParams(1.0*magnitude_vib,1.0*decayRate_vib,1.0*frequency_vib);
        _mySmoothSignals->changeParams(smoothSignals::SMOOTH_FALL,0.5);
      }

      _mySmoothSignals->start();
      _myVibrator->start();
    }
    _posCtrlRef = _posCtrlIn;
    _aState = _aStateNext;
  }

};

void sharedControlGrasp::doSharedControl()
{

  switch (_aState)
  {
  case A_POSITIONING:
    {
      _pidPosition[Axis_Mod[p_x]]->reset();
      _pidPosition[Axis_Mod[p_y]]->reset();
      _pidPosition[Axis_Mod[p_pitch]]->reset();
      _pidPosition[Axis_Mod[p_yaw]]->reset();
      _kiPositionFilter[Axis_Mod[p_x]].reset();
      _kiPositionFilter[Axis_Mod[p_y]].reset();
      _kiPositionFilter[Axis_Mod[p_pitch]].reset();
      _kiPositionFilter[Axis_Mod[p_yaw]].reset();

      _kpPosition(Axis_Mod[p_x]) = _kpPositionFilter[Axis_Mod[p_x]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);
      _kpPosition(Axis_Mod[p_y]) = _kpPositionFilter[Axis_Mod[p_y]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);
      
      _kpPosition(Axis_Mod[p_pitch]) = _kpPositionFilter[Axis_Mod[p_pitch]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kpPosition(Axis_Mod[p_yaw]) =   _kpPositionFilter[Axis_Mod[p_yaw]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);
      
      _kiPosition(Axis_Mod[p_x]) = _kiPositionFilter[Axis_Mod[p_x]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);
      _kiPosition(Axis_Mod[p_y]) = _kiPositionFilter[Axis_Mod[p_y]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);
      _kiPosition(Axis_Mod[p_pitch]) = _kiPositionFilter[Axis_Mod[p_pitch]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kiPosition(Axis_Mod[p_yaw]) = _kiPositionFilter[Axis_Mod[p_yaw]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);

      _kdPosition(Axis_Mod[p_x]) = _kdPositionFilter[Axis_Mod[p_x]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);
      _kdPosition(Axis_Mod[p_y]) = _kdPositionFilter[Axis_Mod[p_y]].update(0.0f * SCALE_GAINS_LINEAR_POSITION);

      _kdPosition(Axis_Mod[p_pitch]) = _kdPositionFilter[Axis_Mod[p_pitch]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kdPosition(Axis_Mod[p_yaw])   = _kdPositionFilter[Axis_Mod[p_yaw]].update(0.0f * SCALE_GAINS_ANGULAR_POSITION);
      
      _kpGrasping = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      _kdGrasping = 0.0f * SCALE_GAINS_ANGULAR_POSITION;
      break;
    }
  
  case A_GRASPING:
    {
      _pidGrasping->reset();
            

      _kpPosition(Axis_Mod[p_x]) = _kpPositionFilter[Axis_Mod[p_x]].update(5000.0f * SCALE_GAINS_LINEAR_POSITION);
      _kpPosition(Axis_Mod[p_y]) = _kpPositionFilter[Axis_Mod[p_y]].update(5000.0f * SCALE_GAINS_LINEAR_POSITION);
      
      _kpPosition(Axis_Mod[p_pitch]) = _kpPositionFilter[Axis_Mod[p_pitch]].update(10000.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kpPosition(Axis_Mod[p_yaw]) =   _kpPositionFilter[Axis_Mod[p_yaw]].update(5000.0f * SCALE_GAINS_ANGULAR_POSITION);

      _kiPosition(Axis_Mod[p_x]) = _kiPositionFilter[Axis_Mod[p_x]].update(100.0f * SCALE_GAINS_LINEAR_POSITION);
      _kiPosition(Axis_Mod[p_y]) = _kiPositionFilter[Axis_Mod[p_y]].update(100.0f * SCALE_GAINS_LINEAR_POSITION);
      _kiPosition(Axis_Mod[p_pitch]) = _kiPositionFilter[Axis_Mod[p_pitch]].update(100.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kiPosition(Axis_Mod[p_yaw]) = _kiPositionFilter[Axis_Mod[p_yaw]].update(100.0f * SCALE_GAINS_ANGULAR_POSITION);

      _kdPosition(Axis_Mod[p_x]) = _kdPositionFilter[Axis_Mod[p_x]].update(500.0f * SCALE_GAINS_LINEAR_POSITION);
      _kdPosition(Axis_Mod[p_y]) = _kdPositionFilter[Axis_Mod[p_y]].update(500.0f * SCALE_GAINS_LINEAR_POSITION);

      _kdPosition(Axis_Mod[p_pitch]) = _kdPositionFilter[Axis_Mod[p_pitch]].update(500.0f * SCALE_GAINS_ANGULAR_POSITION);
      _kdPosition(Axis_Mod[p_yaw])   = _kdPositionFilter[Axis_Mod[p_yaw]].update(500.0f * SCALE_GAINS_ANGULAR_POSITION);

      _kpGrasping = 0.0f * SCALE_GAINS_ANGULAR_POSITION; 
      break;
    }
  }
  _posCtrlInPrev =  _posCtrlIn;
  for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
    {
      
      _posCtrlIn(i)=_platformJointPosition(Axis_Mod[i]);
            
      _pidPosition[i]->setTunings(_kpPosition(i),_kiPosition(i),_kdPosition(i));
      
      
      
      if ((_myVibrator->finished() && _aState==A_POSITIONING) || _aState==A_GRASPING)
      {
        _pidPosition[i]->compute(ros::Time::now());        
        _hapticTorques(Axis_Pos[i])=_posCtrlOut(i);    
      }
      else
      {
        _hapticTorques(Axis_Pos[i])=0.0;    
      }

    }
    
    _graspCtrlIn = _graspingAngle;
    _pidGrasping->setTunings(_kpGrasping,_kiGrasping,_kdGrasping);
}

void sharedControlGrasp::readToolState(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_TOOL_AXIS_FULL; i++) {
    me->_toolJointPosition(i) = msg->position[i];
    me->_toolJointSpeed(i) = msg->velocity[i];
  }

  if (!_flagToolJointsConnected) {
    ROS_INFO("Joints received from tool %i",_myID);
  }
  _flagToolJointsConnected = true;
}

void sharedControlGrasp::readPlatformState(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_PLATFORM_AXIS; i++) {
    // me->_platformJointStates_prev(i) = me->_platformJointStates(i);
    me->_platformJointPosition(i) = msg->position[i];
    me->_platformJointVelocity(i) = msg->velocity[i];
    me->_platformJointEffort(i) = msg->effort[i];
  }

  if (!_flagPlatformJointsConnected) {
    ROS_INFO("Joints received from platform %i", _myID);
    me->_platformJointPositionOffset(p_roll)=10.0*DEG_TO_RAD;
  }
  _flagPlatformJointsConnected = true;
}

void sharedControlGrasp::publishFootInput()
{
    _msgFootInput.ros_effort.fill(0.0f);
    _msgFootInput.ros_position.fill(0.0f);
    _msgFootInput.ros_forceSensor.fill(0.0f);
    _msgFootInput.ros_speed.fill(0.0f);
    _msgFootInput.ros_filterAxisForce.fill(1.0f); 
    _hapticAxisFilterGrasp=1.0f;

    for (unsigned int i=0; i<NB_PLATFORM_AXIS; i++)
    {
      _msgFootInput.ros_effort[i] =  _hapticTorques(i);
    }
    if (_flagSharedGrasping)
    {
      if (_aState == A_GRASPING)
      {
      
        _msgFootInput.ros_filterAxisForce.fill(_hapticAxisFilterPos); 
        _msgFootInput.ros_filterAxisForce[p_roll] = _hapticAxisFilterGrasp;
      }
      if (_aState == A_POSITIONING)
      {
        _msgFootInput.ros_filterAxisForce.fill(_hapticAxisFilterPos); 
        _msgFootInput.ros_filterAxisForce[p_roll] = _hapticAxisFilterGrasp;
      }
    }
    _pubFootInput.publish(_msgFootInput); 
}
  

void sharedControlGrasp::publishSharedGrasp()
{
  _msgSharedGrasp.sGrasp_id = _myID;
  _msgSharedGrasp.sGrasp_stamp = ros::Time::now();
  _msgSharedGrasp.sGrasp_aState=_aState;
  _msgSharedGrasp.sGrasp_threshold =_myThreshold; // In degrees
  for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
  {
      if (_flagSharedGrasping)
      {
        _msgSharedGrasp.sGrasp_hFilters.fill(_hapticAxisFilterPos);
        _msgSharedGrasp.sGrasp_hFilters[p_roll]=_hapticAxisFilterGrasp;
      }
      else
      {
        _msgSharedGrasp.sGrasp_hFilters.fill(1.0f);
      }
      
  }
  _pubSharedGrasp.publish(_msgSharedGrasp);

}
  
