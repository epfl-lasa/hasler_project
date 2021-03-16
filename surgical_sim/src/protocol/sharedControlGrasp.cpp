#include "Utils_math.h"
#include "sharedControlGrasp.h"
#include "tf_conversions/tf_eigen.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


const double MIN_THRESHOLD = 0.0; // [DEG]
const double MAX_THRESHOLD = 7.0; // [DEG]
#define STD 0.5

// #define ListofToolAxes(enumeration, names) names,
// char const *Tool_Axis_Names[]{TOOL_AXES};
// #undef ListofToolAxes

char const *Tool_Names[]{"none", "right", "left"};

char const *State_Names[]{"positioning with open gripper",
                          "grasping",
                          "holding grasp",
                          "positioning with closed gripper",
                          "closing gripper to regain control",
                          "releasing grasp"};


#define DELAY_SEC 3.0


const float Axis_Limits[] =  {0.090f,0.0975f,27.5f*DEG_TO_RAD,120.0f*DEG_TO_RAD};

const float SAMPLING_TIME = 2.5; // in ms 

const int Axis_Mod[NB_PLATFORM_AXIS] = {p_y,p_x,p_pitch,p_yaw,p_roll};

const float Scale_Foot[] = {0.15f/Axis_Limits[0],0.15f/Axis_Limits[1],(2.0f*0.15f)/(1.5f*Axis_Limits[2]),(1.5f* (float) M_PI)/50.0f*DEG_TO_RAD,30.0f*DEG_TO_RAD/20.5f*DEG_TO_RAD}; // Y=1.66[m/m], X=1.54[m/m], Z=0.42[m/rad], YAW=5.41[rad/rad], ROLL(Grasping)=1.46[rad/rad];


const int Axis_Pos[NB_PLATFORM_AXIS] = {p_x,p_y,p_pitch,p_yaw,p_roll};

sharedControlGrasp *sharedControlGrasp::me = NULL;

  double magnitude_vib = 10.0;
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
      ROS_ERROR("[%s sharedControl] You didn't enter a toolID left or right", Tool_Names[_myID]);
      _stop=true;
    }
  _realGripperSpeed=0.0f;
  _realGripperErrorPos=0.0f;
  _realGripperPosition=0.0f;

  
   
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
  _hapticAxisFilterGrasp=1.0;

  _graspingAnglePrev=0.0;
  _graspingAngle=0.0;
  _graspingAngleSpeed=0.0;
  _vibInput=0.0;
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

    // _kpGrasping=0.0;
    // _kiGrasping=0.0;
    // _kdGrasping=0.0;

    // _graspCtrlRef=0.0;
    // _graspCtrlIn=0.0;
    // _graspCtrlOut=0.0;
    _thresholdFilter.setAlpha(0.9);

    _aStateNext=A_POSITIONING_OPEN;
    _aState=A_GRASPING;


   for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
   {
      _pidPosition[i] = new PIDd(&_posCtrlIn(i), &_posCtrlOut(i), &_posCtrlRef(i),
      _kpPosition(i), _kiPosition(i), _kdPosition(i), P_ON_E , DIRECT, 0.9);
      _pidPosition[i]->setMode(AUTOMATIC);
      _pidPosition[i]->setSampleTime(SAMPLING_TIME);
      _kpPositionFilter[i].setAlpha(0.99);
      _kiPositionFilter[i].setAlpha(0.99);
      _kdPositionFilter[i].setAlpha(0.99);
      _pidPosition[i]->setTunings(_kpPosition(i),_kiPosition(i),_kdPosition(i));
   }
    
    // _pidGrasping = new PIDd(&_graspCtrlIn, &_graspCtrlOut, &_graspCtrlRef,
    // _kpGrasping, _kiGrasping, _kdGrasping, P_ON_E , DIRECT, 0.9);
    // _pidGrasping->setMode(AUTOMATIC);
    // _pidGrasping->setSampleTime(SAMPLING_TIME);

    _pidPosition[Axis_Mod[p_x]]->setOutputLimits(-17.0,17.0);
    _pidPosition[Axis_Mod[p_y]]->setOutputLimits(-17.0,17.0);
    _pidPosition[Axis_Mod[p_pitch]]->setOutputLimits(-2.0,2.0);
    _pidPosition[Axis_Mod[p_yaw]]->setOutputLimits(-2.0,2.0);
    
    // _pidGrasping->setOutputLimits(-0.5,0.5);

  std::string simType="dynamic";

  if (!_n.getParam("simType", simType))
  { 
      ROS_ERROR("[%s sharedControl] No indicaton of simType (kinematic or dynamic) was done", Tool_Names[_myID]); 
  }
  if(simType.compare("kinematic")==0){
    _mySimType=KINEMATIC_SIM;
    ROS_INFO("[%s sharedControl] This is a kinematic simulation", Tool_Names[_myID]); 

  }else
  {
    _mySimType=DYNAMIC_SIM;
    ROS_INFO("[%s sharedControl] This is a dynamic simulation", Tool_Names[_myID]); 
  }

   if (!_n.getParam("hapticGraspOn", _flagHapticGrasping))
  { 
      ROS_ERROR("[%s sharedControl] No indicaton of the haptic grasp was done", Tool_Names[_myID]); 
      _flagHapticGrasping = false;
  }
  
     if (!_n.getParam("sharedGraspOn", _flagSharedGrasping))
  { 
      ROS_ERROR("[%s sharedControl] No indicaton of the shared grasp was done", Tool_Names[_myID]); 
      _flagSharedGrasping = false;
  }
    
  if (!_n.getParam("magnitudeVibration", magnitude_vib))
  { 
      ROS_ERROR("[%s sharedControl] No indication of the magnitude of the vibration was given", Tool_Names[_myID]); 
  }

  if (!_n.getParam("decayRateVibration", decayRate_vib))
  { 
      ROS_ERROR("[%s sharedControl] No indication of the decayRate of the vibration was given", Tool_Names[_myID]); 
  }

  if (!_n.getParam("frequencyVibration", frequency_vib))
  { 
      ROS_ERROR("[%s sharedControl] No indication of the frequency of the vibration was given", Tool_Names[_myID]); 
  }

    // _myVibrator = new vibrator(&_toolJointSpeed(tool_wrist_open_angle), &_vibrationGrasping,magnitude_vib,decayRate_vib,frequency_vib,0.0);
  _myVibrator = new vibrator<double>(&_vibInput, &_vibrationGrasping,magnitude_vib,decayRate_vib,frequency_vib,0.0);
  _mySmoothSignalsPos = new smoothSignals<double>(smoothSignals<double>::SMOOTH_RISE, &_hapticAxisFilterPos,3.0); 
  _mySmoothSignalsGrasp = new smoothSignals<double>(smoothSignals<double>::SMOOTH_RISE, &_hapticAxisFilterGrasp,3.0); 
  _flagGraspingStarted=false;
  _flagHoldingGraspStarted=false;
  _flagPositioningStarted=false;
  _flagSurgicalTaskStateReceived=false;

}

sharedControlGrasp::~sharedControlGrasp() { me->_n.shutdown(); }

bool sharedControlGrasp::init() 
{
    _hapticTorques.setZero();
    _pubFootInput = _n.advertise<custom_msgs::FootInputMsg>("/"+std::string(Tool_Names[_myID])+"/shared_control_publisher/foot_input",0); //! should not have _tool
    _pubSharedGrasp = _n.advertise<custom_msgs_gripper::SharedGraspingMsg>("/"+std::string(Tool_Names[_myID])+"_tool/sharedGrasping",0);
    _subToolJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myID])+"_tool/joint_states"
    , 1, boost::bind(&sharedControlGrasp::readToolState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay()); 
    _subPlatformJointStates = _n.subscribe<sensor_msgs::JointState>( "/"+std::string(Tool_Names[_myID])+"_platform/platform_joint_publisher/joint_states"
    , 1, boost::bind(&sharedControlGrasp::readPlatformState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subSurgicalTaskStates = _n.subscribe<custom_msgs::SurgicalTaskStateMsg>( "/surgicalTaskState"
    , 1, boost::bind(&sharedControlGrasp::readSurgicalTaskState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subGripperOutput = _n.subscribe<custom_msgs_gripper::GripperOutputMsg>( "/right/gripperOutput"
    , 1, boost::bind(&sharedControlGrasp::readGripperOutput, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());          

  // Subscriber definitions
  signal(SIGINT, sharedControlGrasp::stopNode);

  if (_n.ok()) {
    srand(time(NULL));
    ros::spinOnce();
    ROS_INFO("[%s sharedControl] The shared control for grasping node is about to start ", Tool_Names[_myID]);
    return true;
  }
  else {
    ROS_ERROR("[%s sharedControl] The ros node has a problem", Tool_Names[_myID]);
    return false;
  }
}

void sharedControlGrasp::stopNode(int sig) { me->_stop = true;  }

void sharedControlGrasp::run() {

  while (!_stop) {
      _hapticTorques.setZero();
      //cout<<"Current state: "<<State_Names[_aState]<<endl;
      if (_flagSharedGrasping)
      {
        estimateActionState();
        estimateActionTransition();
        doSharedControl();
            
      _myVibrator->run(ros::Time::now());
      _mySmoothSignalsPos->run(ros::Time::now());
      _mySmoothSignalsGrasp->run(ros::Time::now());
      _hapticTorques(p_roll) = Utils_math<double>::bound(_vibrationGrasping,-1.5, 1.5);
      }
      publishFootInput();
      publishSharedGrasp();
      
    ros::spinOnce();
    _loopRate.sleep();
  }
  ROS_INFO("[%s sharedControl] The sharedControl spawner stopped", Tool_Names[_myID]);
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void sharedControlGrasp::estimateActionState()
{
  _graspingAnglePrev = _graspingAngle;
  _graspingAngle = Utils_math<double>::map(fabs(_platformJointPosition(p_roll)) + -0.10,
                                                    0.0,0.126892, 
                                                    0,30.0*DEG_TO_RAD);

  //cout<<_graspingAngle<<endl;
  // _graspingAngleSpeed = (_graspingAngle - _graspingAnglePrev) * (1.0 / _dt);
  _graspingAngleSpeed = _platformJointVelocity(p_roll);
  for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
  {
    double speed_comp = Utils_math<double>::bound(1.1*exp(abs(_platformJointVelocity(Axis_Mod[i])))-1.0,0.0,0.5);
    _thresholds(i) = (speed_comp + 1-(1/(STD*sqrt(2*M_PI)))*exp(-0.5*(pow(((_platformJointPosition(Axis_Mod[i])/(0.5*Axis_Limits[i])))/STD,2)))) * (MAX_THRESHOLD-5.0);
  }
  
  _myThreshold = _thresholds.norm();
  
  _myThreshold = _thresholdFilter.update(Utils_math<double>::bound(_myThreshold,MIN_THRESHOLD,MAX_THRESHOLD));
  // if ( (_toolJointPosition(tool_wrist_open_angle)) < (30.0 - _myThreshold) * DEG_TO_RAD)
  // Decision making
   bool graspinAngleThreshold1 = _graspingAngle > (_myThreshold + 2.0) * DEG_TO_RAD;
   bool contactFSOn = _wrenchGrasperRobot.norm() > 2.0;
   bool realGripperErrorBig = _realGripperErrorPos > 15.0f*DEG_TO_RAD;
   bool realGripperSpeedLow = fabs(_realGripperSpeed) < 5.0f * DEG_TO_RAD;


  if (_mySimType==DYNAMIC_SIM)
  {
    switch (_aState)
    {

    case A_POSITIONING_OPEN:
      {

          if ( _graspingAngle > ((_myThreshold) * DEG_TO_RAD))
          { 
            if (_myVibrator->finished())
              {
                _aStateNext = A_GRASPING;
                _startTimeForKeepingGrasp=ros::Time::now();
              }
          } else {
            _aStateNext=_aState;
          }
      break;
      }
      case A_GRASPING:
      {

          if ( graspinAngleThreshold1 && fabs(_graspingAngleSpeed)<0.01)

          {
              double deltaTimeKeeping = ros::Time::now().toSec()-_startTimeForKeepingGrasp.toSec(); 
              //cout<<deltaTimeKeeping<<" "<<_graspingAngle<<" "<<fabs(_graspingAngleSpeed)<<endl;
              if (deltaTimeKeeping>1.5)
              // if (contactFSOn && realGripperErrorBig && realGripperErrorBig)
                {
                  _aStateNext=A_HOLDING_GRASP;
                  _graspingAngleBeforeHolding=_graspingAngle;
                } 
          }
          else
          {
            //  _startTimeForKeepingGrasp=ros::Time::now();
            if ( _graspingAngle < ((_myThreshold) * DEG_TO_RAD))
            {
                _aStateNext=A_POSITIONING_OPEN;
            }
            else
            {
              _aStateNext=A_GRASPING;
            }
          }

      break;
      }
    case A_HOLDING_GRASP:
      {

          if ( _graspingAngle < (_myThreshold) * DEG_TO_RAD)
          {
            _aStateNext=A_POSITIONING_CLOSE;
          }
          else
          {
            _aStateNext=A_HOLDING_GRASP;
          }
          break;
      }


    case A_POSITIONING_CLOSE:
      {

          if ( _graspingAngle > (_myThreshold) * DEG_TO_RAD)
          {
            if (_myVibrator->finished())
              {
                _aStateNext = A_FETCHING_OLD_GRASP;
                _startTimeForReleasingGrasp=ros::Time::now();
              }
          }
          else
          {
            _aStateNext=A_POSITIONING_CLOSE;
          }
      break;
      }
    case A_FETCHING_OLD_GRASP:
    {

        if (_graspingAngle>(_graspingAngleBeforeHolding-3.5) && fabs(_graspingAngleSpeed)<0.01)
        {
            if (_myVibrator->finished())
            {
                  double deltaTimeReleasing = ros::Time::now().toSec()-_startTimeForReleasingGrasp.toSec(); 
                  cout<<deltaTimeReleasing<<" "<<_graspingAngle<<" "<<fabs(_graspingAngleSpeed)<<endl;
                  if (deltaTimeReleasing>1.0)
                  {
                    _aStateNext = A_RELEASE_GRASP;
                  } 
            }

        }
        else
            {
              _startTimeForKeepingGrasp=ros::Time::now();
            if ( _graspingAngle < (_myThreshold) * DEG_TO_RAD)
            {
                _aStateNext=A_POSITIONING_CLOSE;

            }
            }
      break;

    }

    case A_RELEASE_GRASP:
    {

      if ( _graspingAngle < (_myThreshold) * DEG_TO_RAD)
        {
          if (_myVibrator->finished())
          { 
            _aStateNext=A_POSITIONING_OPEN;
            
          }
        }
      break;
    }
    
    default:
      break;
    }
  }else{ // KINEMATIC SIMULATION
    switch (_aState)
    {

    case A_POSITIONING_OPEN:
      {
        if ( _graspingAngle > ((_myThreshold) * DEG_TO_RAD)){ 
            
          if (_myVibrator->finished()){
            _aStateNext = A_GRASPING;
          }
        }
        else {
            _aStateNext=_aState;
          }
        break;
      }
    case A_GRASPING:
    {
        if ( _graspingAngle < ((_myThreshold) * DEG_TO_RAD)){

          _aStateNext=A_RELEASE_GRASP;
          _startTimeForChangingGainsToZero=ros::Time::now();
        }
        else {
          _aStateNext=_aState;
        }
        break;
    }
    case A_RELEASE_GRASP:
    {
        if ((ros::Time::now().toNSec() - _startTimeForChangingGainsToZero.toNSec())>20000000){
          _aStateNext=A_POSITIONING_OPEN;
        }else {
          _aStateNext=_aState;
        }
        break;
    }
    default:
      break;
    }
  }

};


void sharedControlGrasp::estimateActionTransition()
{

if (_aState!=_aStateNext){
    if (_flagSharedGrasping)
    {
       
       _mySmoothSignalsPos->reset();
       _mySmoothSignalsGrasp->reset();
      
      switch (_aStateNext)
      {
      case A_POSITIONING_OPEN: case A_POSITIONING_CLOSE:
      {
        for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
        {
          _pidPosition[i]->reset();
          _kpPosition(i) = 0.0;
          _kiPosition(i) = 0.0;
          _kdPosition(i) = 0.0;
        }

        break;
      }
      
      case A_RELEASE_GRASP:
      
      {
        if (_aState!=A_HOLDING_GRASP)
        {
          _myVibrator->reset();
          _myVibrator->changeParams(1.0*magnitude_vib,2.0*decayRate_vib,1.5*frequency_vib);
          _myVibrator->start();
        }
        break;
      }
      default:
        {
          _myVibrator->reset();
          _kpPosition(Axis_Mod[p_x]) = 1000.0f * SCALE_GAINS_LINEAR_POSITION;
          _kpPosition(Axis_Mod[p_y]) = 1000.0f * SCALE_GAINS_LINEAR_POSITION;
          _kpPosition(Axis_Mod[p_pitch]) = 1000.0f * SCALE_GAINS_ANGULAR_POSITION;
          _kpPosition(Axis_Mod[p_yaw]) =   1000.0f * SCALE_GAINS_ANGULAR_POSITION;

          _kiPosition(Axis_Mod[p_x]) =     10.0f * SCALE_GAINS_LINEAR_POSITION;
          _kiPosition(Axis_Mod[p_y]) =     10.0f * SCALE_GAINS_LINEAR_POSITION;
          _kiPosition(Axis_Mod[p_pitch]) = 10.0f * SCALE_GAINS_ANGULAR_POSITION;
          _kiPosition(Axis_Mod[p_yaw]) =   10.0f * SCALE_GAINS_ANGULAR_POSITION;

          _kdPosition(Axis_Mod[p_x]) = 50.0f * SCALE_GAINS_LINEAR_POSITION;
          _kdPosition(Axis_Mod[p_y]) = 50.0f * SCALE_GAINS_LINEAR_POSITION;
          _kdPosition(Axis_Mod[p_pitch]) = 50.0f * SCALE_GAINS_ANGULAR_POSITION;
          _kdPosition(Axis_Mod[p_yaw])   = 50.0f * SCALE_GAINS_ANGULAR_POSITION;
          if (_aStateNext!=A_HOLDING_GRASP){
            
            _myVibrator->changeParams(1.0*magnitude_vib,1.0*decayRate_vib,1.0*frequency_vib);
          } else {
            _myVibrator->changeParams(0.65*magnitude_vib,0.7*decayRate_vib,1.3*frequency_vib);
            }
          
          _myVibrator->start();

        }
        break;
      }
      if (_aStateNext==A_GRASPING || _aStateNext==A_FETCHING_OLD_GRASP)
      { 
        _posCtrlRef = _posCtrlIn;
      }
        
      _mySmoothSignalsPos->start();
      _mySmoothSignalsGrasp->start();
      
      _aState = _aStateNext;
      ROS_INFO("[%s sharedGrasp]: Next State: %s",Tool_Names[_myID],State_Names[_aStateNext]);
    
      for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
      {
        _pidPosition[i]->setTunings(_kpPosition(i),_kiPosition(i),_kdPosition(i));
      }
    }  
  }
}

void sharedControlGrasp::doSharedControl()
{
  _posCtrlInPrev =  _posCtrlIn;
  for (size_t i = 0; i < NB_AXIS_POSITIONING; i++)
    {
      
      _posCtrlIn(i)=_platformJointPosition(Axis_Mod[i]);  
      _pidPosition[i]->compute(ros::Time::now());        
      _hapticTorques(Axis_Pos[i])=_posCtrlOut(i) * fabs(1.0- _hapticAxisFilterPos);    
    }

}

void sharedControlGrasp::readToolState(const sensor_msgs::JointState::ConstPtr &msg) {

  for (unsigned int i = 0; i < NB_TOOL_AXIS_FULL; i++) {
    me->_toolJointPosition(i) = msg->position[i];
    me->_toolJointSpeed(i) = msg->velocity[i];
  }

  if (!_flagToolJointsConnected) {
    ROS_INFO("[%s sharedControl] Joints received from tool %i",Tool_Names[_myID],_myID);
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
    ROS_INFO("[%s sharedControl] Joints received from platform %i",Tool_Names[_myID],_myID);
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
    

    for (unsigned int i=0; i<NB_PLATFORM_AXIS; i++)
    {
      _msgFootInput.ros_effort[i] =  _hapticTorques(i);
    }
    if (_flagSharedGrasping)
    {
      _msgFootInput.ros_filterAxisForce.fill(_hapticAxisFilterPos); 
      _msgFootInput.ros_filterAxisForce[p_roll] = 1.0;
    }
    _pubFootInput.publish(_msgFootInput); 
}
  

void sharedControlGrasp::publishSharedGrasp()
{
  _msgSharedGrasp.sGrasp_id = _myID;
  _msgSharedGrasp.sGrasp_stamp = ros::Time::now();
  _msgSharedGrasp.sGrasp_aState=_aState;
  _msgSharedGrasp.sGrasp_threshold =_myThreshold; // In degrees
  if (_flagSharedGrasping)
      {

        switch (_aState)
        {
        case A_POSITIONING_OPEN:
          {
            // _msgSharedGrasp.sGrasp_hFilters.fill(1.0);
            // _msgSharedGrasp.sGrasp_hFilters[p_roll]=1.0;
            _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_RISE,0.1);
            _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_RISE,0.1);
          break;
          }
          case A_GRASPING:
          {
            // _msgSharedGrasp.sGrasp_hFilters.fill(0.0);
            // _msgSharedGrasp.sGrasp_hFilters[p_roll]=1.0;              
            _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
            _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_RISE,0.1);
          break;
          }
        case A_HOLDING_GRASP:
          {
            // _msgSharedGrasp.sGrasp_hFilters.fill(0.0);
            _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
            _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
              break;
          }
          

        case A_POSITIONING_CLOSE:
          {
            // _msgSharedGrasp.sGrasp_hFilters.fill(1.0);
            // _msgSharedGrasp.sGrasp_hFilters[p_roll]=0.0; 
            _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_RISE,0.1);
            _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
          break;
          }
        case A_FETCHING_OLD_GRASP:
        {
          //  _msgSharedGrasp.sGrasp_hFilters.fill(0.0);
          //  _msgSharedGrasp.sGrasp_hFilters[p_roll]=0.0; 

          _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
          _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);

          break;

        }

        case A_RELEASE_GRASP:
        {
          //  _msgSharedGrasp.sGrasp_hFilters.fill(0.0);
          //  _msgSharedGrasp.sGrasp_hFilters[p_roll]=1.0; 
          _mySmoothSignalsPos->changeParams(smoothSignals<double>::SMOOTH_FALL,0.1);
          _mySmoothSignalsGrasp->changeParams(smoothSignals<double>::SMOOTH_RISE,0.1);
          break;
        }
        }



        for (size_t i = 0; i < NB_PLATFORM_AXIS; i++)
        {  
         _msgSharedGrasp.sGrasp_hapticTorques[i]=_hapticTorques(i);
        }
        
        _msgSharedGrasp.sGrasp_hFilters.fill(_hapticAxisFilterPos);
        _msgSharedGrasp.sGrasp_hFilters[p_roll]=_hapticAxisFilterGrasp;

      }
      else
      {
        _msgSharedGrasp.sGrasp_hFilters.fill(1.0f);
      }

      
  
  _pubSharedGrasp.publish(_msgSharedGrasp);

}
  

  
void sharedControlGrasp::readSurgicalTaskState(const custom_msgs::SurgicalTaskStateMsgConstPtr& msg)
{
    tf::wrenchMsgToEigen(msg->allToolsWrench[2],_wrenchGrasperRobot);

    if (!_flagSurgicalTaskStateReceived)
    {
      _flagSurgicalTaskStateReceived=true;
    }    
}


void sharedControlGrasp::readGripperOutput(const custom_msgs_gripper::GripperOutputMsgConstPtr& msg)
{
    _realGripperPosition = msg->gripper_position;
    _realGripperErrorPos = msg->gripper_desPosition - _realGripperPosition;
    
    if (!_flagGripperOutputMsgReceived)
    {
      _flagGripperOutputMsgReceived=true;
    }    
}