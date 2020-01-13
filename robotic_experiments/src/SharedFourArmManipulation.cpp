#include "SharedFourArmManipulation.h"
#include "Utils.h"
#include "FootPlatformModel.h"

SharedFourArmManipulation* SharedFourArmManipulation::me = NULL;

SharedFourArmManipulation::SharedFourArmManipulation(ros::NodeHandle &n, double frequency, std::string filename):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename)
{
  me = this;

  _useRobot[LEFT] = true;
  _useRobot[RIGHT] = true;
  _useSim = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE[LEFT] = 0.1315f;
  _toolOffsetFromEE[RIGHT] = 0.1265f;
  _toolMass = 0.2f;

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _x0[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    _wrenchBias[k].setConstant(0.0f);
    _wrench[k].setConstant(0.0f);
    _filteredWrench[k].setConstant(0.0f);
    
    _xd[k].setConstant(0.0f);
    _fx[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _qd[k] << 0.0f,0.0f,1.0f,0.0f;
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _normalForce[k] = 0.0f;
    _Fd[k] = 0.0f;
    _Fdh[k] = 0.0f;

    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = true;
    _waitForFoot[k] = false;

    _wrenchCount[k] = 0;
    _wrenchBiasOK[k] = false;
    _d1[k] = 1.0f;

    _footPose[k].setConstant(0.0f);
    _footWrench[k].setConstant(0.0f);
    _footTwist[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _xdFoot[k].setConstant(0.0f);
    _vdFoot[k].setConstant(0.0f);
    _footInterfaceSequenceID[k] = 0;    
    _firstFootInterfacePose[k] = false;
    _firstFootOutput[k] = false;
    _desiredFootWrench[k].setConstant(0.0f);
    _FdFoot[k].setConstant(0.0f);
    _normalForceAverage[k] = 0.0f;

    _footTipPosition[k].setConstant(0.0f);
    _footTipOrientation[k].setIdentity();
    _xdFootTip[k].setConstant(0.0f);

  }

  _stop = false;
  // _leftRobotOrigin << 0.066f-0.095, 0.9f, 0.0f;
  _leftRobotOrigin << 0.0f, 0.9f, 0.0f;
  _x0[LEFT](0) = _leftRobotOrigin(0)-0.50f;
  _x0[LEFT](1) = _leftRobotOrigin(1)-0.35f;
  _x0[LEFT](2) = _leftRobotOrigin(2)+0.3f;
  _x0[RIGHT](0) = -0.434f;
  _x0[RIGHT](1) = 0.4f;
  _x0[RIGHT](2) = 0.3f;

  _graspingForceThreshold = 4.0f;  // Grasping force threshold [N]
  _objectGrasped = false;
  _prevObjectGrasped = false;
  _targetForce = 25.0f;
  
  _velocityLimit = 0.4f;
  _kxy = 0.0f;
  _dxy = 0.0f;
  _kphi = 0.0f;
  _dphi = 0.0f;

  _footOffset << 0.0f,0.0f,10.0f,0.0f,0.0f;

  _xDd << 0.0f,-0.3f,0.0f;
  _xDd0 = _xDd;


  Eigen::Matrix<float,5,1> temp;
  temp << FOOT_INTERFACE_X_RANGE/2.0f,FOOT_INTERFACE_Y_RANGE/2.0f,0.0f,0.0f,0.0f;
  _H0 = FootPlatformModel::forwardKinematics(temp);

  _coordinationMode = AUTOMATIC;
  _controlStrategy = SINGLE_FOOT_SINGLE_ARM;
  _prevControlStrategy = _controlStrategy;
  _hapticFeedbackStrategy = MEASURED_ERROR;
  _autonomousForceGeneration = true;
}


bool SharedFourArmManipulation::init() 
{
  if(_useRobot[LEFT])
  {
    // Subscriber definitions
    _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&SharedFourArmManipulation::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/lwr2/ee_vel", 1, boost::bind(&SharedFourArmManipulation::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[LEFT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SharedFourArmManipulation::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Left",1, boost::bind(&SharedFourArmManipulation::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useSim)
    {
      _subForceTorqueSensor[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&SharedFourArmManipulation::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  
    // Publisher definitions
    _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
    _pubFilteredWrench[LEFT] = _nh.advertise<geometry_msgs::WrenchStamped>("SharedFourArmManipulation/filteredWrenchLeft", 1);
    _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Left", 1);
    _pubNormalForce[LEFT] = _nh.advertise<std_msgs::Float32>("SharedFourArmManipulation/normalForceLeft", 1);
  }

  if(_useRobot[RIGHT])
  {
    _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&SharedFourArmManipulation::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/lwr/ee_vel", 1, boost::bind(&SharedFourArmManipulation::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SharedFourArmManipulation::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Right",1, boost::bind(&SharedFourArmManipulation::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
   
    if(!_useSim)
    {
      _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&SharedFourArmManipulation::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  
    _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::WrenchStamped>("SharedFourArmManipulation/filteredWrenchRight", 1);
    _pubNormalForce[RIGHT] = _nh.advertise<std_msgs::Float32>("SharedFourArmManipulation/normalForceRight", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Right", 1);
  }

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&SharedFourArmManipulation::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,SharedFourArmManipulation::stopNode);

  _outputFile.open(ros::package::getPath(std::string("robotic_experiments"))+"/data_foot/"+_filename+".txt");

  if(!_nh.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRobot[RIGHT] && !_useSim)
  {
    ROS_ERROR("[SharedFourArmManipulation]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }
  else if(!_nh.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRobot[RIGHT] && _useSim)
  {
    ROS_ERROR("[SharedFourArmManipulation]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }


  if(!_nh.getParamCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]) && _useRobot[LEFT])
  {
    ROS_ERROR("[SharedFourArmManipulation]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }
  else if(!_nh.getParamCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRobot[LEFT] && _useSim)
  {
    ROS_ERROR("[SharedFourArmManipulation]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[SharedFourArmManipulation]: The SharedFourArmManipulation node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[SharedFourArmManipulation]: The ros node has a problem.");
    return false;
  }
}


void SharedFourArmManipulation::run()
{
  _timeInit = ros::Time::now().toSec();

  while (!_stop) 
  {
    if(allDataReceived())
    {
      _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
      if(_d1[RIGHT] < FLT_EPSILON)
      {
        _d1[RIGHT] = 100.0f;
      }
      if(_d1[LEFT] < FLT_EPSILON)
      {
        _d1[LEFT] = 100.0f;
      }

      ros::param::getCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);


      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  // Send zero velocity command to stop the robot
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k] = _q[k];  
    _desiredFootWrench[k].setConstant(0.0f);  
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  ros::shutdown();
}


void SharedFourArmManipulation::stopNode(int sig)
{
  me->_stop = true;
}


bool SharedFourArmManipulation::allDataReceived()
{
  // Check if we receive data from all subscribers
  bool robotStatus[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatus[k] = !_useRobot[k] || (_firstRobotPose[k] && _firstRobotTwist[k]
                      && _firstDampingMatrix[k] && _firstFootOutput[k]
                      && (_useSim || _wrenchBiasOK[k]));

    if(!robotStatus[k])
    {
      std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
      << " p: " << _firstRobotPose[k] << " t: " << _firstRobotTwist[k]
      << " d: " << _firstDampingMatrix[k] << " foot: " << _firstFootOutput[k]
      << " sim/wrench: " << (_useSim || _wrenchBiasOK[k]) << std::endl;
    }
  }
  return robotStatus[LEFT] && robotStatus[RIGHT];
}


void SharedFourArmManipulation::computeCommand()
{
  footDataTransformation();

  footPositionMapping();

  if(!_useSim)
  {
    updateObjectGraspingState();
  }

  switch(_coordinationMode)
  {
    case NO_COORDINATION:
    {
      _controlStrategy = SINGLE_FOOT_SINGLE_ARM;
      break;
    }
    case STRICTLY_COORDINATED:
    {
      _controlStrategy = SINGLE_FOOT_DUAL_ARM;
      break;
    }
    case AUTOMATIC:
    {
      updateControlStrategy();
      break;
    }
    default:
    {
      break;
    }
  }

  switch(_controlStrategy)
  {
    case SINGLE_FOOT_SINGLE_ARM:
    {
      singleFootSingleArmControl();
      break;
    }
    case SINGLE_FOOT_DUAL_ARM:
    {
      singleFootDualArmControl();
      break;
    }
    default:
    {
      _vd[LEFT].setConstant(0.0f);  
      _vd[RIGHT].setConstant(0.0f);  

      break;
    }
  }

  computeHapticFeedback();

  computeDesiredOrientation();

  if(!_useSim)
  {
    computeDesiredFootWrench();
  }
}


void SharedFourArmManipulation::footDataTransformation()
{
  // Foot tip position and orientation
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Matrix<float,5,1> offset;
    offset << FOOT_INTERFACE_X_RANGE/2.0f,FOOT_INTERFACE_Y_RANGE/2.0f,0.0f,0.0f,0.0f;
    Eigen::Matrix4f H;
    H = FootPlatformModel::forwardKinematics(_footPose[k]+offset);
    _footTipPosition[k] = H.block(0,3,3,1);
    _footTipOrientation[k] = H.block(0,0,3,3);
  }

  // Foot center position
  Eigen::Matrix3f R;
  R << 0.0f, 1.0f, 0.0f,
       -1.0f,0.0f,0.0f,
       0.0f,0.0f,1.0f;

  _footPosition[RIGHT] = R*_footPose[RIGHT].segment(0,3);
  _footPosition[LEFT] = R*_footPose[LEFT].segment(0,3);
}


void SharedFourArmManipulation::footPositionMapping()
{
  // Map foot tip position
  Eigen::Vector3f gain;

  Eigen::Matrix3f R;
  R << 0.0f, 1.0f, 0.0f,
     -1.0f,0.0f,0.0f,
     0.0f,0.0f,1.0f;

  gain << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE, 2*_zPositionMapping/FOOT_INTERFACE_Z_RANGE;
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xdFootTip[k] = gain.cwiseProduct(R*(_footTipPosition[k]-_H0.block(0,3,3,1)));
    if(_xdFootTip[k](2)+_x0[k](2)<SAFETY_Z)
    {
      _xdFootTip[k](2) = SAFETY_Z-_x0[k](2);
    }
  }

  // Map foot center position
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE, 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE, 2*_zPositionMapping/FOOT_INTERFACE_PITCH_RANGE;
  gains[LEFT] << 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE, 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE, 2*_zPositionMapping/FOOT_INTERFACE_PITCH_RANGE;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xdFoot[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_xdFoot[k](2)+_x0[k](2)<SAFETY_Z)
    {
      _xdFoot[k](2) = SAFETY_Z-_x0[k](2);
    }
  }
}


void SharedFourArmManipulation::updateObjectGraspingState()
{
  Eigen::Vector3f temp;
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _normalForce[k] = (_wRb[k]*_filteredWrench[k].segment(0,3)).dot(-_wRb[k].col(2));
    if(_normalForceWindow[k].size()<WINDOW_SIZE)
    {
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
    }
    else
    {
      _normalForceWindow[k].pop_front();
      _normalForceWindow[k].push_back(_normalForce[k]);
      _normalForceAverage[k] = 0.0f;
      for(int m = 0; m < WINDOW_SIZE; m++)
      {
        _normalForceAverage[k]+=_normalForceWindow[k][m];
      }
      _normalForceAverage[k] /= WINDOW_SIZE;
    }
  }

  _prevObjectGrasped = _objectGrasped;

  if(_normalForceAverage[LEFT] > _graspingForceThreshold && _normalForceAverage[RIGHT] > _graspingForceThreshold)
  {
    _objectGrasped = true;
  }
  else
  {
    _objectGrasped = false;
  }
  std::cerr << "Object grapsed: " <<(int) _objectGrasped << std::endl;
}


void SharedFourArmManipulation::updateControlStrategy()
{
  _prevControlStrategy = _controlStrategy;

  if(_controlStrategy == SINGLE_FOOT_SINGLE_ARM)
  {
    if(_objectGrasped)
    {
      _controlStrategy = SINGLE_FOOT_DUAL_ARM;
    }
  }
  else
  {
    if(!_objectGrasped && _xD.norm() > _xDd0.norm()+0.2f)
    // if(!_objectGrasped)
    {
      _controlStrategy = SINGLE_FOOT_SINGLE_ARM;
    }
  }

  std::cerr << "control strategy: " << (int) _controlStrategy << std::endl;
}


void SharedFourArmManipulation::singleFootSingleArmControl()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(!_useSim && _objectGrasped && _autonomousForceGeneration)
    {
      _Fd[k] = _targetForce;
    }
    else
    {
      _Fd[k] = 0.0f;
    }

    if(_coordinationMode == AUTOMATIC && _prevControlStrategy == SINGLE_FOOT_DUAL_ARM)
    {
      _waitForFoot[k] = true;
    }

    if(_waitForFoot[k])
    {
      if((_xdFoot[k]+_x0[k]-_x[k]).norm()< 0.07f)
      {
        _waitForFoot[k] = false;
      }

      std::cerr << "wait: " << (int) _waitForFoot[k] << " x[k]: " << _x[k].transpose() << " xd[k]: " << (_xdFoot[k]+_x0[k]).transpose() << std::endl;
      _vd[k].setConstant(0.0f);

    }
    else
    {
      _xd[k] = _xdFoot[k]+_x0[k];
      _vd[k] = 2.0f*(_xd[k]-_x[k])+(_Fd[k]/_d1[k])*_wRb[k].col(2);      
    }

    _vd[k] = Utils<float>::bound(_vd[k],0.3f);
  }
}


void SharedFourArmManipulation::singleFootDualArmControl()
{
  // Compute dual arm state
  _xC = (_x[LEFT]+_x[RIGHT])/2.0f;
  _xD = (_x[RIGHT]-_x[LEFT]);

  // The desired center is defined by the right foot
  _xCd = _xdFoot[RIGHT]+_x0[RIGHT];

  //  Compute initial distance vector after first contact
  if(_objectGrasped && !_prevObjectGrasped)
  {
    _xDd0 = _x[RIGHT]-_x[LEFT];
    _xDd0(0) = 0.0f;
    _xDd0(2) = 0.0f;
    _xDd = _xDd0;
  }

  float d = _xDd.norm();
  d+=_dt*Utils<float>::deadZone(-_footPose[RIGHT](YAW),-5.0f,5.0f)*M_PI/180.0f;
  // d+=-_dt*_footPose[RIGHT](YAW)*M_PI/180.0f;

  if(_objectGrasped)
  {
    d = Utils<float>::bound(d,_xDd0.norm(),XD_MAX_NORM);
    d = _xDd0.norm();
  }
  else
  {
    d = Utils<float>::bound(d,XD_MIN_NORM,XD_MAX_NORM);    
  }

  // Compute a rotation matrix
  static float phi = 0.0f;
  static float theta = 0.0f;
  static float psi = 0.0f;    

  // phi+= _dt*Utils<float>::deadZone(_footPose[RIGHT](ROLL),-4.0f,4.0f)*M_PI/180.0f;
  // phi = Utils<float>::bound(phi,-M_PI/4.0f,M_PI/4.0f);
  
  Eigen::Matrix3f Rtemp;
  Rtemp = Utils<float>::eulerAnglesToRotationMatrix(phi,theta,psi);  

  // Compute desired distance vector between the two arms
  _xDd = d*Rtemp*_xDd0.normalized();
  // _xDd = 1.0f*Rtemp*_xDd0.normalized();

  std::cerr << "Distance: " << d << " phi: " << phi <<std::endl;
  // std::cerr << "xDd0: "<< _xDd0.transpose() << std::endl;


  if(!_useSim && _objectGrasped && _autonomousForceGeneration)
  {
    _Fd[RIGHT] = _targetForce;
    _Fd[LEFT] = _targetForce;
  }
  else
  {
    _Fd[RIGHT] = 0.0f;
    _Fd[LEFT] = 0.0f;
  }

  _Fdh[RIGHT] = _Fd[RIGHT]*Utils<float>::smoothRise(_footPose[RIGHT](YAW),-20.0f,-1.0f);
    std::cerr << "Force human: " <<  _Fdh[RIGHT] << std::endl;
  _Fdh[LEFT] = _Fd[LEFT]*Utils<float>::smoothRise(_footPose[RIGHT](YAW),-20.0f,-1.0f);

  _xd[RIGHT] = _xCd+_xDd/2.0f;
  _xd[LEFT] = _xCd-_xDd/2.0f;

  Eigen::Vector3f n[NB_ROBOTS];
  n[LEFT] = _xDd0.normalized();
  n[RIGHT] = -_xDd0.normalized();

  for(int k = 0; k <NB_ROBOTS; k++)
  {
    _vd[k] = 2.0f*(_xd[k]-_x[k])+(_Fdh[k]/_d1[k])*n[k];
    _vd[k] = Utils<float>::bound(_vd[k],0.3f);
  }


  Eigen::Vector3f vC, vD;
  vC = 4.0f*(_xCd-_xC);
  if(_objectGrasped)
  {
    _xDd = _xDd0;
    vD = (Rtemp*_xDd0-_xD);
  }
  else
  {
    _xDd = _xD;
    _xDd(0) = 0.0f;
    _xDd(2) = 0.0f;
    vD = 2.0f*(_xDd-_xD);
  }

  Eigen::Vector3f vH[NB_ROBOTS];

  vH[RIGHT] = 0.2f*(Utils<float>::deadZone(-_footPose[RIGHT](YAW),-5.0f,5.0f)/20.f)*(Rtemp*_xDd0.normalized());
  vH[LEFT] = -0.2f*(Utils<float>::deadZone(-_footPose[RIGHT](YAW),-5.0f,5.0f)/20.f)*(Rtemp*_xDd0.normalized());


  float FdiffRight = _d1[RIGHT]*vC.dot(n[RIGHT])+_Fd[RIGHT];
  float FdiffLeft = _d1[LEFT]*vC.dot(n[LEFT])+_Fd[LEFT];
  float alphaR = Utils<float>::smoothRise(FdiffRight,0.0f,10.0f);
  float alphaL = Utils<float>::smoothRise(FdiffLeft,0.0f,10.0f);


  // float FdiffRight = _Fd[RIGHT]-_normalForce[RIGHT];
  // float FdiffLeft = _Fd[LEFT]-_normalForce[LEFT];
  // float alphaR = Utils<float>::smoothFall(FdiffRight,0.0f,10.0f);
  // float alphaL = Utils<float>::smoothFall(FdiffLeft,0.0f,10.0f);


  std::cerr << "Fdiff R: " << FdiffRight << " alphaR: " << alphaR << std::endl;
  std::cerr << "Fdiff L: " << FdiffLeft << " alphaL: " << alphaL << std::endl;
  // std::cerr << "gain: " << (4.0f-alphaR-alphaL)/2.0f << std::endl;
  // std::cerr << "gain: " << (3.0f-alphaR-alphaL) << std::endl;

  // _vd[RIGHT] = 0.5*(alphaL+alphaR)*vC+vD/2.0f+vH[RIGHT]+(_Fd[RIGHT]/_d1[RIGHT])*n[RIGHT];
  // _vd[LEFT] = 0.5*(alphaL+alphaR)*vC-vD/2.0f+vH[LEFT]+(_Fd[LEFT]/_d1[LEFT])*n[LEFT];

  // _vd[RIGHT] = vC+vD/2.0f+vH[RIGHT]+((4.0f-alphaR-alphaL)/2.0f)*(_Fd[RIGHT]/_d1[RIGHT])*n[RIGHT];
  // _vd[LEFT] = vC-vD/2.0f+vH[LEFT]+((4.0f-alphaR-alphaL)/2.0f)*(_Fd[LEFT]/_d1[LEFT])*n[LEFT];

  // _vd[RIGHT] = vC+vD/2.0f+vH[RIGHT]+(3.0f-alphaR-alphaL)*(_Fd[RIGHT]/_d1[RIGHT])*n[RIGHT];
  // _vd[LEFT] = vC-vD/2.0f+vH[LEFT]+(3.0f-alphaR-alphaL)*(_Fd[LEFT]/_d1[LEFT])*n[LEFT];

  _vd[RIGHT] = vC+vD/2.0f+vH[RIGHT]+(-alphaL+2.0f)*(_Fd[RIGHT]/_d1[RIGHT])*n[RIGHT];
  _vd[LEFT] = vC-vD/2.0f+vH[LEFT]+(-alphaR+2.0f)*(_Fd[LEFT]/_d1[LEFT])*n[LEFT];


  std::cerr << "vd R: " << _vd[RIGHT].transpose() << std::endl;
  std::cerr << "vd L: " << _vd[LEFT].transpose() << std::endl;
  for(int k = 0; k <NB_ROBOTS; k++)
  {
    _vd[k] = Utils<float>::bound(_vd[k],0.4f);
  }

}


void SharedFourArmManipulation::computeHapticFeedback()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    switch(_hapticFeedbackStrategy)
    {
      case NO_FEEDBACK:
      {
        _FdFoot[k].setConstant(0.0f);
        break;
      }
      case MEASURED_FORCE:
      {
        _FdFoot[k] = _wRb[k]*_filteredWrench[k].segment(0,3);
        break;
      }
      case MEASURED_ERROR:
      {
        // if(_controlStrategy==SINGLE_FOOT_SINGLE_ARM)
        // {
          _FdFoot[k] = _wRb[k]*_filteredWrench[k].segment(0,3)+_Fd[k]*_wRb[k].col(2);
          if(_waitForFoot[k])
          {
            // _FdFoot[k] += 150.0f*(_x[k]-_xdFoot[k]-_x0[k]);
            _FdFoot[k] += Utils<float>::bound(200.0f*(_x[k]-_xdFoot[k]-_x0[k]),15.0f);
          }
        // }
        // else
        // {
        //   _FdFoot[k] = (_Fd[k]-_Fdh[k])*_wRb[k].col(2);
        // }
        break;
      }
      default:
      {
        _FdFoot[k].setConstant(0.0f);
        break;
      }
    }
  }
}


void SharedFourArmManipulation::computeDesiredFootWrench()
{
  // temp.setConstant(0.0f);
  _desiredFootWrench[LEFT].setConstant(0.0f);
  _desiredFootWrench[RIGHT].setConstant(0.0f);

  if(_controlStrategy==SINGLE_FOOT_DUAL_ARM)
  {
    _desiredFootWrench[RIGHT](YAW) = _FdFoot[RIGHT](1)*0.2f;
    std::cerr << "Desired force: " << _FdFoot[RIGHT](1)*0.2f << std::endl;
  }
  else
  {
    _desiredFootWrench[RIGHT](1) = _FdFoot[RIGHT](0);
    _desiredFootWrench[RIGHT](0) = -_FdFoot[RIGHT](1);
    _desiredFootWrench[RIGHT](2) = _FdFoot[RIGHT](2)*0.2;
    _desiredFootWrench[LEFT](1) = _FdFoot[LEFT](0);
    _desiredFootWrench[LEFT](0) = -_FdFoot[LEFT](1);
    _desiredFootWrench[LEFT](2) = _FdFoot[LEFT](2)*0.2;
  }

  // _desiredFootWrench[RIGHT](0) += -_kxy*_footPose[RIGHT](0)-_dxy*_footTwist[RIGHT](0);
  // _desiredFootWrench[RIGHT](1) += -_kxy*_footPose[RIGHT](1)-_dxy*_footTwist[RIGHT](1);
  // _desiredFootWrench[RIGHT](2) += -_kphi*_footPose[RIGHT](2)-_dphi*_footTwist[RIGHT](2);
  // _desiredFootWrench[RIGHT](3) += -_kphi*_footPose[RIGHT](3)-_dphi*_footTwist[RIGHT](3);
  // _desiredFootWrench[RIGHT](4) += -_kphi*_footPose[RIGHT](4)-_dphi*_footTwist[RIGHT](4);

  // _desiredFootWrench[LEFT](0) += -_kxy*_footPose[LEFT](0)-_dxy*_footTwist[LEFT](0);
  // _desiredFootWrench[LEFT](1) += -_kxy*_footPose[LEFT](1)-_dxy*_footTwist[LEFT](1);
  // _desiredFootWrench[LEFT](2) += -_kphi*_footPose[LEFT](2)-_dphi*_footTwist[LEFT](2);
  // _desiredFootWrench[LEFT](3) += -_kphi*_footPose[LEFT](3)-_dphi*_footTwist[LEFT](3);
  // _desiredFootWrench[LEFT](4) += -_kphi*_footPose[LEFT](4)-_dphi*_footTwist[LEFT](4);

  // _desiredFootWrench[LEFT].setConstant(0.0f);
  // _desiredFootWrench[RIGHT].setConstant(0.0f);

  for(int k = 0; k < 2; k++)
  {
    if(_desiredFootWrench[RIGHT](k)>25.0f)
    {
      _desiredFootWrench[RIGHT](k) = 25.0f;
    }
    else if(_desiredFootWrench[RIGHT](k)<-25.0f)
    {
      _desiredFootWrench[RIGHT](k) = -25.0f;
    }

    if(_desiredFootWrench[LEFT](k)>25.0f)
    {
      _desiredFootWrench[LEFT](k) = 25.0f;
    }
    else if(_desiredFootWrench[LEFT](k)<-25.0f)
    {
      _desiredFootWrench[LEFT](k) = -25.0f;
    }
  }

  for(int k = 0 ; k < 3; k++)
  {
    if(_desiredFootWrench[RIGHT](k+2)>5.0f)
    {
      _desiredFootWrench[RIGHT](k+2) = 5.0f;
    }
    else if(_desiredFootWrench[RIGHT](k+2)<-5.0f)
    {
      _desiredFootWrench[RIGHT](k+2) = -5.0f;
    }

    if(_desiredFootWrench[LEFT](k+2)>5.0f)
    {
      _desiredFootWrench[LEFT](k+2) = 5.0f;
    }
    else if(_desiredFootWrench[LEFT](k+2)<-5.0f)
    {
      _desiredFootWrench[LEFT](k+2) = -5.0f;
    }
  }
}


void SharedFourArmManipulation::computeDesiredOrientation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {

    if(_controlStrategy == SINGLE_FOOT_SINGLE_ARM)
    {
      ////////////////////////////////////
      // With full orientation matrices //
      ////////////////////////////////////


      // Eigen::Matrix3f Rd;
      // if(k == RIGHT)
      // {
      //   Rd << -1.0f, 0.0f, 0.0f,
      //         0.0f, 0.0f, 1.0f,
      //         0.0f, 1.0f, 0.0f;
      // }
      // else
      // {
      //   Rd << -1.0f, 0.0f, 0.0f,
      //         0.0f, 0.0f, -1.0f,
      //         0.0f, -1.0f, 0.0f;
      // }
      
      // Eigen::Matrix3f Rtemp;
      // float phi = -_rollGain*_footPose[k](ROLL)*M_PI/180.0f;
      // float theta;
      // if(k==RIGHT)
      // {
      //   theta = _yawGain*_footPose[k](YAW)*M_PI/180.0f;
      // }
      // else
      // {
      //   theta = -_yawGain*_footPose[k](YAW)*M_PI/180.0f;
      // }
      // float psi = -_yawGain*_footPose[k](YAW)*M_PI/180.0f;
      // // phi = 0.0f;
      // theta = 0.0f;
      // psi = 0.0f; 

      // Rtemp = Utils<float>::eulerAnglesToRotationMatrix(phi,theta,psi);  
      // Rd = Rd*Rtemp; 
 
      // _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

    //////////////
    // With tip //
    //////////////
      
    // Eigen::Matrix3f Re = (_H0.block(0,0,3,3).inverse()*_footTipOrientation[k]);
    // Eigen::Vector4f qe;
    // qe = Utils<float>::rotationMatrixToQuaternion(Re);
    // Eigen::Vector3f axis;
    // float angle;
    // Utils<float>::quaternionToAxisAngle(qe,axis,angle);
    // angle *= _rollGain;
    // qe = Utils<float>::axisAngleToQuaterion(axis,angle);
    // Rd = Utils<float>::quaternionToRotationMatrix(qe)*Rd;
   
    // Rd = (_H0.block(0,0,3,3).inverse()*_footTipOrientation[k])*Rd;

      Eigen::Vector3f temp;
      temp << 0.0f, -1.0f, 0.0f;
      Eigen::Vector4f qe;
      if(k== LEFT)
      {
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[k].col(2),temp));
      }
      else
      {
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[k].col(2),-temp));        
      }

      Eigen::Vector3f omega;  
      float angle;
      Utils<float>::quaternionToAxisAngle(qe,omega,angle);
      std::cerr << "angle: "  << k << ": " << angle << std::endl;

      // // Compute final quaternion on plane
      _qd[k] = Utils<float>::quaternionProduct(qe,_q[k]);
   
    }
    else
    {
      Eigen::Vector4f qe;
      if(k== LEFT)
      {
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[k].col(2),_xDd.normalized()));
      }
      else
      {
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[k].col(2),-_xDd.normalized()));        
      }

      Eigen::Vector3f omega;  
      float angle;
      Utils<float>::quaternionToAxisAngle(qe,omega,angle);
      std::cerr << "angle: "  << k << ": " << angle << std::endl;

      // // Compute final quaternion on plane
      _qd[k] = Utils<float>::quaternionProduct(qe,_q[k]);
    }

    if(_q[k].dot(_qd[k])<0)
    {
      _qd[k] *=-1.0f;
    }

    Eigen::Vector4f qe, qinv;
    qinv = _q[k];
    qinv.segment(1,3) *=-1.0f;

    qe = Utils<float>::quaternionProduct(_qd[k],qinv);
    Eigen::Vector3f axis;
    float angle;
    Utils<float>::quaternionToAxisAngle(qe,axis,angle);
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angle,-0.2f,0.2f));
    _qd[k] = Utils<float>::quaternionProduct(qe,_q[k]);

    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 

    if(_controlStrategy==SINGLE_FOOT_DUAL_ARM)
    {    
      // Utils<float>::deadZone(_footPose[RIGHT](ROLL),-7.0f,7.0f)
      float selfRotationCommand = 2.0f*1.0f*Utils<float>::deadZone(_footPose[RIGHT](ROLL),-7.0f,7.0f)/FOOT_INTERFACE_YAW_RANGE;
      std::cerr << "Self rotation: " << k << " :" << selfRotationCommand << std::endl;
      _omegad[k] += -selfRotationCommand*_xDd.normalized();
    }
    else
    {
      float selfRotationCommand = 2.0f*1.0f*Utils<float>::deadZone(_footPose[k](ROLL),-7.0f,7.0f)/FOOT_INTERFACE_YAW_RANGE;
      _omegad[k] += selfRotationCommand*_wRb[k].col(2);
    }
  }
}


void SharedFourArmManipulation::logData()
{
  // _outputFile << ros::Time::now() << " "
  //             << _x[LEFT].transpose() << " "
  //             << _vd[LEFT].transpose() << " "
  //             << (_wRb[LEFT]*_filteredWrench[LEFT].segment(0,3)).transpose() << " "
  //             << (_wRb[LEFT]*_filteredWrench[LEFT].segment(3,3)).transpose() << " "
  //             << _x[RIGHT].transpose() << " "
  //             << _vd[RIGHT].transpose() << " "
  //             << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(0,3)).transpose() << " "
  //             << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(3,3)).transpose() << " "
  //             << _footPose[LEFT](0) << " "
  //             << _footPose[LEFT](1) << " "
  //             << _footPose[LEFT](2) << " "
  //             << _footWrench[LEFT](0) << " "
  //             << _footWrench[LEFT](1) << " "
  //             << _footWrench[LEFT](2) << " "
  //             << _desiredFootWrench[LEFT](0) << " "
  //             << _desiredFootWrench[LEFT](1) << " "
  //             << _desiredFootWrench[LEFT](2) << " "
  //             << _footState[LEFT] << " "
  //             << _footPose[RIGHT](0) << " "
  //             << _footPose[RIGHT](1) << " "
  //             << _footPose[RIGHT](2) << " "
  //             << _footWrench[RIGHT](0) << " "
  //             << _footWrench[RIGHT](1) << " "
  //             << _footWrench[RIGHT](2) << " "
  //             << _desiredFootWrench[RIGHT](0) << " "
  //             << _desiredFootWrench[RIGHT](1) << " "
  //             << _desiredFootWrench[RIGHT](2) << " "
  //             << _footState[RIGHT] << std::endl;


}


void SharedFourArmManipulation::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Publish desired twist (passive ds controller)
    _msgDesiredTwist.linear.x  = _vd[k](0);
    _msgDesiredTwist.linear.y  = _vd[k](1);
    _msgDesiredTwist.linear.z  = _vd[k](2);

    // Convert desired end effector frame angular velocity to world frame
    _msgDesiredTwist.angular.x = _omegad[k](0);
    _msgDesiredTwist.angular.y = _omegad[k](1);
    _msgDesiredTwist.angular.z = _omegad[k](2);

    _pubDesiredTwist[k].publish(_msgDesiredTwist);

    // Publish desired orientation
    _msgDesiredOrientation.w = _qd[k](0);
    _msgDesiredOrientation.x = _qd[k](1);
    _msgDesiredOrientation.y = _qd[k](2);
    _msgDesiredOrientation.z = _qd[k](3);

    _pubDesiredOrientation[k].publish(_msgDesiredOrientation);

    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _filteredWrench[k](0);
    _msgFilteredWrench.wrench.force.y = _filteredWrench[k](1);
    _msgFilteredWrench.wrench.force.z = _filteredWrench[k](2);
    _msgFilteredWrench.wrench.torque.x = _filteredWrench[k](3);
    _msgFilteredWrench.wrench.torque.y = _filteredWrench[k](4);
    _msgFilteredWrench.wrench.torque.z = _filteredWrench[k](5);
    _pubFilteredWrench[k].publish(_msgFilteredWrench);

    std_msgs::Float32 msg;
    msg.data = _normalForce[k];
    _pubNormalForce[k].publish(msg); 


    for(int m = 0; m < 5; m++)
    {
      _msgFootInput.ros_effort[m] = _desiredFootWrench[k](m);
    }
    _pubFootInput[k].publish(_msgFootInput);
  }
}


void SharedFourArmManipulation::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffsetFromEE[k]*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _x[k] += _leftRobotOrigin;
  }

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    // _x0[k] = _x[k];
    _qd[k] = _q[k];
    _vd[k].setConstant(0.0f);
  }
}


void SharedFourArmManipulation::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void SharedFourArmManipulation::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrenchBias[k].segment(0,3) -= loadForce;
    _wrenchBias[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _wrenchBias[k] += raw; 
    _wrenchCount[k]++;
    if(_wrenchCount[k]==NB_SAMPLES)
    {
      _wrenchBias[k] /= NB_SAMPLES;
      _wrenchBiasOK[k] = true;
      std::cerr << "[SharedFourArmManipulation]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK[k] && _firstRobotPose[k])
  {
    _wrench[k] = raw-_wrenchBias[k];
    Eigen::Vector3f loadForce = _wRb[k].transpose()*_toolMass*_gravity;
    _wrench[k].segment(0,3) -= loadForce;
    _wrench[k].segment(3,3) -= _toolComPositionFromSensor.cross(loadForce);
    _filteredWrench[k] = _filteredForceGain*_filteredWrench[k]+(1.0f-_filteredForceGain)*_wrench[k];
  }
}


void SharedFourArmManipulation::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void SharedFourArmManipulation::updateFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg, int k)
{

  for(int m = 0; m < 5; m++)
  {
    _footPose[k](m) = msg->platform_position[m];
    _footTwist[k](m) = msg->platform_speed[m];
    _footWrench[k](m) = msg->platform_effortD[m];
  }
  // _footPose[k] -= _footOffset;
  _footState[k] = msg->platform_machineState;

  if(!_firstFootOutput[k])
  {
    _firstFootOutput[k] = true;
  }
}


void SharedFourArmManipulation::dynamicReconfigureCallback(robotic_experiments::feetTelemanipulation_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _filteredForceGain = config.filteredForceGain;
  _velocityLimit = config.velocityLimit;
  _kxy = config.kxy;
  _dxy = config.dxy;
  _kphi = config.kphi;
  _dphi = config.dphi;
  _xyPositionMapping = config.xyPositionMapping;
  _zPositionMapping = config.zPositionMapping;
  _rollGain = config.rollGain;
  _yawGain = config.yawGain;
  _useSharedControl = config.useSharedControl;
}