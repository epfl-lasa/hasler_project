#include "KukaDemo.h"
#include "Utils.h"


KukaDemo* KukaDemo::me = NULL;

KukaDemo::KukaDemo(ros::NodeHandle &n, double frequency, std::string filename):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename)
{
  me = this;
  
  _stop = false;
  _useRobot[LEFT] = true;
  _useRobot[RIGHT] = true;
  _useSim = false;
  _useJoystick = false;
  _useCustomTrocars = true;
  _useSphericalTrocars = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE[LEFT] = 0.422f+0.015f;
  _toolOffsetFromEE[RIGHT] = 0.43f+0.015f;
  // _toolOffsetFromEE[LEFT] = 0.41f;
  // _toolOffsetFromEE[RIGHT] = 0.41f;
  _toolMass = 0.2f;
  
  // _leftRobotOrigin << 0.07f, 1.083f, 0.0f;
  _leftRobotOrigin << 0.0f, 1.04f, 0.0f;

  _footOffset[LEFT].setConstant(0.0f);
  _footOffset[RIGHT].setConstant(0.0f);
  _footOffset[RIGHT](2) = 10.0f;

  _footMode[LEFT] = VELOCITY;
  _footMode[RIGHT] = POSITION;

  _adaptationRate = 50.0f;
  _joyOffsetPitch = 0.5f;

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _xEE[k].setConstant(0.0f);
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
    _Fd[k] = 0.0f;

    _trocarsRegistered[k] = false;
    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _wrenchBiasOK[k] = false;
    _firstDampingMatrix[k] = true;
    _firstJointsUpdate[k] = false;
    _firstJoystick[k] = false;

    _wrenchCount[k] = 0;
    _d1[k] = 1.0f;

    _footPose[k].setConstant(0.0f);
    _footWrench[k].setConstant(0.0f);
    _footTwist[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _xdFoot[k].setConstant(0.0f);
    _vdFoot[k].setConstant(0.0f);
    _footInterfaceSequenceID[k] = 0;    
    _firstFootOutput[k] = false;
    _desiredFootWrench[k].setConstant(0.0f);
    _FdFoot[k].setConstant(0.0f);
    _normalForceAverage[k] = 0.0f;
    _nullspaceWrench[k].setConstant(0.0f);
    _alignedWithTrocar[k] = false;
    _nullspaceCommand[k].setConstant(0.0f);
    _inputAlignedWithOrigin[k] = false;

  }

  if(_useCustomTrocars)
  {
    initializeCustomTrocars();
    _trocarsRegistered[LEFT] = true;
    _trocarsRegistered[RIGHT] = true;
  }
}


bool KukaDemo::init() 
{
  if(_useRobot[LEFT])
  {
    // Subscriber definitions
    _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&KukaDemo::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/lwr2/ee_vel", 1, boost::bind(&KukaDemo::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subCurrentJoints[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/lwr2/joint_states", 1, boost::bind(&KukaDemo::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[LEFT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&KukaDemo::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useSim)
    {
      _subForceTorqueSensor[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&KukaDemo::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  
    if(_useJoystick)
    {
      _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&KukaDemo::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Left",1, boost::bind(&KukaDemo::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    // Publisher definitions
    _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[LEFT] = _nh.advertise<geometry_msgs::Wrench>("/lwr2/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[LEFT] = _nh.advertise<geometry_msgs::WrenchStamped>("KukaDemo/filteredWrenchLeft", 1);
    _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Left", 1);
    _pubNullspaceCommand[LEFT] = _nh.advertise<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_command_nullspace", 1);
  }

  if(_useRobot[RIGHT])
  {
    _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&KukaDemo::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/lwr/ee_vel", 1, boost::bind(&KukaDemo::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subCurrentJoints[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/lwr/joint_states", 1, boost::bind(&KukaDemo::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&KukaDemo::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
   
    if(!_useSim)
    {
      _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&KukaDemo::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
   
    if(_useJoystick)
    {
      _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&KukaDemo::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Right",1, boost::bind(&KukaDemo::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::WrenchStamped>("KukaDemo/filteredWrenchRight", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Right", 1);
    _pubNullspaceCommand[RIGHT] = _nh.advertise<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_command_nullspace", 1);
  }

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&KukaDemo::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,KukaDemo::stopNode);

  _outputFile.open(ros::package::getPath(std::string("robotic_experiments"))+"/data_demo/"+_filename+".txt");

  if(!_nh.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRobot[RIGHT])
  {
    ROS_ERROR("[KukaDemo]: Cannot read first eigen value of passive ds controller for right robot");
    // return false;
  }

  if(!_nh.getParamCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]) && _useRobot[LEFT])
  {
    ROS_ERROR("[KukaDemo]: Cannot read first eigen value of passive ds controller for left robot");
    // return false;
  }

  if(_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[KukaDemo]: The KukaDemo node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[KukaDemo]: The ros node has a problem.");
    return false;
  }
}


void KukaDemo::run()
{
  while (!_stop) 
  {
    if(allDataReceived())
    {
      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
      ros::param::getCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);
          
      if(_trocarsRegistered[LEFT] && _trocarsRegistered[RIGHT])
      {
        // Compute control command
        computeCommand();

        // Publish data to topics
        publishData();

        // Log data
        logData();
      }
      else
      {
        registerTrocars();
      }      
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


void KukaDemo::stopNode(int sig)
{
  me->_stop = true;
}


bool KukaDemo::allDataReceived()
{
  // Check if we receive data from all subscribers

  bool robotStatus[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatus[k] = !_useRobot[k] || (_firstRobotPose[k] && _firstRobotTwist[k]
                      && _firstDampingMatrix[k] && _firstJointsUpdate[k] 
                      && (_firstFootOutput[k] || _firstJoystick[k])
                      && (_useSim || _wrenchBiasOK[k]));

    if(!robotStatus[k])
    {
      std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
      << " p: " << _firstRobotPose[k] << " t: " << _firstRobotTwist[k]
      << " d: " << _firstDampingMatrix[k] << " j: " << _firstJointsUpdate[k]
      << " foot/joy: " << (_firstFootOutput[k] || _firstJoystick[k])
      << " sim/wrench: " << (_useSim || _wrenchBiasOK[k]) << std::endl;
    }
  }
  
  return robotStatus[LEFT] && robotStatus[RIGHT];
}


void KukaDemo::computeCommand()
{
  footDataTransformation();

  for(int r = 0; r <NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {
      updateTrocarInformation(r);
      
      selectRobotMode(r);
      
      switch(_robotMode[r])
      {
        case TROCAR_SELECTION:
        {
          trocarSelection(r);
          break;
        }
        case TROCAR_INSERTION:
        {
          trocarInsertion(r);
          break;
        }
        case TROCAR_SPACE:
        {
          trocarSpace(r);
          break;
        }
        default:
        {
          break;
        }
      }
    }
  }

  if(!_useSim && !_useJoystick)
  {
    computeDesiredFootWrench();
  }
}


void KukaDemo::updateTrocarInformation(int r)
{
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    _rEETrocar[r][k] = _trocarPosition[r][k]-_xEE[r];
    _xRCM[r][k] = _xEE[r]+(_trocarPosition[r][k]-_xEE[r]).dot(_wRb[r].col(2))*_wRb[r].col(2);
    _rEERCM[r][k] = _xRCM[r][k]-_xEE[r];

    Eigen::Vector3f n;
    n << 0.0f,0.0f,1.0f;

    // Compute attractor that is aligned with the desired trocar orientation
    // float s = -n.dot(_xEE[r]-_trocarPosition[k])/(n.dot(_trocarOrientation[k]));
    // _xdEE[r][k] = _trocarPosition[k]-_trocarOrientation[k]*s;
    // _xdEE[r][k] = _trocarPosition[r][k]-_trocarOrientation[r][k]*(_toolOffsetFromEE[r]+0.15f);
    // _xdEE[r][k] = _trocarPosition[r][k]+std::max(_rEETrocar[r][k].dot(_trocarOrientation[r][k]),_toolOffsetFromEE[r]+0.05f)*(-_trocarOrientation[r][k]);
    _xdEE[r][k] = _trocarPosition[r][k]-_rEETrocar[r][k].dot(_trocarOrientation[r][k])*_trocarOrientation[r][k];


    // Linear DS to go to the attractor
    _fxk[r][k]= 4.0f*(_xdEE[r][k]-_xEE[r]);
  }

  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);

  if(std::fabs(1.0f-bmax)<FLT_EPSILON && _robotMode[r] == TROCAR_INSERTION)
  {
    if((Utils<float>::orthogonalProjector(_rEETrocar[r][indexMax].normalized())*_filteredWrench[r].segment(0,3)).norm() > 30.0f)
    {
      ROS_INFO("TROCAR UPDATE");
      _trocarPosition[r][indexMax] = _x[r];
    }
  }     
}

void KukaDemo::selectRobotMode(int r)
{
  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);

  if(std::fabs(1.0f-bmax)>FLT_EPSILON)
  {
    _robotMode[r] = TROCAR_SELECTION;
  }
  else
  {
    if(_alignedWithTrocar[r]==true)
    {
      float distance = (_trocarPosition[r][indexMax]-_xEE[r]).dot(_wRb[r].col(2))-_toolOffsetFromEE[r];
      std::cerr << r << ": Distance tool-trocar: " <<(_x[r]-_trocarPosition[r][indexMax]).norm() << std::endl;
      std::cerr << r << ": Distance RCM-trocar: " << (_trocarPosition[r][indexMax]-_xRCM[r][indexMax]).norm() <<std::endl;
      if(distance >= 0.04f)
      {
        _robotMode[r] = TROCAR_SELECTION;
      }
      else if(distance >= -0.02f && distance < 0.04f)
      {
        _robotMode[r] = TROCAR_INSERTION;
      }
      else
      {
        _robotMode[r] = TROCAR_SPACE;
      }
    }
  }
}


void KukaDemo::trocarSelection(int r)
{
  std::cerr << r << ": TROCAR_SELECTION" << std::endl;

  if(_nbTrocar[r]>1)
  {
    // Perform task adaptation
    trocarAdaptation(r);  
  }
  else
  {
    _beliefs[r](0) = 1.0f;
    _fx[r] = _fxk[r][0];
  }
  // Find max belief
  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);

  Eigen::Vector3f offset = _sphereCenter;

  if(fabs(bmax-1.0f)>FLT_EPSILON)
  {
    _alignedWithTrocar[r] = false;
    _vd[r] = _fx[r];

    std::cerr << "TROCAR NOT SELECTED" << std::endl;
  }
  else 
  {
    if(!_alignedWithTrocar[r])
    {
      _vd[r] = _fx[r];
    }
    else
    {
      _vd[r] = _fx[r];

      if(!_useJoystick)
      {
        _vd[r]+= -2.0f*0.1*Utils<float>::deadZone(_footPosition[r](2),-5,5)/FOOT_INTERFACE_PITCH_RANGE*(_rEETrocar[r][indexMax].normalized());
      }
      else
      {
        // _vd[r] += 0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*(-_rEETrocar[r][indexMax].normalized());
      }
    }
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.3f);
  
  Eigen::Vector4f qek[_nbTrocar[r]];
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    qek[k] = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][k]));
  }

  Eigen::Vector4f qe;

  if(_nbTrocar[r]>1)
  {
    qe = Utils<float>::slerpQuaternion(qek,_beliefs[r],_nbTrocar[r]);
  }
  else
  {
    qe = qek[0];
  }

  Eigen::Vector3f axis;  
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,axis,angle);

  if(std::fabs(angle)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angle,-MAX_ORIENTATION_ERROR,MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  // std::cerr << "angle: " << angle << "distance: " << _fx[r].norm() << std::endl;
  std::cerr << r << ": Angle error to torcar position: " << angle << std::endl;
  std::cerr << r << ": Angle error to trocar orientation: " << std::acos(_trocarOrientation[r][indexMax].dot(_wRb[r].col(2))) << std::endl;
  std::cerr << r << ": Distance to attractor: " << _fxk[r][indexMax].norm()/4.0f << std::endl;
  std::cerr << _qd[r].transpose() << std::endl;


  // if(std::fabs(angle) < 0.05f && (_fx[r]).norm()<0.04 && _alignedWithTrocar[r] == false && fabs(bmax-1.0f)<FLT_EPSILON)
  if(std::fabs(angle) < 0.05f && std::fabs(std::acos(_trocarOrientation[r][indexMax].dot(_wRb[r].col(2))))<0.07f && _alignedWithTrocar[r] == false && fabs(bmax-1.0f)<FLT_EPSILON)
  {
    _alignedWithTrocar[r] = true;
  }

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);
  // _omegad[r].setConstant(0.0f);

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  std::cerr << r << ": Beliefs: " << _beliefs[r].transpose() << std::endl;
  std::cerr << r << ": Aligned with trocar: " << _alignedWithTrocar[r] << std::endl;
}


void KukaDemo::trocarAdaptation(int r)
{
  ////////////////////////////////////
  // Compute human desired velocity //
  ////////////////////////////////////

  float velocityLimit = 0.3f;
  Eigen::Vector3f gains;
  if(!_useJoystick)
  {
    gains << 2.0f*velocityLimit/FOOT_INTERFACE_X_RANGE, 2.0f*velocityLimit/FOOT_INTERFACE_Y_RANGE, 0.0f;
  }
  else
  {
    gains.setConstant(velocityLimit);
    gains(2) = 0.0f;
  }
  
  Eigen::Vector3f temp;
  temp(0) = Utils<float>::deadZone(_footPosition[r](0),-0.02f,0.02f);
  temp(1) = Utils<float>::deadZone(_footPosition[r](1),-0.02f,0.02f);
  if(!_useJoystick)
  {
    temp(2) = Utils<float>::deadZone(_footPosition[r](2),-5.0f,5.0f);
  }
  else
  {
    temp(2) = Utils<float>::deadZone(_footPosition[r](2),-0.02f,0.02f);
  }
  _vdFoot[r] = gains.cwiseProduct(temp);

  if(_vdFoot[r].norm()>velocityLimit)
  {
    _vdFoot[r] *= velocityLimit/_vdFoot[r].norm();
  }  

  /////////////////////
  // Task adaptation //
  /////////////////////

  _fx[r].setConstant(0.0f);
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    _fx[r]+=_beliefs[r](k)*_fxk[r][k];
  }

  Eigen::MatrixXf::Index indexMax;

  float a, b;
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    a = _adaptationRate*((_v[r]-_fx[r]).dot(_fxk[r][k]));
    b = _adaptationRate*(0.5f*(_beliefs[r](k)-0.5f)*_fxk[r][k].squaredNorm());

    _dbeliefs[r](k) = a+b;
    std::cerr << r <<": Dbeliefs " << k << ": " << a << " " << b << std::endl;
  }

  std::cerr << r << ": a: " << _dbeliefs[r].transpose() << std::endl;
  float dbmax = _dbeliefs[r].array().maxCoeff(&indexMax);

  if(std::fabs(1.0f-_beliefs[r](indexMax))< FLT_EPSILON && std::fabs(_beliefs[r].sum()-1)<FLT_EPSILON)
  {
    _dbeliefs[r].setConstant(0.0f);
  }
  else
  {
    Eigen::VectorXf temp;
    temp.resize(_nbTrocar[r]-1);
    temp.setConstant(0.0f);
    int m = 0;
    for(int k = 0; k < _nbTrocar[r]; k++)
    {
      if(k!=indexMax)
      {
        temp(m) = _dbeliefs[r](k);
        m++;
      }
    }
    float db2max = temp.array().maxCoeff();
      std::cerr << db2max << std::endl;

    float z = (dbmax+db2max)/2.0f;
    _dbeliefs[r].array() -= z;

    float S = 0.0f;
    for(int k = 0; k < _nbTrocar[r]; k++)
    {
      if(fabs(_beliefs[r](k))>FLT_EPSILON || _dbeliefs[r](k) > 0)
      {
        S+=_dbeliefs[r](k);
      }
    }
    _dbeliefs[r][indexMax]-=S;
  }
  std::cerr << r << ": Dbeliefs: " << _dbeliefs[r].transpose() << std::endl;

  _beliefs[r]+=_dt*_dbeliefs[r];
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    _beliefs[r](k) = Utils<float>::bound(_beliefs[r](k),0.0f,1.0f);
  }

  _fx[r].setConstant(0.0f);
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    _fx[r]+=_beliefs[r](k)*_fxk[r][k];
  }

  _fx[r] = Utils<float>::bound(_fx[r],0.3f);
}

void KukaDemo::trocarInsertion(int r)
{
  std::cerr << r << ": TROCAR_INSERTION" << std::endl;

  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);
  Eigen::Vector3f offset = _sphereCenter;

  _vd[r].setConstant(0.0f);

  if(_useJoystick)
  {

    _vd[r] += -0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*_rEETrocar[r][indexMax].normalized();
  }
  else
  {
    if(r==LEFT)
    {
      _vd[r] += 2.0f*0.15f*_footPosition[r](2)/FOOT_INTERFACE_Y_RANGE*_wRb[r].col(2);

    }
  
  }

  std::cerr << r <<": vd: " << _vd[r].transpose() << std::endl; 

  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][indexMax]));

  Eigen::Vector3f axis;  
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,axis,angle);

  std::cerr << "Angle error to trocar position: " << angle << std::endl;

  if(std::fabs(angle)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angle,-MAX_ORIENTATION_ERROR,MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  std::cerr << _qd[r].transpose() << std::endl;

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);
  // _omegad[r].setConstant(0.0f);


  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _wRb0[r] = _wRb[r];
  _xd0[r] = _x[r];
  _inputAlignedWithOrigin[r] = false;
  _desiredOffset[r].setConstant(0.0f);
  _vdToolPast[r].setConstant(0.0f);
  _vdToolFiltPast[r].setConstant(0.0f);

}


void KukaDemo::trocarSpace(int r)
{

  std::cerr << r << ": TROCAR_SPACE" << std::endl;
  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);

  Eigen::Vector3f vdTool, gains, temp;

  if(!_useJoystick)
  {

    if(_footMode[r]==VELOCITY)
    {
      gains << 2.0f*0.03f/FOOT_INTERFACE_ROLL_RANGE,
               2.0f*0.03f/FOOT_INTERFACE_PITCH_RANGE,
               2.0f*0.15f/FOOT_INTERFACE_Y_RANGE;

      temp(0) = Utils<float>::deadZone(_footPosition[r](0),-5.0f,5.0f);
      temp(1) = Utils<float>::deadZone(_footPosition[r](1),-5.0f,5.0f);
      temp(2) = Utils<float>::deadZone(_footPosition[r](2),-0.03f,0.03f);
      // temp = _footPosition[r];

      // if((_x[r]-_trocarPosition[r][indexMax]).norm()<0.03f)
      // {
      //   temp(0) = 0.0f;
      //   temp(1) = 0.0f;
      // }
      

      vdTool = _wRb[r]*(gains.cwiseProduct(temp));      
      // Eigen::Vector3f temp
      // float alpha = 0.95;
      _desiredOffset[r]+=_dt*vdTool;

      // Eigen::Vector3f vdToolFilt = alpha*_vdToolFiltPast[r]+alpha*(vdTool-_vdToolPast[r]);
      // _vdToolFiltPast[r] = vdToolFilt;
      // _desiredOffset[r] += _dt*vdToolFilt;
      // _vdToolPast[r] = vdTool;
      _desiredOffset[r](0) = Utils<float>::bound(_desiredOffset[r](0),-0.15f,0.15f);
      _desiredOffset[r](1) = Utils<float>::bound(_desiredOffset[r](1),-0.15f,0.15f);
      _desiredOffset[r](2) = Utils<float>::bound(_desiredOffset[r](2),-0.15f,0.0f);

      vdTool = 2.0f*(_xd0[r]+_desiredOffset[r]-_x[r]);  
      std::cerr << "Desired offset: " <<_desiredOffset[r].transpose() << std::endl;
    }
    else if(_footMode[r]==POSITION)
    {
      Eigen::Vector3f desiredOffset, currentOffset, xd;

      gains << 2.0f/FOOT_INTERFACE_X_RANGE,
               2.0f/FOOT_INTERFACE_Y_RANGE,
               2.0f/FOOT_INTERFACE_PITCH_RANGE;
      
      temp  = gains.cwiseProduct(_footPosition[r]);


      desiredOffset.setConstant(0.0f);
      desiredOffset(2) = 0.25f*std::min(temp(2),0.0f);
      desiredOffset(1) = -desiredOffset(2)*std::tan(45.0f*M_PI/180.0f*std::min(std::max(temp(1),-1.0f),1.0f));
      desiredOffset(0) = -desiredOffset(2)*std::tan(45.0f*M_PI/180.0f*std::min(std::max(temp(0),-1.0f),1.0f));        
      
      currentOffset = _x[r]-_xd0[r];

      if((desiredOffset-currentOffset).norm()<0.07f)
      {
        _inputAlignedWithOrigin[r]=true;
        // std::cerr << "bou: " << std::endl;
      }

      if(_inputAlignedWithOrigin[r]==false)
      {
        vdTool.setConstant(0.0f);
      }
      else
      {
        xd = _xd0[r]+desiredOffset;
        vdTool = 2.0f*(xd-_x[r]);        
      }

      std::cerr << r << ": Desired offset: " << desiredOffset.transpose() << std::endl; 
      std::cerr << r << ": Current offset: " << currentOffset.transpose() << std::endl; 
      std::cerr << r << ": vd: " << vdTool.transpose() << std::endl; 
    }
    else
    {
      std::cerr << r << ": Foot mode unknown !" << std::endl;
      vdTool.setConstant(0.0f);

    }

    vdTool = Utils<float>::bound(vdTool,0.1f);

  }
  else
  {

    if(_footMode[r]==VELOCITY)
    {
      gains << 0.05f, -0.05f, -0.15f;
      vdTool = _wRb[r]*(gains.cwiseProduct(_footPosition[r].segment(0,3)));
    }
    else if(_footMode[r]==POSITION)
    {
      Eigen::Vector3f xd, desiredOffset, currentOffset;

      desiredOffset(2) = 0.25f*std::min(_footPosition[r](2),0.0f);
      desiredOffset(1) = desiredOffset(2)*std::tan(_footPosition[r](1)*45.0f*M_PI/180.0f);
      desiredOffset(0) = desiredOffset(2)*std::tan(_footPosition[r](0)*45.0f*M_PI/180.0f);

      currentOffset = _x[r]-_xd0[r];

      if((desiredOffset-currentOffset).norm()<0.02f && _inputAlignedWithOrigin[r]==false)
      {
        std::cerr << "Bring input to current robot position" << std::endl;
        _inputAlignedWithOrigin[r] = true;
        vdTool.setConstant(0.0f);
      }
      else
      {
        // if(currentOffset.norm()<0.01f && desiredOffset.norm()<0.01f)
        // {
        //   vdTool.setConstant(0.0f);
        // }
        // else
        {
          xd = _xd0[r]+desiredOffset;
          xd(2) -= 0.01f;
          vdTool = 2.0f*(xd-_x[r]);          
        }
      }

      std::cerr << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
      std::cerr << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
      std::cerr << r <<": vd: " << vdTool.transpose() << std::endl; 
    } 
    else
    {
      std::cerr << r << ": Foot mode unknown !" << std::endl;
      vdTool.setConstant(0.0f);
    }   


    vdTool = Utils<float>::bound(vdTool,0.1f);

  }

  Eigen::Matrix<float,6,6> A;
  A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2))*Eigen::Matrix3f::Identity();
  A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r][indexMax]);
  A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_toolOffsetFromEE[r]*_wRb[r].col(2));
  Eigen::Matrix<float,6,1> x, b;
  b.setConstant(0.0f);
  b.segment(3,3) = vdTool;

  x = A.colPivHouseholderQr().solve(b);
  _vd[r] = x.segment(0,3);

  if(_useSim)
  {
    _vd[r]+=20.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  }
  _vd[r] = Utils<float>::bound(_vd[r],0.3f);

  _omegad[r] = x.segment(3,3);
  _omegad[r] = Utils<float>::bound(_omegad[r],1.0f);


  _nullspaceWrench[r].setConstant(0.0f);
  // _nullspaceWrench[r].segment(0,3) = 50.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  // _nullspaceCommand[r] = 50.0f*jacobianRCM(_currentJoints[r],_trocarPosition[r][indexMax]).transpose()*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);


  std::cerr << "vd: " << _vd[r].transpose() << std::endl;


  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][indexMax]));


  Eigen::Vector3f axis;  
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,axis,angle);

  std::cerr << "Angle error to trocar position: " << angle << std::endl;

  if(std::fabs(angle)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angle,-MAX_ORIENTATION_ERROR,MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  float selfRotationCommand;
  if(!_useJoystick)
  {
    selfRotationCommand = -2.0f*1.0f*_footPose[r](YAW)/FOOT_INTERFACE_YAW_RANGE;
  }
  else
  {
    selfRotationCommand = -1.0f*_footPose[r](YAW);
  }

  _omegad[r] += Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r])+selfRotationCommand*_wRb[r].col(2);
  
  std::cerr << "omega: " <<_omegad[r].transpose() << std::endl;
}


void KukaDemo::footDataTransformation()
{
  Eigen::Matrix3f R;

  if(!_useJoystick)
  {

    R << 0.0f, 1.0f, 0.0f,
       -1.0f,0.0f,0.0f,
       0.0f,0.0f,1.0f;
  _footPosition[RIGHT] = R*_footPose[RIGHT].segment(0,3);
  // _footPosition[LEFT] = R*_footPose[LEFT].segment(0,3);

    R << 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, -1.0f,
         1.0f, 0.0f, 0.0f;

  // _footPosition[RIGHT] = R*_footPose[RIGHT].segment(1,3);
  _footPosition[LEFT] = R*_footPose[LEFT].segment(1,3);

  }
  else
  {
    R << 0.0f, 1.0f, 0.0f,
          1.0f,0.0f,0.0f,
          0.0f,0.0f,1.0f;
  _footPosition[RIGHT] = R*_footPose[RIGHT].segment(0,3);
  _footPosition[LEFT] = R*_footPose[LEFT].segment(0,3);
  }
  
}


void KukaDemo::computeDesiredFootWrench()
{
  // temp.setConstant(0.0f);
  _desiredFootWrench[RIGHT].setConstant(0.0f);
  // _desiredFootWrench[RIGHT](1) = _FdFoot[RIGHT](0);
  // _desiredFootWrench[RIGHT](0) = -_FdFoot[RIGHT](1);
  // _desiredFootWrench[RIGHT](2) = _FdFoot[RIGHT](2)*0.2;
  // _desiredFootWrench[RIGHT](0) += -_kxy*_footPose[RIGHT](0)-_dxy*_footTwist[RIGHT](0);
  // _desiredFootWrench[RIGHT](1) += -_kxy*_footPose[RIGHT](1)-_dxy*_footTwist[RIGHT](1);
  // _desiredFootWrench[RIGHT](2) += -_kphi*_footPose[RIGHT](2)-_dphi*_footTwist[RIGHT](2);
  // _desiredFootWrench[RIGHT](3) += -_kphi*_footPose[RIGHT](3)-_dphi*_footTwist[RIGHT](3);
  // _desiredFootWrench[RIGHT](4) += -_kphi*_footPose[RIGHT](4)-_dphi*_footTwist[RIGHT](4);

  _desiredFootWrench[LEFT].setConstant(0.0f);
  // _desiredFootWrench[LEFT](1) = _FdFoot[LEFT](0);
  // _desiredFootWrench[LEFT](0) = -_FdFoot[LEFT](1);
  // _desiredFootWrench[LEFT](2) = _FdFoot[LEFT](2)*0.2;
  // _desiredFootWrench[LEFT](0) += -_kxy*_footPose[LEFT](0)-_dxy*_footTwist[LEFT](0);
  // _desiredFootWrench[LEFT](1) += -_kxy*_footPose[LEFT](1)-_dxy*_footTwist[LEFT](1);
  // _desiredFootWrench[LEFT](2) += -_kphi*_footPose[LEFT](2)-_dphi*_footTwist[LEFT](2);
  // _desiredFootWrench[LEFT](3) += -_kphi*_footPose[LEFT](3)-_dphi*_footTwist[LEFT](3);
  // _desiredFootWrench[LEFT](4) += -_kphi*_footPose[LEFT](4)-_dphi*_footTwist[LEFT](4);
}


void KukaDemo::logData()
{
  _outputFile << ros::Time::now() << " "
              << _x[LEFT].transpose() << " "
              << _vd[LEFT].transpose() << " "
              << (_wRb[LEFT]*_filteredWrench[LEFT].segment(0,3)).transpose() << " "
              << (_wRb[LEFT]*_filteredWrench[LEFT].segment(3,3)).transpose() << " "
              << _x[RIGHT].transpose() << " "
              << _vd[RIGHT].transpose() << " "
              << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(0,3)).transpose() << " "
              << (_wRb[RIGHT]*_filteredWrench[RIGHT].segment(3,3)).transpose() << " "
              << _footPose[LEFT](0) << " "
              << _footPose[LEFT](1) << " "
              << _footPose[LEFT](2) << " "
              << _footWrench[LEFT](0) << " "
              << _footWrench[LEFT](1) << " "
              << _footWrench[LEFT](2) << " "
              << _desiredFootWrench[LEFT](0) << " "
              << _desiredFootWrench[LEFT](1) << " "
              << _desiredFootWrench[LEFT](2) << " "
              << _footState[LEFT] << " "
              << _footPose[RIGHT](0) << " "
              << _footPose[RIGHT](1) << " "
              << _footPose[RIGHT](2) << " "
              << _footWrench[RIGHT](0) << " "
              << _footWrench[RIGHT](1) << " "
              << _footWrench[RIGHT](2) << " "
              << _desiredFootWrench[RIGHT](0) << " "
              << _desiredFootWrench[RIGHT](1) << " "
              << _desiredFootWrench[RIGHT](2) << " "
              << _footState[RIGHT] << std::endl;


}


void KukaDemo::publishData()
{
  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {

      // Publish desired twist (passive ds controller)
      _msgDesiredTwist.linear.x  = _vd[r](0);
      _msgDesiredTwist.linear.y  = _vd[r](1);
      _msgDesiredTwist.linear.z  = _vd[r](2);

      // Convert desired end effector frame angular velocity to world frame
      _msgDesiredTwist.angular.x = _omegad[r](0);
      _msgDesiredTwist.angular.y = _omegad[r](1);
      _msgDesiredTwist.angular.z = _omegad[r](2);

      _pubDesiredTwist[r].publish(_msgDesiredTwist);

      // Publish desired orientation
      _msgDesiredOrientation.w = _qd[r](0);
      _msgDesiredOrientation.x = _qd[r](1);
      _msgDesiredOrientation.y = _qd[r](2);
      _msgDesiredOrientation.z = _qd[r](3);

      _pubDesiredOrientation[r].publish(_msgDesiredOrientation);

      _msgFilteredWrench.header.frame_id = "world";
      _msgFilteredWrench.header.stamp = ros::Time::now();
      _msgFilteredWrench.wrench.force.x = _filteredWrench[r](0);
      _msgFilteredWrench.wrench.force.y = _filteredWrench[r](1);
      _msgFilteredWrench.wrench.force.z = _filteredWrench[r](2);
      _msgFilteredWrench.wrench.torque.x = _filteredWrench[r](3);
      _msgFilteredWrench.wrench.torque.y = _filteredWrench[r](4);
      _msgFilteredWrench.wrench.torque.z = _filteredWrench[r](5);
      _pubFilteredWrench[r].publish(_msgFilteredWrench);

      // for(int m = 0; m < 5; m++)
      // {
      //   _msgFootInput.ros_effort[m] = _desiredFootWrench[r](m);
      // }
      // _pubFootInput[r].publish(_msgFootInput);

      geometry_msgs::Wrench wrench;
      wrench.force.x = _nullspaceWrench[r](0);
      wrench.force.y = _nullspaceWrench[r](1);
      wrench.force.z = _nullspaceWrench[r](2);
      wrench.torque.x = _nullspaceWrench[r](3);
      wrench.torque.y = _nullspaceWrench[r](4);
      wrench.torque.z = _nullspaceWrench[r](5);
      _pubDesiredWrench[r].publish(wrench);

      _msgNullspaceCommand.data.resize(7);
      for(int m = 0; m < 7; m++)
      {
        _msgNullspaceCommand.data[m] = _nullspaceCommand[r](m);
      }
      _pubNullspaceCommand[r].publish(_msgNullspaceCommand);
    }
  }
}


void KukaDemo::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _xEE[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _xEE[k]+_toolOffsetFromEE[k]*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _xEE[k] += _leftRobotOrigin;
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


void KukaDemo::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void KukaDemo::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
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
      std::cerr << "[KukaDemo]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
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


void KukaDemo::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void KukaDemo::updateFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg, int k)
{
  for(int m = 0; m < 5; m++)
  {
    _footPose[k](m) = msg->platform_position[m];
    _footTwist[k](m) = msg->platform_speed[m];
    _footWrench[k](m) = msg->platform_effortD[m];
  }
  _footPose[k] -= _footOffset[k];
  _footState[k] = msg->platform_machineState;

  if(!_firstFootOutput[k])
  {
    _firstFootOutput[k] = true;
  }
}


void KukaDemo::updateJoystick(const sensor_msgs::Joy::ConstPtr& joy, int k)
{

  _footPose[k].setConstant(0.0f);
  _footPose[k](X) = joy->axes[0];
  _footPose[k](Y) = joy->axes[1];
  // _footPose[k](PITCH) = joy->axes[3];
  // _footPose[k](YAW) = joy->axes[2];
  _footPose[k](PITCH) = joy->axes[4]-_joyOffsetPitch;
  _footPose[k](YAW) = joy->axes[3];
  
  if(!_firstJoystick[k])
  {
    _firstJoystick[k]= true;
  }
}

void KukaDemo::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k) 
{
  for(int m = 0; m < 7; m++)
  {
    _currentJoints[k](m) = msg->position[m];
  }
  if(!_firstJointsUpdate[k])
  {
    _firstJointsUpdate[k] = true;
  }
}



void KukaDemo::dynamicReconfigureCallback(robotic_experiments::feetTelemanipulation_paramsConfig &config, uint32_t level)
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
  _useSharedControl = config.useSharedControl;
}


void KukaDemo::initializeCustomTrocars()
{
  if(_useSphericalTrocars)
  {
    _sphereCenter << -0.33f-0.13f, 0.45f, -0.015f;

    float sphereRadius = 0.13f;
    float medialDivision = 10.0f;
    float transversalDivision = 12.0f;

    float angle1 = M_PI/medialDivision;
    float angle2  = 2*M_PI/transversalDivision;

    Eigen::MatrixXf positions;
    Eigen::MatrixXf orientations;
    positions.resize((medialDivision/2-1)*transversalDivision+1,3);
    orientations.resize((medialDivision/2-1)*transversalDivision+1,3);

    int id = 0;
    positions.row(id) << 0.0f,0.0f,sphereRadius;
    positions.row(id)+=_sphereCenter.transpose();
    orientations.row(id) = -(positions.row(id)-_sphereCenter.transpose()).normalized();
    id++;
    for(int k = 1; k <medialDivision/2; k+=1)
    {
      for(int m = 1 ; m < transversalDivision+1; m+=1)
      {
        positions.row(id) << sphereRadius*cos(k*angle1)*cos((m-1)*angle2),
                                   sphereRadius*cos(k*angle1)*sin((m-1)*angle2),
                                   sphereRadius*sin(k*angle1);
              positions.row(id)+=_sphereCenter.transpose();
              orientations.row(id) = -(positions.row(id)-_sphereCenter.transpose()).normalized();
              id++;
      }
    }
    std::vector<int> idConserved[NB_ROBOTS];
    // idConserved[LEFT].push_back(0);
    idConserved[LEFT].push_back(28);
    idConserved[LEFT].push_back(4);
    idConserved[RIGHT].push_back(34);
    idConserved[RIGHT].push_back(10);

    for(int r = 0; r < NB_ROBOTS; r++)
    {
      _nbTrocar[r] = idConserved[r].size();
      for(int k = 0; k < _nbTrocar[r]; k++)
      {
        _trocarPosition[r].push_back(positions.row(idConserved[r][k]).transpose());
        _trocarOrientation[r].push_back(orientations.row(idConserved[r][k]).transpose());
        std::cerr << r << ": " << k << ":  " << _trocarPosition[r][k].transpose() << std::endl;
        std::cerr << r << ": " << k << ":  " << _trocarOrientation[r][k].transpose() << std::endl;
      }
    }
    
  }
  else
  {

    Eigen::Vector3f temp;
    // temp << -0.3, -0.45f, 0.243; // with respect to left robot
    temp << -0.304, -0.432f, 0.696f-_toolOffsetFromEE[LEFT]; // with respect to left robot

    temp += _leftRobotOrigin;
    _trocarPosition[LEFT].push_back(temp);

    // temp << 0.378f,-0.376f,-0.846f;
    temp << 0.265f,-0.490f,-0.830f;
    temp.normalize();
    _trocarOrientation[LEFT].push_back(temp);

    // temp << -0.202, 0.507f, 0.266;
    temp << -0.308f, 0.471f, 0.741f-_toolOffsetFromEE[RIGHT]; // with respect to left robot
    _trocarPosition[RIGHT].push_back(temp);  

    // temp << 0.104f,0.259f,-0.960f;
    temp << 0.116f,0.220f,-0.968f;
    temp.normalize();
    _trocarOrientation[RIGHT].push_back(temp); 
  }

  for(int r = 0; r < NB_ROBOTS; r++)
  {
    _nbTrocar[r] = _trocarPosition[r].size();
    _rEETrocar[r].resize(_nbTrocar[r]);
    _rEERCM[r].resize(_nbTrocar[r]);
    _xRCM[r].resize(_nbTrocar[r]);
    _xdEE[r].resize(_nbTrocar[r]);
    _fxk[r].resize(_nbTrocar[r]);
    _beliefs[r].resize(_nbTrocar[r]);
    _beliefs[r].setConstant(0.0f);
    _beliefs[r](0) = 1.0f;
    _dbeliefs[r].resize(_nbTrocar[r]);
    _dbeliefs[r].setConstant(0.0f);
    _robotMode[r] = TROCAR_SELECTION;
  }
}


void KukaDemo::registerTrocars()
{
  if(!_trocarsRegistered[LEFT] || !_trocarsRegistered[RIGHT])
  {
    static struct termios oldSettings, newSettings;
    tcgetattr( STDIN_FILENO, &oldSettings);           // save old settings
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON);                 // disable buffering     
    newSettings.c_cc[VMIN] = 0; 
    newSettings.c_cc[VTIME] = 0; 
    tcsetattr( STDIN_FILENO, TCSANOW, &newSettings);  // apply new settings

    int c = getchar();  // read character (non-blocking)
    int add[NB_ROBOTS];
    add[LEFT] = 'a';
    add[RIGHT] = 'b';
    int finish[NB_ROBOTS];
    finish[LEFT] = 'l';
    finish[RIGHT] = 'r';

    Eigen::VectorXf temp;
    for(int r = 0; r < NB_ROBOTS; r++)
    { 
      if(_useRobot[r])
      {        
        if(c==add[r])
        {
          if(_trocarPosition[r].empty())
          {
            _trocarPosition[r].push_back(_x[r]);
            _trocarOrientation[r].push_back(_wRb[r].col(2));
            std::cerr << r << ": Adding first trocar: " << _x[r].transpose() << std::endl;
          }
          else
          {
            temp.resize(_trocarPosition[r].size());
            for(int m = 0 ; m < temp.size(); m++)
            {
              temp(m) = (_trocarPosition[r][m]-_x[r]).norm();
            }
            Eigen::MatrixXf::Index indexMin;
            float dmin = temp.array().minCoeff(&indexMin);
            std::cerr << "dmin: " << dmin << std::endl;
            if(dmin<0.01f)
            {
              _trocarPosition[r][indexMin] = _x[r];
              _trocarOrientation[r][indexMin] = _wRb[r].col(2);
              std::cerr << r << ": Changing trocar " << indexMin <<  ":" << _x[r].transpose() << std::endl;

            }
            else
            {
              _trocarPosition[r].push_back(_x[r]);
              _trocarOrientation[r].push_back(_wRb[r].col(2));
              std::cerr << r << ": Adding trocar: " << _x[r].transpose() <<  std::endl;
            }
          }
        }
        else if(c==finish[r])
        {
          if(!_trocarPosition[r].empty())
          {
            _trocarsRegistered[r] = true;
            std::cerr << r << ": Finished registering" << std::endl;
          }
        }
      } 
      else
      {
        _trocarsRegistered[r] = true;
      }    
    }
    tcsetattr( STDIN_FILENO, TCSANOW, &oldSettings);  // restore old settings
  
  }

  if(_trocarsRegistered[LEFT] && _trocarsRegistered[RIGHT])
  {
    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_useRobot[r])
      {      
        _nbTrocar[r] = _trocarPosition[r].size();
        _rEETrocar[r].resize(_nbTrocar[r]);
        _rEERCM[r].resize(_nbTrocar[r]);
        _xRCM[r].resize(_nbTrocar[r]);
        _xdEE[r].resize(_nbTrocar[r]);
        _fxk[r].resize(_nbTrocar[r]);
        _beliefs[r].resize(_nbTrocar[r]);
        _beliefs[r].setConstant(0.0f);
        _beliefs[r](_nbTrocar[r]-1) = 1.0f;
        _dbeliefs[r].resize(_nbTrocar[r]);
        _dbeliefs[r].setConstant(0.0f);
        _robotMode[r] = TROCAR_SELECTION;
      }
    }
  }
}