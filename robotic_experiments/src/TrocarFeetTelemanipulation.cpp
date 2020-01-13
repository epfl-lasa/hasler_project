#include "TrocarFeetTelemanipulation.h"
#include "Utils.h"
#include "jacobianRCM.h"


TrocarFeetTelemanipulation* TrocarFeetTelemanipulation::me = NULL;

TrocarFeetTelemanipulation::TrocarFeetTelemanipulation(ros::NodeHandle &n, double frequency, std::string filename):
  _n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _filename(filename)
{
  me = this;

  _useLeftRobot = true;
  _useRightRobot = true;
  _sim = false;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  // _toolOffsetFromEE[LEFT] = 0.41f;
  // _toolOffsetFromEE[RIGHT] = 0.43f;
  // _toolOffsetFromEE[LEFT] = 0.44f+0.015f;
  _toolOffsetFromEE[LEFT] = 0.408f+0.015f;
  _toolOffsetFromEE[RIGHT] = 0.43f+0.015f;

  _toolMass = 0.2f;

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
    _normalForce[k] = 0.0f;
    _Fd[k] = 0.0f;

    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _firstWrenchReceived[k] = false;
    _firstDampingMatrix[k] = true;
    _firstJointsUpdate[k] = false;
    _firstJoystick[k] = false;

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
    _xTrocar[k] << -0.5f,0.0f,0.1f;
    _nullspaceWrench[k].setConstant(0.0f);
    _alignedWithTrocar[k] = false;
    _nullspaceCommand[k].setConstant(0.0f);

  }

  _stop = false;
  // _leftRobotOrigin << 0.066f, 0.9f, 0.0f;
  _leftRobotOrigin << 0.07f, 1.083f, 0.0f;
  // _leftRobotOrigin.setConstant(0.0f);
  // _x0[LEFT](0) = _leftRobotOrigin(0)-0.60f;
  // _x0[LEFT](1) = _leftRobotOrigin(1)-0.35f;
  // _x0[LEFT](2) = _leftRobotOrigin(2)+0.45f;
  // _x0[RIGHT](0) = -0.52f;
  // _x0[RIGHT](1) = 0.4f;
  // _x0[RIGHT](2) = 0.45f;
  // _graspingForceThreshold = 4.0f;  // Grasping force threshold [N]
  // _objectGrasped = false;
  // _targetForce = 15.0f;


  
  // _velocityLimit = 0.4f;
  // _kxy = 0.0f;
  // _dxy = 0.0f;
  // _kphi = 0.0f;
  // _dphi = 0.0f;

  // _trocarPosition[0] << -0.5f,0.0f,0.1f;
  // _trocarOrientation[0] << 0.0f,0.0f,-1.0f;
  // _trocarPosition[1] << -0.5f,0.15f,0.1f;
  // _trocarOrientation[1] << 0, -0.5, -0.86603;
  // _trocarPosition[2] << -0.5f,-0.15f,0.1f;
  // _trocarOrientation[2] << 0, 0.5, -0.86603;


  Eigen::Vector3f offset;
  // offset << -0.5f, 0.0f, 0.0f;
  // _sphereCenter << -0.33f-0.13f, 0.45f, -0.015f;
  // _sphereCenter << -0.503, 0.452f, 0.012f;
  // _sphereCenter << -0.23, 0.452f, -0.015f;
  _sphereCenter << -0.376, 0.536f, 0.022f;

  float sphereRadius = 0.13f;
  float medialDivision = 10.0f;
  float transversalDivision = 12.0f;

  float angle1 = M_PI/medialDivision;
  float angle2  = 2*M_PI/transversalDivision;

  // int id = 0;
  // _trocarPosition[id] << 0.0f,0.0f,sphereRadius;
  // _trocarPosition[id]+=offset;
  // _trocarOrientation[id] << 0.0f, 0.0f, -1.0f;
  // id++;
  // for(int k = 1; k <medialDivision/2; k+=2)
  // {
  //   for(int m = 1 ; m < transversalDivision+1; m+=3)
  //   {
  //     _trocarPosition[id] << sphereRadius*cos(k*angle1)*cos((m-1)*angle2),
  //                                sphereRadius*cos(k*angle1)*sin((m-1)*angle2),
  //                                sphereRadius*sin(k*angle1);
  //           _trocarPosition[id]+=offset;
  //           _trocarOrientation[id] = -(_trocarPosition[id]-offset).normalized();
  //           id++;
  //   }
  // }

  // Eigen::MatrixXf positions;
  // Eigen::MatrixXf orientations;
  // positions.resize((medialDivision/2-1)*transversalDivision+1,3);
  // orientations.resize((medialDivision/2-1)*transversalDivision+1,3);

  // int id = 0;
  // positions.row(id) << 0.0f,0.0f,sphereRadius;
  // positions.row(id)+=_sphereCenter.transpose();
  // orientations.row(id) = -(positions.row(id)-_sphereCenter.transpose()).normalized();
  // id++;
  // for(int k = 1; k <medialDivision/2; k+=1)
  // {
  //   for(int m = 1 ; m < transversalDivision+1; m+=1)
  //   {
  //     positions.row(id) << sphereRadius*cos(k*angle1)*cos((m-1)*angle2),
  //                                sphereRadius*cos(k*angle1)*sin((m-1)*angle2),
  //                                sphereRadius*sin(k*angle1);
  //           positions.row(id)+=_sphereCenter.transpose();
  //           orientations.row(id) = -(positions.row(id)-_sphereCenter.transpose()).normalized();
  //           id++;
  //   }
  // }
  // std::vector<int> idConserved[NB_ROBOTS];
  // // idConserved.push_back(0);
  // // idConserved[LEFT].push_back(0);
  // idConserved[LEFT].push_back(28);
  // idConserved[LEFT].push_back(4);
  // idConserved[RIGHT].push_back(34);
  // idConserved[RIGHT].push_back(10);

  // for(int r = 0; r < NB_ROBOTS; r++)
  // {
  //   _nbTrocar[r] = idConserved[r].size();
  //   for(int k = 0; k < _nbTrocar[r]; k++)
  //   {
  //     _trocarPosition[r].push_back(positions.row(idConserved[r][k]).transpose());
  //     _trocarOrientation[r].push_back(orientations.row(idConserved[r][k]).transpose());
  //     std::cerr << r << ": " << k << ":  " << _trocarPosition[r][k].transpose() << std::endl;
  //     std::cerr << r << ": " << k << ":  " << _trocarOrientation[r][k].transpose() << std::endl;
  //   }
  //   _rEETrocar[r].resize(_nbTrocar[r]);
  //   _rToolTrocar[r].resize(_nbTrocar[r]);
  //   _rEERCM[r].resize(_nbTrocar[r]);
  //   _xRCM[r].resize(_nbTrocar[r]);
  //   _xdEE[r].resize(_nbTrocar[r]);
  //   _fxk[r].resize(_nbTrocar[r]);
  //   _beliefs[r].resize(_nbTrocar[r]);
  //   _beliefs[r].setConstant(0.0f);
  //   _beliefs[r](0) = 1.0f;
  //   _dbeliefs[r].resize(_nbTrocar[r]);
  //   _dbeliefs[r].setConstant(0.0f);
  //   _mode[r] = TROCAR_SELECTION;
  // }
  Eigen::Vector3f temp;
  // temp << -0.35, 0.46f, 0.14f;
  temp << -0.3, -0.45f, 0.243; // with respect to left robot
  temp += _leftRobotOrigin;
  _trocarPosition[LEFT].push_back(temp);
  
  temp << -0.202, 0.507f, 0.266; // with respect to left robot
  _trocarPosition[RIGHT].push_back(temp);

  temp << 0.378f,-0.376f,-0.846f;
  temp.normalize();
  _trocarOrientation[LEFT].push_back(temp);
  
  temp << 0.104f,0.259f,-0.960f;
  temp.normalize();
  _trocarOrientation[RIGHT].push_back(temp);
  // temp << -0.46, 0.54, 0.14;
  // _trocarPosition[LEFT].push_back(temp)
  // _trocarPosition[RIGHT].push_back(temp)


  for(int r = 0; r < NB_ROBOTS; r++)
  {
    _nbTrocar[r] = _trocarPosition[r].size();
    _rEETrocar[r].resize(_nbTrocar[r]);
    _rToolTrocar[r].resize(_nbTrocar[r]);
    _rEERCM[r].resize(_nbTrocar[r]);
    _xRCM[r].resize(_nbTrocar[r]);
    _xdEE[r].resize(_nbTrocar[r]);
    _fxk[r].resize(_nbTrocar[r]);
    _beliefs[r].resize(_nbTrocar[r]);
    _beliefs[r].setConstant(0.0f);
    _beliefs[r](0) = 1.0f;
    _dbeliefs[r].resize(_nbTrocar[r]);
    _dbeliefs[r].setConstant(0.0f);
    _mode[r] = TROCAR_SELECTION;
  }
  _adaptationRate = 50.0f;


  _useJoystick = false;

  _footOffset[LEFT].setConstant(0.0f);
  _footOffset[RIGHT].setConstant(0.0f);
  _footOffset[RIGHT](2) = 10.0f;
}





bool TrocarFeetTelemanipulation::init() 
{
  // Subscriber definitions
  _subRobotPose[LEFT] = _n.subscribe<geometry_msgs::Pose>("/lwr2/ee_pose", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _n.subscribe<geometry_msgs::Twist>("/lwr2/ee_vel", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[LEFT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&TrocarFeetTelemanipulation::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[LEFT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCurrentJoints[LEFT] = _n.subscribe<sensor_msgs::JointState>("/lwr2/joint_states", 1, boost::bind(&TrocarFeetTelemanipulation::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[LEFT] = _n.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Left",1, boost::bind(&TrocarFeetTelemanipulation::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[RIGHT] = _n.subscribe<geometry_msgs::Pose>("/lwr/ee_pose", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _n.subscribe<geometry_msgs::Twist>("/lwr/ee_vel", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subDampingMatrix[RIGHT] = _n.subscribe<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&TrocarFeetTelemanipulation::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor[RIGHT] = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&TrocarFeetTelemanipulation::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[RIGHT] = _n.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Right",1, boost::bind(&TrocarFeetTelemanipulation::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subCurrentJoints[RIGHT] = _n.subscribe<sensor_msgs::JointState>("/lwr/joint_states", 1, boost::bind(&TrocarFeetTelemanipulation::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
 

  if(_useJoystick)
  {
    _subJoystick[LEFT] = _n.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&TrocarFeetTelemanipulation::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subJoystick[RIGHT] = _n.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&TrocarFeetTelemanipulation::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  }
  // Publisher definitions
  _pubDesiredTwist[LEFT] = _n.advertise<geometry_msgs::Twist>("/lwr2/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _n.advertise<geometry_msgs::Quaternion>("/lwr2/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench[LEFT] = _n.advertise<geometry_msgs::Wrench>("/lwr2/joint_controllers/passive_ds_command_force", 1);
  _pubFilteredWrench[LEFT] = _n.advertise<geometry_msgs::WrenchStamped>("TrocarFeetTelemanipulation/filteredWrenchLeft", 1);
  _pubNormalForce[LEFT] = _n.advertise<std_msgs::Float32>("TrocarFeetTelemanipulation/normalForceLeft", 1);
  _pubFootInput[LEFT] = _n.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Right", 1);
  _pubNullspaceCommand[LEFT] = _n.advertise<std_msgs::Float32MultiArray>("/lwr2/joint_controllers/passive_ds_command_nullspace", 1);

  _pubDesiredTwist[RIGHT] = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredWrench[RIGHT] = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);

  _pubFilteredWrench[RIGHT] = _n.advertise<geometry_msgs::WrenchStamped>("TrocarFeetTelemanipulation/filteredWrenchRight", 1);
  _pubNormalForce[RIGHT] = _n.advertise<std_msgs::Float32>("TrocarFeetTelemanipulation/normalForceRight", 1);
  _pubFootInput[RIGHT] = _n.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Left", 1);
  _pubNullspaceCommand[RIGHT] = _n.advertise<std_msgs::Float32MultiArray>("/lwr/joint_controllers/passive_ds_command_nullspace", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&TrocarFeetTelemanipulation::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,TrocarFeetTelemanipulation::stopNode);

  _outputFile.open(ros::package::getPath(std::string("robotic_experiments"))+"/data_foot/"+_filename+".txt");

  if(!_n.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRightRobot && !_sim)
  {
    ROS_ERROR("[TrocarFeetTelemanipulation]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }
  else if(!_n.getParamCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useRightRobot && _sim)
  {
    ROS_ERROR("[TrocarFeetTelemanipulation]: Cannot read first eigen value of passive ds controller for right robot");
    return false;
  }


  if(!_n.getParamCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]) && _useLeftRobot)
  {
    ROS_ERROR("[TrocarFeetTelemanipulation]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }
  else if(!_n.getParamCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]) && _useLeftRobot && _sim)
  {
    ROS_ERROR("[TrocarFeetTelemanipulation]: Cannot read first eigen value of passive ds controller for left robot");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[TrocarFeetTelemanipulation]: The TrocarFeetTelemanipulation node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[TrocarFeetTelemanipulation]: The ros node has a problem.");
    return false;
  }
}


void TrocarFeetTelemanipulation::run()
{
  _timeInit = ros::Time::now().toSec();

  while (!_stop) 
  {
    if(allDataReceived())
    {
      // _mutex.lock();

      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
      ros::param::getCached("/lwr2/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);
          
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();

      // _mutex.unlock();
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


void TrocarFeetTelemanipulation::stopNode(int sig)
{
  me->_stop = true;
}


bool TrocarFeetTelemanipulation::allDataReceived()
{
  if(_useLeftRobot && _useRightRobot && !_sim)
  {
    // std::cerr <<(int) _firstRobotPose[RIGHT] << (int) _firstRobotTwist[RIGHT] << (int) _wrenchBiasOK[RIGHT] << (int) _firstDampingMatrix[RIGHT] << (int) _firstFootOutput[RIGHT] << (int)
    //         _firstRobotPose[LEFT] << (int) _firstRobotTwist[LEFT] << (int) _wrenchBiasOK[LEFT] << (int) _firstDampingMatrix[LEFT] << (int) _firstFootOutput[LEFT] << std::endl;
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _wrenchBiasOK[RIGHT] && _firstDampingMatrix[RIGHT] && _firstJointsUpdate[RIGHT] && (_firstFootOutput[RIGHT] || _firstJoystick[RIGHT]) &&
            _firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[LEFT] && _firstJointsUpdate[LEFT] && (_firstFootOutput[LEFT] || _firstJoystick[LEFT]) );
     
  }
  else if(_useLeftRobot && _useRightRobot && _sim)
  {
    // std::cerr <<(int) _firstRobotPose[RIGHT] << (int) _firstRobotTwist[RIGHT] << (int) _wrenchBiasOK[RIGHT] << (int) _firstDampingMatrix[RIGHT] << (int) _firstFootOutput[RIGHT] << (int)
    //         _firstRobotPose[LEFT] << (int) _firstRobotTwist[LEFT] << (int) _wrenchBiasOK[LEFT] << (int) _firstDampingMatrix[LEFT] << (int) _firstFootOutput[LEFT] << std::endl;
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && (_firstFootOutput[RIGHT] || _firstJoystick[RIGHT]) && _firstJointsUpdate[RIGHT] &&
            _firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && (_firstFootOutput[LEFT] || _firstJoystick[LEFT]) && _firstJointsUpdate[LEFT]);
     
  }  else if(_useLeftRobot && !_useRightRobot && !_sim)
  {

    // std::cerr << (int)             _firstRobotPose[LEFT] << (int) _firstRobotTwist[LEFT] << (int) _wrenchBiasOK[LEFT] << (int) _firstDampingMatrix[LEFT] << (int) _firstJoystick[LEFT] << std::endl;
    return (_firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && _wrenchBiasOK[LEFT] && _firstDampingMatrix[LEFT] && _firstJointsUpdate[LEFT] && (_firstFootOutput[LEFT] || _firstJoystick[LEFT]));
  }
  else if(!_useLeftRobot && _useRightRobot && !_sim)
  {
    // return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _wrenchBiasOK[RIGHT] && _firstDampingMatrix[RIGHT] && _firstFootOutput[RIGHT]);
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _firstDampingMatrix[RIGHT] && _wrenchBiasOK[RIGHT] && (_firstFootOutput[RIGHT]|| _firstJoystick));
  }
  else if(_useLeftRobot && _sim)
  {
    // std::cerr << _firstRobotPose[LEFT] << " " << _firstRobotTwist[LEFT] << " " << _firstJoystick << " " << _firstJointsUpdate[LEFT] << std::endl;

    return (_firstRobotPose[LEFT] && _firstRobotTwist[LEFT] && (_firstFootOutput[LEFT] || _firstJoystick[LEFT]) && _firstJointsUpdate[LEFT]);
  }
  else if(_useRightRobot && _sim)
  {
    // return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _firstDampingMatrix[RIGHT] && _firstFootOutput[RIGHT]);
    // std::cerr << _firstRobotPose[RIGHT] << " " << _firstRobotTwist[RIGHT] << std::endl;
    return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && (_firstFootOutput[RIGHT] || _firstJoystick[RIGHT]) && _firstJointsUpdate[RIGHT]);
  }
  // else if(_useRightRobot && !_sim)
  // {
  //   // return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && _firstDampingMatrix[RIGHT] && _firstFootOutput[RIGHT]);
    // std::cerr << _firstRobotPose[RIGHT] << " " << _firstRobotTwist[RIGHT] << " " << _firstJoystick << std::endl;
  //   return (_firstRobotPose[RIGHT] && _firstRobotTwist[RIGHT] && (_firstFootOutput[RIGHT]|| _firstJoystick));
  // }
  else
  {
    return false;
  }
}


void TrocarFeetTelemanipulation::computeCommand()
{
  footDataTransformation();

  updateTrocarInformation();

  selectMode();

  for(int r = 0; r <NB_ROBOTS; r++)
  {
    if((r == LEFT && _useLeftRobot) || (r==RIGHT && _useRightRobot))
    {
      switch(_mode[r])
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

  // _vd[LEFT].setConstant(0.0f);


  // computeDesiredOrientation();

  if(!_sim && !_useJoystick)
  {
    computeDesiredFootWrench();
  }
}


void TrocarFeetTelemanipulation::updateTrocarInformation()
{

  for(int r = 0; r < NB_ROBOTS; r++)
  {
    for(int k = 0; k < _nbTrocar[r]; k++)
    {
      _rEETrocar[r][k] = _trocarPosition[r][k]-_xEE[r];
      _xRCM[r][k] = _xEE[r]+(_trocarPosition[r][k]-_xEE[r]).dot(_wRb[r].col(2))*_wRb[r].col(2);

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

    if(std::fabs(1.0f-bmax)<FLT_EPSILON && _mode[r] == TROCAR_INSERTION)
    {
      if((Utils<float>::orthogonalProjector(_rEETrocar[r][indexMax].normalized())*_filteredWrench[r].segment(0,3)).norm() > 30.0f)
      {
        ROS_INFO("TROCAR UPDATE");
        _trocarPosition[r][indexMax] = _x[r];
      }
    }     
  }
}

void TrocarFeetTelemanipulation::selectMode()
{


  for(int r = 0; r < NB_ROBOTS; r++)
  {
    Eigen::MatrixXf::Index indexMax;
    float bmax = _beliefs[r].array().maxCoeff(&indexMax);

    if(std::fabs(1.0f-bmax)>FLT_EPSILON)
    {
      _mode[r] = TROCAR_SELECTION;
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
          _mode[r] = TROCAR_SELECTION;
        }
        else if(distance >= -0.01f && distance < 0.04f)
        {
          _mode[r] = TROCAR_INSERTION;
        }
        else
        {
          _mode[r] = TROCAR_SPACE;
        }
      }
    }
  }
}


void TrocarFeetTelemanipulation::trocarSelection(int r)
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

  // std::cerr << _trocarPosition[r][indexMax].transpose() << std::endl;
  // std::cerr << _x[r].transpose() << std::endl;
  Eigen::Vector3f offset = _sphereCenter;

  if(fabs(bmax-1.0f)>FLT_EPSILON)
  {
    _alignedWithTrocar[r] = false;
    // _vd[r] = _fx[r];
    // _vd[r] = Utils<float>::orthogonalProjector((offset-_xEE[r]).normalized())*_fx[r];
    _vd[r] = _fx[r];

    std::cerr << "TROCAR NOT SELECTED" << std::endl;
  }
  else 
  {
    if(!_alignedWithTrocar[r])
    {
      // _vd[r] = Utils<float>::orthogonalProjector((offset-_xEE[r]).normalized())*_fx[r];
      _vd[r] = _fx[r];

    }
    else
    {
      // _vd[r] = 2.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
      // _vd[r].setConstant(0.0f);
      // _vd[r] = Utils<float>::orthogonalProjector((offset-_xEE[r]).normalized())*_fx[r];
      _vd[r] = _fx[r];

      // if(!_useJoystick)
      // {
      //   _vd[r]+= -2.0f*0.1*Utils<float>::deadZone(_footPosition[r](2),-5,5)/FOOT_INTERFACE_PITCH_RANGE*(_rEETrocar[r][indexMax].normalized());
      // }
      // else
      // {
      //   _vd[r] += 0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*(-_rEETrocar[r][indexMax].normalized());
      // }
    }
  }

  if(_vd[r].norm()>0.3f)
  {
    _vd[r] *= 0.3f/_vd[r].norm();
  } 
  
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

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);


  // std::cerr << "angle: " << angle << "distance: " << _fx[r].norm() << std::endl;
  std::cerr << r << ": Angle error to torcar position: " << angle << std::endl;
  std::cerr << r << ": Angle error to trocar orientation: " << std::acos(_trocarOrientation[r][indexMax].dot(_wRb[r].col(2))) << std::endl;
  std::cerr << r << ": Distance to attractor: " << _fxk[r][indexMax].norm()/4.0f << std::endl;


  // if(std::fabs(angle) < 0.05f && (_fx[r]).norm()<0.04 && _alignedWithTrocar[r] == false && fabs(bmax-1.0f)<FLT_EPSILON)
  if(std::fabs(angle) < 0.05f && std::fabs(std::acos(_trocarOrientation[r][indexMax].dot(_wRb[r].col(2))))<0.07f && _alignedWithTrocar[r] == false && fabs(bmax-1.0f)<FLT_EPSILON)
  {
    _alignedWithTrocar[r] = true;
  }

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r],5.0f);

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  std::cerr << r << ": Beliefs: " << _beliefs[r].transpose() << std::endl;
  std::cerr << r << ": Aligned with trocar: " << _alignedWithTrocar[r] << std::endl;
}


void TrocarFeetTelemanipulation::trocarAdaptation(int r)
{
  /////////////////////////////
  // Update individual tasks //
  /////////////////////////////

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
  // std::cerr << "vdfoot: " << _vdFoot[r].transpose() << std::endl;

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
    a = _adaptationRate*((_vdFoot[r]-_fx[r]).dot(_fxk[r][k]));
    b = _adaptationRate*(0.5f*(_beliefs[r](k)-0.5f)*_fxk[r][k].squaredNorm());

    _dbeliefs[r](k) = a+b;
    std::cerr << r <<": Dbeliefs " << k << ": " << a << " " << b << std::endl;
  }

  std::cerr << r << ": a: " << _dbeliefs[r].transpose() << std::endl;
  // Eigen::MatrixXf::Index indexMax;
  float dbmax = _dbeliefs[r].array().maxCoeff(&indexMax);
  // std::cerr << dbmax << std::endl;
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
  std::cerr << r << "b: " << _dbeliefs[r].transpose() << std::endl;

  _beliefs[r]+=_dt*_dbeliefs[r];
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    if(_beliefs[r](k)< 0.0f)
    {
      _beliefs[r](k) = 0.0f;
    }
    else if(_beliefs[r](k) > 1.0f)
    {
      _beliefs[r](k) = 1.0f;
    }
  }

  _fx[r].setConstant(0.0f);
  for(int k = 0; k < _nbTrocar[r]; k++)
  {
    _fx[r]+=_beliefs[r](k)*_fxk[r][k];
  }

  if(_fx[r].norm()>0.3f)
  {
    _fx[r] *=0.3f/_fx[r].norm();
  }
}

void TrocarFeetTelemanipulation::trocarInsertion(int r)
{
  std::cerr << r << ": TROCAR_INSERTION" << std::endl;

  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);
  Eigen::Vector3f offset = _sphereCenter;
  // offset << -0.5f, 0.0f, 0.0f;
  // _vd[RIGHT] = 2.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);

   // _fx[r] = _fxk[r][indexMax];


  // _vd[RIGHT] = Utils<float>::orthogonalProjector((offset-_xEE[RIGHT]).normalized())*_fx[RIGHT];
  _vd[r].setConstant(0.0f);
   // float distance = (_trocarPosition[r][indexMax]-_xEE[r]).dot(_wRb[r].col(2))-_toolOffsetFromEE[r];
  if(!_useJoystick)
  {

    // if(distance> 0.0f)
    // {
    //   _vd[r] = Utils<float>::orthogonalProjector((_sphereCenter-_xEE[r]).normalized())*_fx[r];
    // }
    // _vd[r] += -2.0f*0.1*Utils<float>::deadZone(_footPosition[r](2),-8,8)/FOOT_INTERFACE_PITCH_RANGE*(_rEETrocar[r][indexMax].normalized());
    // _vd[r] += 2.0f*0.15f*_footPosition[r](2)/FOOT_INTERFACE_Y_RANGE*(_rEETrocar[r][indexMax].normalized());
  }
  else
  {
    // if(distance>0.0f)
    // {
    //   _vd[r] = Utils<float>::orthogonalProjector((_sphereCenter-_xEE[r]).normalized())*_fx[r];
    // }
    _vd[r] += -0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*_rEETrocar[r][indexMax].normalized();
  }
  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][indexMax]));

  Eigen::Vector3f axis;  
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,axis,angle);

  std::cerr << "Angle error to trocar position: " << angle << std::endl;
  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r],5.0f);

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _wRb0[r] = _wRb[r];
  _xd0[r] = _x[r];
}


void TrocarFeetTelemanipulation::trocarSpace(int r)
{

  std::cerr << r << ": TROCAR_SPACE" << std::endl;
  Eigen::MatrixXf::Index indexMax;
  float bmax = _beliefs[r].array().maxCoeff(&indexMax);

  Eigen::Vector3f vdTool, gains;

  if(!_useJoystick)
  {
    gains.setConstant(0.0f);
    gains.setConstant(0.05f);
    gains(1) = -0.05;
    gains(2) = -0.1f;
    // gains << 2.0f*0.04f/FOOT_INTERFACE_X_RANGE, -2.0f*0.04f/FOOT_INTERFACE_Y_RANGE, -2.0f*0.15f/FOOT_INTERFACE_PITCH_RANGE;

    gains << 2.0f*0.03f/FOOT_INTERFACE_ROLL_RANGE, 2.0f*0.03f/FOOT_INTERFACE_PITCH_RANGE, 2.0f*0.15f/FOOT_INTERFACE_Y_RANGE;

    Eigen::Vector3f temp;
    temp(0) = Utils<float>::deadZone(_footPosition[r](0),-0.02f,0.02f);
    temp(1) = Utils<float>::deadZone(_footPosition[r](1),-0.02f,0.02f);
    temp(2) = Utils<float>::deadZone(_footPosition[r](2),-7.0f,7.0f);
    temp = _footPosition[r];

    // if(r==RIGHT && _mode[LEFT]==TROCAR_SPACE)
    // {
    //   vdTool = _wRb[LEFT]*(gains.cwiseProduct(temp));
    // }
    // else
    if(r==LEFT)
    {
      vdTool = _wRb[r]*(gains.cwiseProduct(temp));      
    }
    else
    {

      Eigen::Vector3f offset, xd;
      // temp  = gains.cwiseProduct(_footPosition[r]);
      // offset(2) = 0.25f*std::min(temp(2),1.0f);
      // offset(1) = offset(2)*std::tan(M_PI/6.0f*std::min(std::max(temp(1),-1.0f),1.0f));
      // offset(0) = offset(2)*std::tan(M_PI/6.0f*std::min(std::max(temp(0),-1.0f),1.0f));
      // xd = _xd0[r]+_wRb0[r]*offset;

      // gains << 2.0f/FOOT_INTERFACE_PITCH_RANGE, 2.0f/FOOT_INTERFACE_ROLL_RANGE, 2.0f/FOOT_INTERFACE_Y_RANGE;
      gains << 2.0f/FOOT_INTERFACE_X_RANGE, 2.0f/FOOT_INTERFACE_Y_RANGE, 2.0f/FOOT_INTERFACE_PITCH_RANGE;
      temp  = gains.cwiseProduct(_footPosition[r]);

      offset.setConstant(0.0f);
      offset(2) = 0.25f*std::min(temp(2),0.0f);
      if(std::fabs(offset(2))>0.03f)
      {
        offset(1) = -offset(2)*std::tan(45.0f*M_PI/180.0f*std::min(std::max(temp(1),-1.0f),1.0f));
        offset(0) = -offset(2)*std::tan(45.0f*M_PI/180.0f*std::min(std::max(temp(0),-1.0f),1.0f));        
        xd = _xd0[r]+offset;
        vdTool = 2.0f*(xd-_x[r]);
      }
      else
      {
        vdTool.setConstant(0.0f);
      }


      // if((_x[r]-_trocarPosition[r][indexMax]).norm()>0.04f)
      {
      }
      // else
      // {
      //   vdTool.setConstant(0.0f);
      // }
      if(vdTool.norm()>0.1f)
      {
        vdTool*=0.1f/vdTool.norm();
      }

      std::cerr << r <<": offset: " << offset.transpose() << std::endl; 
      std::cerr << r <<": vd: " << vdTool.transpose() << std::endl; 
      std::cerr << r <<": xd: " << xd.transpose() << std::endl; 
      std::cerr << r <<": x: " << _x[r].transpose() << std::endl; 
      
    }

  }
  else
  {
    gains.setConstant(0.05f);
    gains(1) = -0.05;
    gains(2) = -0.1f;
    if(r==RIGHT && _mode[LEFT]==TROCAR_SPACE)
    {
      vdTool = _wRb[LEFT]*(gains.cwiseProduct(_footPosition[r].segment(0,3)));
    }
    else
    {
      vdTool = _wRb[r]*(gains.cwiseProduct(_footPosition[r].segment(0,3)));
    }    

    Eigen::Vector3f offset, xd;

    offset(2) = -0.25f*std::min(_footPosition[r](2),0.0f);
    offset(0) = offset(2)*std::tan(_footPosition[r](0)*M_PI/6);
    offset(1) = offset(2)*std::tan(_footPosition[r](1)*M_PI/6);
    xd = _xd0[r]+_wRb0[r]*offset;
    vdTool = 2.0f*(xd-_x[r]);
    if(vdTool.norm()>0.1f)
    {
      vdTool*=0.1f/vdTool.norm();
    }

    std::cerr << " xd: " << xd.transpose() << std::endl; 
    std::cerr << " x: " << _x[r].transpose() << std::endl; 
  }

  // _vd[r] = 40.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax])+vdTool;
  // _vd[r] = vdTool;

  Eigen::Vector3f rEETool, rEERCM;
  rEETool = _toolOffsetFromEE[r]*_wRb[r].col(2);
  rEERCM = _xRCM[r][indexMax]-_xEE[r];

  // _vd[r] = vdTool;

  // _nullspaceWrench[r].setConstant(0.0f);
  // _nullspaceWrench[r].segment(0,3) = 40000.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);

  _omegad[r] = (rEETool-rEERCM).cross(Utils<float>::orthogonalProjector(_wRb[r].col(2))*vdTool)/(rEETool-rEERCM).squaredNorm();
 
  if(_omegad[r].norm()>1.0f)
  {
    _omegad[r]*=1.0f/_omegad[r].norm();
  }
  // _vd[r] = -_omegad[r].cross(rEETool)+2.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  // _vd[r] = vdTool;
  _vd[r] = vdTool-_omegad[r].cross(rEETool);

  if(_vd[r].norm()>0.3f)
  {
    _vd[r]*=0.3f/_vd[r].norm();
  }

  // std::cerr << "v1: " << _vd[r].transpose() << std::endl;
  // std::cerr << "v: " << _omegad[r].transpose() << std::endl;
  ///////////////////1
  // Eigen::Matrix<float,9,6> A;
  // A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2));
  // A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  // A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEETool);
  // A.block(6,0,3,3) = Eigen::Matrix3f::Identity();
  // A.block(6,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // Eigen::Matrix<float,6,1> x;
  // Eigen::Matrix<float,9,1> b;
  // b.setConstant(0.0f);
  // b.segment(3,3) = vdTool;
  // b.segment(6,3) = 5*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);

  ////////////////2
  Eigen::Matrix<float,6,6> A;
  A.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2))*Eigen::Matrix3f::Identity();
  A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(rEERCM);
  A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEETool);
  Eigen::Matrix<float,6,1> x, b;
  b.setConstant(0.0f);
  // b.segment(0,3) = 2.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  b.segment(3,3) = vdTool;

  x = A.colPivHouseholderQr().solve(b);
  // std::cerr << x.transpose() << std::endl;
  _vd[r] = x.segment(0,3);

  _omegad[r] = x.segment(3,3);

  if(_omegad[r].norm()>1.0f)
  {
    _omegad[r]*=1.0f/_omegad[r].norm();
  }

  if(_vd[r].norm()>0.3f)
  {
    _vd[r]*=0.3f/_vd[r].norm();
  }

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceWrench[r].segment(0,3) = 50.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  _nullspaceCommand[r] = 50.0f*jacobianRCM(_currentJoints[r],_trocarPosition[r][indexMax]).transpose()*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);


  std::cerr << "vd: " << _vd[r].transpose() << std::endl;
  ////////////////3
  // Eigen::Matrix<float,6,6> A;
  // A.block(0,0,3,3) = Utils<float>::getSkewSymmetricMatrix(_wRb[r].col(2));
  // A.block(0,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  // A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEETool);
  // Eigen::Matrix<float,6,1> x, b;
  // b.segment(0,3).setConstant(0.0f);
  // b.segment(3,3) = vdTool;

  // x = A.colPivHouseholderQr().solve(b);
  // // std::cerr << x.transpose() << std::endl;
  // _vd[r] = x.segment(0,3);

  // _omegad[r] = x.segment(3,3);

  ///////////////4
  // Eigen::Matrix<float,6,6> C;
  // C.block(0,0,3,3) = Utils<float>::orthogonalProjector(_wRb[r].col(2));
  // C.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // C.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  // C.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEETool);
  // Eigen::Matrix<float,3,6> A;
  // A.block(0,0,3,3) = -Eigen::Matrix3f::Identity();
  // A.block(0,3,3,3) = Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // Eigen::Vector3f b;
  // b = -100.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  // Eigen::Matrix<float,12,12> M;
  // M.setConstant(0.0f);
  // M.block(0,0,6,6) = A.transpose()*A;
  // M.block(0,6,6,6) = C.transpose();
  // M.block(6,0,6,6) = C;

  // Eigen::Matrix<float,6,1> d;
  // d.segment(0,3).setConstant(0.0f);
  // d.segment(3,3) = vdTool;
  // Eigen::Matrix<float,12,1> B;
  // B.segment(0,6) = A.transpose()*b;
  // B.segment(6,6) = d;

  // Eigen::Matrix<float,12,1> Y;
  // Y = M.colPivHouseholderQr().solve(B);
  // _vd[r] = Y.segment(0,3);
  // _omegad[r] = Y.segment(3,3);


  // Eigen::Matrix<float,3,6> C;
  // C.block(0,0,3,3) = Eigen::Matrix3f::Identity();
  // C.block(0,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(rEERCM);
  // Eigen::Matrix<float,3,6> A;
  // A.block(0,0,3,3) = -Eigen::Matrix3f::Identity();
  // A.block(0,3,3,3) = Utils<float>::getSkewSymmetricMatrix(rEETool);
  // Eigen::Vector3f b;
  // b = -vdTool;
  // Eigen::Matrix<float,9,9> M;
  // M.setConstant(0.0f);
  // M.block(0,0,6,6) = A.transpose()*A;
  // M.block(0,6,6,3) = C.transpose();
  // M.block(6,0,3,6) = C;

  // Eigen::Vector3f d;
  // d = 10.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  // Eigen::Matrix<float,9,1> B;
  // B.segment(0,6) = A.transpose()*b;
  // B.segment(6,3) = d;
  // Eigen::Matrix<float,9,1> Y;
  // Y = M.colPivHouseholderQr().solve(B);
  // std::cerr << Y.transpose() << std::endl;
  // _vd[r] = Y.segment(0,3);
  // _omegad[r] = Y.segment(3,3);

  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][indexMax]));


  Eigen::Vector3f omega;  
  float angle;
  Utils<float>::quaternionToAxisAngle(qe,omega,angle);

  std::cerr << "Angle error to trocar position: " << angle << std::endl;
  // // Compute final quaternion on plane
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

  //  Eigen::Vector3f vTemp;
  // vTemp = _wRb[r].col(0)*gains(0)*_footPosition[r](0)+_wRb[r].col(1)*gains(1)*_footPosition[r](1);
  // _vd[r] = -vTemp*(_trocarPosition[r][indexMax]-_xEE[r]).norm()/(_x[r]-_trocarPosition[r][indexMax]).norm();
  // // std::cerr << "a: " << vTemp.transpose() << std::endl;
  // // std::cerr << "b: " <<_vd[r].transpose() << std::endl;
  // _vd[r] += _wRb[r].col(2)*gains(2)*_footPosition[r](2)+10*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);

  // if(_vd[r].norm()>0.2f)
  // {
  //   _vd[r]*=0.2f/_vd[r].norm();
  // }

  // // _qd[r] = _q[r];
  // _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r],5.0f);

  _omegad[r] += Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r])+selfRotationCommand*_wRb[r].col(2);
  
  std::cerr << "omega: " <<_omegad[r].transpose() << std::endl;
  // _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r])+(rEETool-rEERCM).cross((Eigen::Matrix3f::Identity()-_wRb[r].col(2)*_wRb[r].col(2).transpose())*vdTool)/(rEETool-rEERCM).squaredNorm(); 



  // Eigen::Vector3f dir;
  // dir = _wRb[r].transpose()*_rEETrocar[r][indexMax];
  // float distance = dir.norm();
  // dir.normalize();
    
  // Eigen::Matrix3f P = Eigen::Matrix3f::Identity()-dir*dir.transpose();

    
  // Eigen::Matrix3f S;
  // S = Utils<float>::getSkewSymmetricMatrix(dir);
  
  // Eigen::Matrix<float,3,6> L;
  // L.block(0,0,3,3) = -P/distance;
  // L.block(0,3,3,3) = S;

  // Eigen::Matrix<float,6,3> Linv;
  // Linv = pseudoInverse(L);
  
  // Eigen::Vector3f sd;
  // sd << 0.0f,0.0f,1.0f;
  // // sd << _wRb[r].transpose()*zt;
  // // std::cerr << _wRb[r].transpose() << std::endl;
  // // std::cerr << "sd: " << sd.transpose() << std::endl;

  // Eigen::Matrix<float,6,1> twistB,gainsB;
  // gainsB << 10.0f*Eigen::Vector3f::Ones(), 10.0f*Eigen::Vector3f::Ones();  
  // twistB = gainsB.cwiseProduct(Linv*(sd-dir));
  // // _vd[r] += _wRb[r]*twistB.segment(0,3);
  // // _omegad[r] += _wRb[r]*twistB.segment(3,3);
  //   // _omegad[k] += _selfRotationCommand[k]*_wRb[k].col(2); 

  //   // // Compute nullspace basis
  //   Eigen::Vector3f ex, ey;
  //   ex << 1.0f, 0.0f, 0.0f;
  //   ey << 0.0f, 1.0f, 0.0f;

  //   // ex = -_wRb[k].transpose()*ex;
  //   // ey = -_wRb[k].transpose()*ey;

  //   Eigen::Matrix<float, 6,1> n1,n2,n3,n4;
  //   n1.setConstant(0.0f);
  //   n2.setConstant(0.0f);
  //   n3.setConstant(0.0f);
  //   n4.setConstant(0.0f);

  //   n1.segment(0,3) = dir;
  //   n2.segment(3,3) = dir;
  //   n3.segment(0,3) = -S*ey;
  //   n3.segment(3,3) = -P*ey/distance;
  //   n4.segment(0,3) = S*ex;
  //   n4.segment(3,3) = P*ex/distance;

  //   Eigen::MatrixXf N;
  //   N.resize(6,4);
  //   N.col(0) = n1;
  //   N.col(1) = n2;
  //   N.col(2) = n3;
  //   N.col(3) = n4;

  //  Eigen::Matrix<float,4,6> Ninv;
  //   // Ninv = pseudoInverse(N);
  //  Eigen::Matrix<float,6,6> res;
  //  res.setIdentity();
  //  Ninv = N.colPivHouseholderQr().solve(res);
  //   Eigen::Matrix<float,6,1> X,X2;
  //   X.setConstant(0.0f);
  //   // X.segment(0 = _wRb[r].transpose()*(xEEd-xEE);
  //   vdTool = gains.cwiseProduct(_footPosition[r].segment(0,3));
  //   X.segment(0,3) = vdTool;
  //   X2 = N*N.colPivHouseholderQr().solve(X);
    // std::cerr << (N.colPivHouseholderQr().solve(X)).transpose() << std::endl;
    // _vd[r] += _wRb[r]*X2.segment(0,3);
    // _omegad[r] += _wRb[r]*X2.segment(3,3);

  // Eigen::Vector4f qe;
  // qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r][indexMax]));

  // Compute final quaternion on plane
  // _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  // _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);


}





void TrocarFeetTelemanipulation::footDataTransformation()
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

void TrocarFeetTelemanipulation::positionPositionMapping()
{
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE, 2*_zPositionMapping/FOOT_INTERFACE_PITCH_RANGE;
  gains[LEFT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE, 2*_zPositionMapping/FOOT_INTERFACE_PITCH_RANGE;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _xdFoot[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_xdFoot[k](2)+_x0[k](2)<0.03f)
    {
      _xdFoot[k](2) = 0.03f-_x0[k](2);
    }
    // std::cerr << "Master position " << k << " : " <<_xdFoot[k].transpose() << std::endl;
  }
}


void TrocarFeetTelemanipulation::positionVelocityMapping()
{

  _velocityLimit = 0.3f;
  Eigen::Vector3f gains[NB_ROBOTS];
  gains[RIGHT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE, 2.0f*_velocityLimit/FOOT_INTERFACE_PITCH_RANGE;
  gains[LEFT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE, 2.0f*_velocityLimit/FOOT_INTERFACE_PITCH_RANGE;
  
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vdFoot[k] = gains[k].cwiseProduct(_footPosition[k]);
    if(_vdFoot[k].norm()>_velocityLimit)
    {
      _vdFoot[k] *= _velocityLimit/_vdFoot[k].norm();
    }    
  }
}

// void TrocarFeetTelemanipulation::trocarSelectionFootMapping()
// {

// }



void TrocarFeetTelemanipulation::trackTarget()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {

    Eigen::Vector3f offset;
    offset = _xTrocar[k];
    offset(2) -= 0.2f;

    Eigen::Vector3f gains;

    gains << 2*0.15f/FOOT_INTERFACE_Y_RANGE, 2*0.15f/FOOT_INTERFACE_X_RANGE, 2*0.15f/FOOT_INTERFACE_PITCH_RANGE;

    _xdFoot[k] = gains.cwiseProduct(_footPosition[k]);
  
    Eigen::Vector3f vdTool, rToolEE,rTrocarEE, xEE;

    vdTool = _xdFoot[k]+offset-_x[k];

    xEE = _x[k]-_toolOffsetFromEE[k]*_wRb[k].col(2);  
    rToolEE = _wRb[k].col(2)*_toolOffsetFromEE[k];
    rTrocarEE = _xTrocar[k]-xEE;

    Eigen::Vector3f xRCM, rRCMEE, vdRCM;
    xRCM = xEE+rTrocarEE.dot(_wRb[k].col(2))*_wRb[k].col(2);
    rRCMEE = xRCM-xEE;
    
    _omegad[k] = (rToolEE-rRCMEE).cross((Eigen::Matrix3f::Identity()-_wRb[k].col(2)*_wRb[k].col(2).transpose())*vdTool)/(rToolEE-rRCMEE).squaredNorm();
    if (_omegad[k].norm() > 3.0f) 
    {
      _omegad[k] = 3.0f*_omegad[k]/_omegad[k].norm();
    }
    vdRCM = 5*(_xTrocar[k]-xRCM);
    // vdRCM = 80000*(_xTrocar[k]-xRCM);
    // _nullspaceWrench[k].segment(0,3) = vdRCM;
    // vdRCM.setConstant(0.0f);

    _vd[k] = 3*vdTool-_omegad[k].cross(rToolEE)+vdRCM;

    if(_vd[k].norm()>0.4f)
    {
      _vd[k] *= 0.4f/_vd[k].norm();
    }  
    // std::cerr << k << "a: " << (_x0[k]+_xd[k]).transpose() << std::endl;
    // std::cerr << k << "v: " << _vd[k].transpose() << std::endl;
    _FdFoot[k] =  _wRb[k]*_filteredWrench[k].segment(0,3);


    std::cerr << vdTool.norm() << " " << (_xTrocar[k]-xRCM).norm() << std::endl;

        // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f w;
    Eigen::Vector3f zBd;
    zBd = (_xTrocar[k]-(_x[k]-_toolOffsetFromEE[k]*_wRb[k].col(2))).normalized();
    w = (_wRb[k].col(2)).cross(zBd);
    float c = (_wRb[k].col(2)).transpose()*zBd;  
    float s = w.norm();
    w /= s;
    
    Eigen::Matrix3f K;
    K << Utils<float>::getSkewSymmetricMatrix(w);

    Eigen::Matrix3f Re;
    if(fabs(s)< FLT_EPSILON)
    {
      Re = Eigen::Matrix3f::Identity();
    }
    else
    {
      Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    }
    
    // Convert rotation error into axis angle representation
    Eigen::Vector3f omega;
    float angle;
    Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
    Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

    // Compute final quaternion on plane
    Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[k]);

    _qd[k] = qf;


    // err_orient = rot_des_*rot_msr_.Inverse();
    // err_orient.GetQuaternion(qx,qy,qz,qw);
    // q = tf::Quaternion(qx,qy,qz,qw);

    // err_orient_axis = q.getAxis();
    // err_orient_angle = q.getAngle();

    // rotational stiffness. This is correct sign and everything!!!! do not mess with this!
    // torque_orient = err_orient_axis * err_orient_angle * (rot_stiffness);
    // // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
    // _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(5.0f*_vd[k].norm()));

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 2.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    // _nullspaceWrench[k].segment(3,3) = 
    _omegad[k] += omegaTemp; 

    // Eigen::Matrix3f R;
    // R = Utils<float>::quaternionToRotationMatrix(_qd[k])*Utils<float>::quaternionToRotationMatrix(_q[k]).inverse();
    // Eigen::Vector3f axis;
    // Utils<float>::quaternionToAxisAngle(Utils<float>::rotationMatrixToQuaternion(R),axis,angle);
    // _nullspaceWrench[k].segment(3,3) = 200*angle*axis;
  }
}


void TrocarFeetTelemanipulation::alignWithTrocar()
{

  Eigen::Vector3f d, zt, xEE, xEEd, w, rTrocarEE;
  xEE = _x[RIGHT]-_toolOffsetFromEE[RIGHT]*_wRb[RIGHT].col(2); 

  rTrocarEE = _xTrocar[RIGHT]-xEE;
  d = (_xTrocar[RIGHT]-_x[RIGHT]).normalized();
  // zt << 0.0f,0.0f,-1.0f;
  zt << 0, 0.5, -0.86603;

  xEEd = _xTrocar[RIGHT]-zt*(_toolOffsetFromEE[RIGHT]+0.1);

  d = (_xTrocar[RIGHT]-xEE).normalized();

  _vd[RIGHT] = (xEEd-xEE);

  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  w = (_wRb[RIGHT].col(2)).cross(d);
  float c = (_wRb[RIGHT].col(2)).transpose()*d;  
  float s = w.norm();
  w /= s;
  
  Eigen::Matrix3f K;
  K << Utils<float>::getSkewSymmetricMatrix(w);

  Eigen::Matrix3f Re;
  if(fabs(s)< FLT_EPSILON)
  {
    Re = Eigen::Matrix3f::Identity();
  }
  else
  {
    Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  }
  
  // Convert rotation error into axis angle representation
  Eigen::Vector3f omega;
  float angle;
  Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
  Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

  // Compute final quaternion on plane
  Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[RIGHT]);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
  // _qd[RIGHT] = Utils<float>::slerpQuaternion(_q[RIGHT],qf,1.0f-std::tanh(5.0f*_vd[RIGHT].norm()));
  _qd[RIGHT] = qf;

  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _q[RIGHT](0);
  qcurI.segment(1,3) = -_q[RIGHT].segment(1,3);
  wq = 2.0f*Utils<float>::quaternionProduct(qcurI,_qd[RIGHT]-_q[RIGHT]);
  Eigen::Vector3f omegaTemp = _wRb[RIGHT]*wq.segment(1,3);
  _omegad[RIGHT] = omegaTemp; 

  // Eigen::Matrix3f S;
  // S = Utils<float>::getSkewSymmetricMatrix(zt);

  Eigen::Vector3f dir;
  dir = _wRb[RIGHT].transpose()*rTrocarEE;
  float distance = dir.norm();
  dir.normalize();
    
  Eigen::Matrix3f P = Eigen::Matrix3f::Identity()-dir*dir.transpose();

    
  Eigen::Matrix3f S;
  S = Utils<float>::getSkewSymmetricMatrix(dir);
  
  Eigen::Matrix<float,3,6> L;
  L.block(0,0,3,3) = -P/distance;
  L.block(0,3,3,3) = S;

  Eigen::Matrix<float,6,3> Linv;
  Linv = pseudoInverse(L);
  
  Eigen::Vector3f sd;
  sd << 0.0f,0.0f,1.0f;
  // sd << _wRb[RIGHT].transpose()*zt;
  // std::cerr << _wRb[RIGHT].transpose() << std::endl;
  // std::cerr << "sd: " << sd.transpose() << std::endl;

  Eigen::Matrix<float,6,1> twistB,gains;
  gains << 1.0f*Eigen::Vector3f::Ones(), 1.0f*Eigen::Vector3f::Ones();  
  twistB = gains.cwiseProduct(Linv*(sd-dir));
  // _vd[RIGHT] += _wRb[RIGHT]*twistB.segment(0,3);
  // _omegad[RIGHT] += _wRb[RIGHT]*twistB.segment(3,3);
    // _omegad[k] += _selfRotationCommand[k]*_wRb[k].col(2); 

    // // Compute nullspace basis
    Eigen::Vector3f ex, ey;
    ex << 1.0f, 0.0f, 0.0f;
    ey << 0.0f, 1.0f, 0.0f;

    // ex = -_wRb[k].transpose()*ex;
    // ey = -_wRb[k].transpose()*ey;

    Eigen::Matrix<float, 6,1> n1,n2,n3,n4;
    n1.setConstant(0.0f);
    n2.setConstant(0.0f);
    n3.setConstant(0.0f);
    n4.setConstant(0.0f);

    n1.segment(0,3) = dir;
    n2.segment(3,3) = dir;
    n3.segment(0,3) = -S*ey;
    n3.segment(3,3) = -P*ey/distance;
    n4.segment(0,3) = S*ex;
    n4.segment(3,3) = P*ex/distance;

    Eigen::MatrixXf N;
    N.resize(6,4);
    N.col(0) = n1;
    N.col(1) = n2;
    N.col(2) = n3;
    N.col(3) = n4;

      // Eigen::Matrix<float,4,6> Ninv;
      // Ninv = pseudoInverse(N);

      // Eigen::Matrix<float,6,1> X,X2;
      // X.setConstant(0.0f);
      // X.segment(0,3) = _wRb[RIGHT].transpose()*(xEEd-xEE);
      // X2 = N*Ninv*X;
      // _vd[RIGHT] += _wRb[RIGHT]*X2.segment(0,3);
      // _omegad[RIGHT] += _wRb[RIGHT]*X2.segment(3,3);

      // _vd[RIGHT] += xEEd-xEE;

      // std::cerr <<"bou" << std::endl;

      // std::cerr << "xEE: " << xEE.transpose() << std::endl;
      // std::cerr << "xEEd: " << xEEd.transpose() << std::endl;
      // std::cerr << (_wRb[RIGHT]*X2.segment(0,3)).transpose() << std::endl;

      // Eigen::Matrix<float,6,1> X, Xtemp;
      // X.segment(0,3) = _wRb[k].transpose()*vdTemp;
      // X.segment(3,3) = _wRb[k].transpose()*omegadTemp;
      // Eigen::Vector4f q;
      // q = (N.transpose()*N).inverse()*N.transpose()*X;
      // Xtemp = N*q;
      // _vd[k]+=_wRb[k]*Xtemp.segment(0,3);
      // _omegad[k]+=_wRb[k]*Xtemp.segment(3,3);
      // svd = Eigen::JacobiSVD<Eigen::MatrixXf>(N, Eigen::ComputeFullU | Eigen::ComputeFullV);
      // singularValues = svd.singularValues();

      // tolerance = 1e-6f*std::max(N.rows(),N.cols())*singularValues.array().abs().maxCoeff();

      // Eigen::Matrix<float,4,6> Wn; 
      // Wn.setConstant(0.0f);
      // for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
      // {
      //   if(singularValues(m,0)>tolerance)
      //   {
      //     Wn(m,m) = 1.0f/singularValues(m,0);
      //   }
      //   else
      //   {
      //     Wn(m,m) = 0.0f;
      //   }
      // }
      // Eigen::Matrix<float,4,6> Ninv;
      // Ninv = svd.matrixV()*Wn*svd.matrixU().adjoint();
      // Eigen::Matrix<float,6,1> X;
      // X.segment(0,3) = _wRb[k].transpose()*(_vd[k]);
      // X.segment(3,3) = _wRb[k].transpose()*(_omegad[k]);





    // std::cerr << c << std::endl;
  // std::cerr << _omegad[RIGHT].transpose() << std::endl;
  // std::cerr << S << std::endl << std::endl;;
  // std::cerr << S.pinv() << std::endl;
  // _vd[RIGHT].setConstant(0.0f);
  // _qd[RIGHT] = _q[RIGHT];

  // for(int k = 0; k < NB_ROBOTS; k++)
  // {
  //   _vd[k] = 2.0f*(_xTrocar[k]-_x[k]);

  //   // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  //   Eigen::Vector3f w;
  //   Eigen::Vector3f zBd;
  //   zBd = (_xTrocar[k]-(_x[k]-_toolOffsetFromEE*_wRb[k].col(2))).normalized();
  //   if((int) k == LEFT)
  //   {
  //     std::cerr << _x[k].transpose() << std::endl;
  //     std::cerr << _wRb[k].col(2).transpose() << std::endl;
  //     std::cerr << zBd.transpose() << std::endl;
  //   }
  //   w = (_wRb[k].col(2)).cross(zBd);
  //   float c = (_wRb[k].col(2)).transpose()*zBd;  
  //   float s = w.norm();
  //   w /= s;
    
  //   Eigen::Matrix3f K;
  //   K << Utils<float>::getSkewSymmetricMatrix(w);

  //   Eigen::Matrix3f Re;
  //   if(fabs(s)< FLT_EPSILON)
  //   {
  //     Re = Eigen::Matrix3f::Identity();
  //   }
  //   else
  //   {
  //     Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  //   }
    
  //   // Convert rotation error into axis angle representation
  //   Eigen::Vector3f omega;
  //   float angle;
  //   Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
  //   Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

  //   // Compute final quaternion on plane
  //   Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[k]);

  //   // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
  //   _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(5.0f*_vd[k].norm()));

  //   // Compute needed angular velocity to perform the desired quaternion
  //   Eigen::Vector4f qcurI, wq;
  //   qcurI(0) = _q[k](0);
  //   qcurI.segment(1,3) = -_q[k].segment(1,3);
  //   wq = 2.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
  //   Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
  //   _omegad[k] = omegaTemp; 


  //   if(angle < 0.05f && (_xTrocar[k]-_x[k]).norm()<0.04)
  //   {
  //     // _alignedWithTrocar[k] = true;
  //   }

  //   std::cerr << "[RobotsTaskGeneration]: " << k << " position error: " << _vd[k].norm() << " angular error: " << angle << std::endl;
  //   _xd[k] = _x[k];
  // }
}


void TrocarFeetTelemanipulation::computeDesiredFootWrench()
{
  // temp.setConstant(0.0f);
  _desiredFootWrench[RIGHT].setConstant(0.0f);
  _desiredFootWrench[RIGHT](1) = _FdFoot[RIGHT](0);
  _desiredFootWrench[RIGHT](0) = -_FdFoot[RIGHT](1);
  _desiredFootWrench[RIGHT](2) = _FdFoot[RIGHT](2)*0.2;
  _desiredFootWrench[RIGHT](0) += -_kxy*_footPose[RIGHT](0)-_dxy*_footTwist[RIGHT](0);
  _desiredFootWrench[RIGHT](1) += -_kxy*_footPose[RIGHT](1)-_dxy*_footTwist[RIGHT](1);
  _desiredFootWrench[RIGHT](2) += -_kphi*_footPose[RIGHT](2)-_dphi*_footTwist[RIGHT](2);
  _desiredFootWrench[RIGHT](3) += -_kphi*_footPose[RIGHT](3)-_dphi*_footTwist[RIGHT](3);
  _desiredFootWrench[RIGHT](4) += -_kphi*_footPose[RIGHT](4)-_dphi*_footTwist[RIGHT](4);

  _desiredFootWrench[LEFT].setConstant(0.0f);
  _desiredFootWrench[LEFT](1) = _FdFoot[LEFT](0);
  _desiredFootWrench[LEFT](0) = -_FdFoot[LEFT](1);
  _desiredFootWrench[LEFT](2) = _FdFoot[LEFT](2)*0.2;
  _desiredFootWrench[LEFT](0) += -_kxy*_footPose[LEFT](0)-_dxy*_footTwist[LEFT](0);
  _desiredFootWrench[LEFT](1) += -_kxy*_footPose[LEFT](1)-_dxy*_footTwist[LEFT](1);
  _desiredFootWrench[LEFT](2) += -_kphi*_footPose[LEFT](2)-_dphi*_footTwist[LEFT](2);
  _desiredFootWrench[LEFT](3) += -_kphi*_footPose[LEFT](3)-_dphi*_footTwist[LEFT](3);
  _desiredFootWrench[LEFT](4) += -_kphi*_footPose[LEFT](4)-_dphi*_footTwist[LEFT](4);
}


void TrocarFeetTelemanipulation::computeDesiredOrientation()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Matrix3f Rd;
    if(k == RIGHT)
    {
      Rd << -1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f;
    }
    else
    {
      Rd << -1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, -1.0f,
            0.0f, -1.0f, 0.0f;
    }

    Eigen::Matrix3f Rtemp;
    float phi = -_footPose[k](ROLL)*M_PI/180.0f;
    float theta = _footPose[k](YAW)*M_PI/180.0f;
    float psi = 0.0f;
    float cphi = std::cos(phi);
    float sphi = std::sin(phi);
    float ctheta = std::cos(theta);
    float stheta = std::sin(theta);
    float cpsi = std::cos(psi);
    float spsi = std::sin(psi);
    Rtemp << cpsi*ctheta, cpsi*stheta*sphi-spsi*cphi, cpsi*stheta*cphi+spsi*sphi,
             spsi*ctheta, spsi*stheta*sphi+cpsi*cphi, spsi*stheta*cphi-cpsi*sphi,
             -stheta, ctheta*sphi, ctheta*cphi;
    Rd = Rd*Rtemp;

    _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

    if(_q[k].dot(_qd[k])<0)
    {
      _qd[k] *=-1.0f;
    }

    //   Rd << -1.0f, 0.0f, 0.0f,
    //         0.0f, 1.0f, 0.0f,
    //         0.0f, 0.0f, -1.0f;
    // Eigen::Matrix3f Rx;
    // float angle = _footPose[RIGHT](4)*M_PI/180.0f;
    // Rx << 1.0f, 0.0f, 0.0f,
    //       0.0f, cos(angle), -sin(angle),
    //       0.0f, sin(angle), cos(angle);

    // Rd = Rx*Rd;
    // std::cerr << Rd << std::endl;

    // _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

    // if(_q[k].dot(_qd[k])<0)
    // {
    //   _qd[k] *=-1.0f;
    // }


     // _qd[k] << 0.0f,0.0f,1.0f,0.0f;

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 
  }
}


void TrocarFeetTelemanipulation::logData()
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


void TrocarFeetTelemanipulation::publishData()
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

    geometry_msgs::Wrench wrench;
    wrench.force.x = _nullspaceWrench[k](0);
    wrench.force.y = _nullspaceWrench[k](1);
    wrench.force.z = _nullspaceWrench[k](2);
    wrench.torque.x = _nullspaceWrench[k](3);
    wrench.torque.y = _nullspaceWrench[k](4);
    wrench.torque.z = _nullspaceWrench[k](5);
    _pubDesiredWrench[k].publish(wrench);

    _msgNullspaceCommand.data.resize(7);
    for(int m = 0; m < 7; m++)
    {
      _msgNullspaceCommand.data[m] = _nullspaceCommand[k](m);
    }
    _pubNullspaceCommand[k].publish(_msgNullspaceCommand);

  }
}


void TrocarFeetTelemanipulation::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
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


void TrocarFeetTelemanipulation::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void TrocarFeetTelemanipulation::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
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
      std::cerr << "[TrocarFeetTelemanipulation]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
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


void TrocarFeetTelemanipulation::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void TrocarFeetTelemanipulation::updateFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg, int k)
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


void TrocarFeetTelemanipulation::updateJoystick(const sensor_msgs::Joy::ConstPtr& joy, int k)
{

  _footPose[k].setConstant(0.0f);
  _footPose[k](X) = joy->axes[0];
  _footPose[k](Y) = joy->axes[1];
  _footPose[k](PITCH) = joy->axes[3];
  _footPose[k](YAW) = joy->axes[2];
  // _footPose[k](PITCH) = joy->axes[4];
  
  if(!_firstJoystick[k])
  {
    _firstJoystick[k]= true;
  }
}

void TrocarFeetTelemanipulation::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k) 
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



void TrocarFeetTelemanipulation::dynamicReconfigureCallback(robotic_experiments::feetTelemanipulation_paramsConfig &config, uint32_t level)
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

Eigen::MatrixXf TrocarFeetTelemanipulation::pseudoInverse(const Eigen::MatrixXf &M_, bool damped)
{ 
  double lambda_ = damped?0.2:0.0;

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXf S_ = M_; // copying the dimensions of M_, its content is not needed.
  S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

  Eigen::MatrixXf Mpinv;
  Mpinv = Eigen::MatrixXf(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
  return Mpinv;
}