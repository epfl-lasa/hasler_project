#include "SurgicalTask.h"
#include "Utils.h"
#include <chrono>
#include <thread>
#include "gazebo_msgs/SetModelState.h"
#include "qpSolver.h"


SurgicalTask* SurgicalTask::me = NULL;

SurgicalTask::SurgicalTask(ros::NodeHandle &n, double frequency):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{
  me = this;

  _gravity << 0.0f, 0.0f, -9.80665f;
  _toolComPositionFromSensor << 0.0f,0.0f,0.02f;
  _toolOffsetFromEE[LEFT] = 0.4f;
  _toolOffsetFromEE[RIGHT] = 0.4f;
  _toolMass = 0.2f;

  _useRobot[LEFT] = true;
  _useRobot[RIGHT] = true;
  _mapping[LEFT] = POSITION_VELOCITY;
  _mapping[RIGHT] = POSITION_VELOCITY;
  _useSim = true;
  _useJoystick = true;
  _sphericalTrocarId[LEFT] = 14;
  _sphericalTrocarId[RIGHT] = 23;

  for(int k= 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    
    _xd[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _qdPrev[k].setConstant(0.0f);

    _firstRobotPose[k] = false;
    _firstRobotTwist[k] = false;
    _alignedWithTrocar[k] = false;
    _xdOffset[k].setConstant(0.0f);
    _joyAxes[k].setConstant(0.0f);
    _selfRotationCommand[k] = 0.0f;

    _footPose[k].setConstant(0.0f);
    _footWrench[k].setConstant(0.0f);
    _footTwist[k].setConstant(0.0f);
    // _footDesiredWrench[k].setConstant(0.0f);
    _footPosition[k].setConstant(0.0f);
    _firstFootOutput[k] = false;
    _vdOffset[k].setConstant(0.0f);
    _sequenceID[k] = 0;
    _joystickSequenceID[k] = 100;
    _desiredFootWrench[k].setConstant(0.0f);
    _firstRobotBaseFrame[k] = false;
    _firstSphericalTrocarFrame[k] = false;
    _xRobotBaseOrigin[k].setConstant(0.0f);
    _qRobotBaseOrigin[k] << 1.0f, 0.0f, 0.0f, 0.0f;
    _D[k].setConstant(0.0f);

    _robotMode[k] = TROCAR_SELECTION;
    _ikJoints[k].resize(7);
    _currentJoints[k].resize(7);
  }


  _pillarsId.resize(2);
  // _pillarsId << 1, 3, 11, 10;
  _pillarsId << 1, 3;

  int nbTasks = _pillarsId.size();

  if(_useRobot[RIGHT])
  {
    nbTasks += 1;
  }
  _beliefsC.resize(nbTasks);
  _dbeliefsC.resize(nbTasks);
  for(int k = 0; k < nbTasks; k++)
  {
    if(!(_useRobot[RIGHT] && k == nbTasks-1))
    {
      _firstPillarsFrame[k] = false;
    }
    _beliefsC(k) = 0.0f;
    _dbeliefsC(k) = 0.0f;
  }

  _beliefsC(0) = 1.0f;

  _pillarsPosition.resize(_pillarsId.size(),3);

  _pillarsPosition.setConstant(0.0f);
  _stop = false;
  
  // _strategy = PRIMARY_TASK;
  _strategy = VIRTUAL_RCM;
  _humanInput = FOOT;

}


bool SurgicalTask::init() 
{
  if(_useRobot[LEFT])
  {
    // Subscriber definitions
    _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/left_lwr/ee_pose", 1, boost::bind(&SurgicalTask::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/left_lwr/ee_vel", 1, boost::bind(&SurgicalTask::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subCurrentJoints[LEFT] = _nh.subscribe<sensor_msgs::JointState>("/left_lwr/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[LEFT] = _nh.subscribe<std_msgs::Float32MultiArray>("/left_lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SurgicalTask::updateDampingMatrix,this,_1,LEFT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    
    if(!_useSim)
    {
      _subForceTorqueSensor[LEFT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_left/netft_data", 1, boost::bind(&SurgicalTask::updateRobotWrench,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
  
    if(_useJoystick)
    {
      _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Left",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    // Publisher definitions
    _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/left_lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/left_lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[LEFT] = _nh.advertise<geometry_msgs::Wrench>("/left_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[LEFT] = _nh.advertise<geometry_msgs::WrenchStamped>("SurgicalTask/filteredWrenchLeft", 1);
    _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Left", 1);
    _pubNullspaceCommand[LEFT] = _nh.advertise<std_msgs::Float32MultiArray>("/left_lwr/joint_controllers/passive_ds_command_nullspace", 1);
    // _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/command_joint_pos", 1);
    _pubDesiredJoints[LEFT] = _nh.advertise<std_msgs::Float64MultiArray>("left_lwr/joint_controllers/passive_ds_nullspace_joints", 1);
  }

  if(_useRobot[RIGHT])
  {
    _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/right_lwr/ee_pose", 1, boost::bind(&SurgicalTask::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/right_lwr/ee_vel", 1, boost::bind(&SurgicalTask::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    _subCurrentJoints[RIGHT] = _nh.subscribe<sensor_msgs::JointState>("/right_lwr/joint_states", 1, boost::bind(&SurgicalTask::updateCurrentJoints,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
    _subDampingMatrix[RIGHT] = _nh.subscribe<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_damping_matrix", 1, boost::bind(&SurgicalTask::updateDampingMatrix,this,_1,RIGHT),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
   
    if(!_useSim)
    {
      _subForceTorqueSensor[RIGHT] = _nh.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor_right/netft_data", 1, boost::bind(&SurgicalTask::updateRobotWrench,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
   
    if(_useJoystick)
    {
      _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right/joy",1, boost::bind(&SurgicalTask::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }
    else
    {
      _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg_v2>("/FI_Output/Right",1, boost::bind(&SurgicalTask::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    }

    _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/right_lwr/joint_controllers/passive_ds_command_vel", 1);
    _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/right_lwr/joint_controllers/passive_ds_command_orient", 1);
    _pubDesiredWrench[RIGHT] = _nh.advertise<geometry_msgs::Wrench>("/right_lwr/joint_controllers/passive_ds_command_force", 1);
    _pubFilteredWrench[RIGHT] = _nh.advertise<geometry_msgs::WrenchStamped>("SurgicalTask/filteredWrenchRight", 1);
    _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg_v2>("/FI_Input/Right", 1);
    _pubNullspaceCommand[RIGHT] = _nh.advertise<std_msgs::Float32MultiArray>("/right_lwr/joint_controllers/passive_ds_command_nullspace", 1);
    // _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/command_joint_pos", 1);
    _pubDesiredJoints[RIGHT] = _nh.advertise<std_msgs::Float64MultiArray>("right_lwr/joint_controllers/passive_ds_nullspace_joints", 1);

  }

  // _subSphericalTrocars = _nb.subscribe<std_msgs::Float32MultiArray>("/spherical_trocar_frames")

  signal(SIGINT,SurgicalTask::stopNode);

  // _startThread = true;
  // if(pthread_create(&_thread, NULL, &SurgicalTask::startIkLoop, this))
  // {
  //     throw std::runtime_error("Cannot create reception thread");  
  // }

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[SurgicalTask]: The SurgicalTask node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[SurgicalTask]: The SurgicalTask node has a problem.");
    return false;
  }
}

void SurgicalTask::run()
{
  while(!_stop) 
  {
    if(allSubscribersOK() && allFramesOK())
    {
      // Check for update of the DS-impedance controller gain
      ros::param::getCached("/left_lwr/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);
      ros::param::getCached("/right_lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
          
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      // Log data
      logData();
    }
    else
    {
      if(!allFramesOK())
      {
        receiveFrames();
      }
    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  // _startThread = false;
  // pthread_join(_thread,NULL);

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


void SurgicalTask::stopNode(int sig)
{
  me->_stop = true;
}


bool SurgicalTask::allSubscribersOK()
{
  bool robotStatus[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    robotStatus[k] = !_useRobot[k] || (_firstRobotPose[k] && _firstRobotTwist[k]
                      // && _firstDampingMatrix[k] 
                      && _firstJointsUpdate[k] 
                      && (_firstFootOutput[k] || _firstJoystick[k])
                      && (_useSim || _wrenchBiasOK[k]));

    if(!robotStatus[k])
    {
      std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
                << " pose: " << _firstRobotPose[k] << " twist: " << _firstRobotTwist[k]
                << " damp: " << _firstDampingMatrix[k] << " joints: " << _firstJointsUpdate[k]
                << " foot/joy: " << (_firstFootOutput[k] || _firstJoystick[k])
                << " sim/wrench: " << (_useSim || _wrenchBiasOK[k]) << std::endl;
    }
  }
  
  return robotStatus[LEFT] && robotStatus[RIGHT];
}

bool SurgicalTask::allFramesOK()
{
  bool frameStatus[NB_ROBOTS];

  for(int k = 0; k < NB_ROBOTS; k++)
  {

    bool pillarsStatus = true;
    if(k == 0)
    {
      for(int m = 0; m < _pillarsId.size(); m++)
      {
        pillarsStatus = pillarsStatus && _firstPillarsFrame[m];
      }
    }

    frameStatus[k] = !_useRobot[k] || (_firstRobotBaseFrame[k] && _firstSphericalTrocarFrame[k] && pillarsStatus);

    if(!frameStatus[k])
    {
      std::cerr << k << ": Status: " << "not use: " << !_useRobot[k] 
                << " robot base: " << _firstRobotBaseFrame[k] 
                << " trocar : " << _firstSphericalTrocarFrame[k] << " pillars: " << pillarsStatus << std::endl;
    }
  }

  return frameStatus[LEFT] && frameStatus[RIGHT]; 
}

void SurgicalTask::receiveFrames()
{

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(_useRobot[k])
    {      
      try
      { 
        if(!_firstRobotBaseFrame[k])
        {
          if(k==LEFT)
          {
            _lr.waitForTransform("/world", "/left_lwr_base_link", ros::Time(0), ros::Duration(3.0));
            _lr.lookupTransform("/world", "/left_lwr_base_link", ros::Time(0), _transform);        
          }
          else
          {
            _lr.waitForTransform("/world", "/right_lwr_base_link", ros::Time(0), ros::Duration(3.0));
            _lr.lookupTransform("/world", "/right_lwr_base_link", ros::Time(0), _transform); 
          }
          _xRobotBaseOrigin[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
          _qRobotBaseOrigin[k] << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
          _firstRobotBaseFrame[k] = true;
          std::cerr << "[SurgicalTask]: Robot " << k << " origin received: " << _xRobotBaseOrigin[k].transpose() << std::endl;
        } 
      } 
      catch (tf::TransformException ex)
      {
      }
    }

    if(!_firstSphericalTrocarFrame[k])
    {
      try
      { 

        _lr.waitForTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "f" + std::to_string(_sphericalTrocarId[k]), ros::Time(0), _transform);        
        _trocarPosition[k] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
        Eigen::Vector4f temp;
        temp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstSphericalTrocarFrame[k] = true;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " origin received: " << _trocarPosition[k].transpose() << std::endl;
        std::cerr << "[SurgicalTask]: Spherical trocar for robot " << k << " orientation received: " << _trocarOrientation[k].transpose() << std::endl;
      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }

  for(int k = 0; k < _pillarsId.size(); k++)
  {
    if(_firstSphericalTrocarFrame[LEFT] && !_firstPillarsFrame[k])
    {    
      try
      { 
        _lr.waitForTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), ros::Duration(3.0));
        _lr.lookupTransform("/world", "p" + std::to_string(_pillarsId[k]), ros::Time(0), _transform);        
        _pillarsPosition.row(k) = Eigen::Vector3f(_transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z());
        Eigen::Vector4f q;
        q << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
        // _trocarOrientation[k] = -Utils<float>::quaternionToRotationMatrix(temp).col(2);
        _firstPillarsFrame[k] = true;
        std::cerr << "[SurgicalTask]: Pillars " << k << " origin received: " << _pillarsPosition.row(k).transpose() << std::endl;


        ros::ServiceClient client = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        //position
        geometry_msgs::Point linkPosition;
        linkPosition.x = _pillarsPosition(k,0);
        linkPosition.y = _pillarsPosition(k,1);
        linkPosition.z = _pillarsPosition(k,2);
        //orientation
        geometry_msgs::Quaternion linkOrientation;

        Eigen::Vector4f qe, qd;
        Eigen::Vector3f z, zd;
        z = Utils<float>::quaternionToRotationMatrix(q).col(2);
        zd = (_trocarPosition[LEFT]-_pillarsPosition.row(k).transpose()).normalized();
        qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(z,zd));
        qd = Utils<float>::quaternionProduct(qe,q);

        // Compute final quaternion on plane
        linkOrientation.x = qd(1);
        linkOrientation.y = qd(2);
        linkOrientation.z = qd(3);
        linkOrientation.w = qd(0);

        //pose (Pose + Orientation)
        geometry_msgs::Pose modelPose;
        modelPose.position = linkPosition;
        modelPose.orientation = linkOrientation;

        //ModelState
        gazebo_msgs::ModelState modelState;
        modelState.model_name = (std::string) "target"+std::to_string(k+1);
        modelState.pose = modelPose;

        gazebo_msgs::SetModelState srv;
        srv.request.model_state = modelState;

        if(client.call(srv))
        {
            ROS_INFO("Set object pose");
        }
        else
        {
            ROS_ERROR("Reset frame pose! Error msg:%s",srv.response.status_message.c_str());
        }

      } 
      catch (tf::TransformException ex)
      {
      }
    }
  }


}



void SurgicalTask::computeCommand()
{
  humanInputTransformation();

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
}


void SurgicalTask::updateTrocarInformation(int r)
{
  _rEETrocar[r] = _trocarPosition[r]-_xEE[r];
  _xRCM[r] = _xEE[r]+(_trocarPosition[r]-_xEE[r]).dot(_wRb[r].col(2))*_wRb[r].col(2);
  _rEERCM[r] = _xRCM[r]-_xEE[r];

  _xdEE[r] = _trocarPosition[r]-_rEETrocar[r].dot(_trocarOrientation[r])*_trocarOrientation[r];

  // Linear DS to go to the attractor
  _fxk[r]= 4.0f*(_xdEE[r]-_xEE[r]); 
  std::cerr << "[SurgicalTask]: " << r << ": Distance to attractor: " << (_xdEE[r]-_xEE[r]).norm() << std::endl;
}



void SurgicalTask::selectRobotMode(int r)
{
  _robotMode[r] = TROCAR_SELECTION;

  if(_alignedWithTrocar[r]==true)
  {
    float distance = (_trocarPosition[r]-_xEE[r]).dot(_wRb[r].col(2))-_toolOffsetFromEE[r];
    std::cerr << "[SurgicalTask]: " << r << ": Distance tool-trocar: " << distance << std::endl;
    std::cerr << "[SurgicalTask]: " << r << ": Distance RCM-trocar: " << (_trocarPosition[r]-_xRCM[r]).norm() <<std::endl;
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


void SurgicalTask::trocarSelection(int r)
{
  std::cerr << "[SurgicalTask]: " << r << ": TROCAR SELECTION" << std::endl;


  _fx[r] = _fxk[r];

  // // Find max belief
  // Eigen::MatrixXf::Index indexMax;
  // float bmax = _beliefs[r].array().maxCoeff(&indexMax);


  if(!_alignedWithTrocar[r])
  {
    _vd[r] = _fx[r];
  }
  else
  {
    _vd[r] = _fx[r];

    if(!_useJoystick)
    {
      // _vd[r]+= -2.0f*0.1*Utils<float>::deadZone(_footPosition[r](2), -5, 5) / FOOT_INTERFACE_PITCH_RANGE
      //          *(_rEETrocar[r].normalized());
      float vi;
      vi = 0.15f*Utils<float>::deadZone(_footPosition[r](2)/(FOOT_INTERFACE_Y_RANGE/2.0f), -0.1f, 0.1f);
      _vd[r]+= vi*(_rEETrocar[r].normalized());
      std::cerr << "vi: " << vi << std::endl;
    }
    else
    {
      _vd[r] += 0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*_rEETrocar[r].normalized();
    }
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.3f);
  
  Eigen::Vector4f qe;
  
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

  Eigen::Vector3f axis;  
  float angleErrorToTrocarPosition;
  Utils<float>::quaternionToAxisAngle(qe, axis, angleErrorToTrocarPosition);

  if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                     -MAX_ORIENTATION_ERROR,
                                                                     MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe, _q[r]);

  float angleErrorToTrocarOrientation = std::acos(_trocarOrientation[r].dot(_wRb[r].col(2)));

  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;
  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarOrientation: " << angleErrorToTrocarOrientation << std::endl;
  // std::cerr << "[SurgicalTask]: " << r << ": Distance to attractor: " << _fx[r].norm()/4.0f << std::endl;


  if(std::fabs(angleErrorToTrocarPosition) < 0.05f && 
     std::fabs(angleErrorToTrocarOrientation)< 0.07f && 
     _alignedWithTrocar[r] == false)
  {
    _alignedWithTrocar[r] = true;
  }

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r], _qd[r]);

  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  // std::cerr << r << ": Beliefs: " << _beliefs[r].transpose() << std::endl;
  std::cerr << "[SurgicalTask]: " << r << ": Aligned with trocar: " << _alignedWithTrocar[r] << std::endl;
}


void SurgicalTask::trocarInsertion(int r)
{
  std::cerr << "[SurgicalTask]: " << r << ": TROCAR INSERTION" << std::endl;

  // _vd[r] = _fxk[r];
  _vd[r].setConstant(0.0f);

  if(_useJoystick)
  {

    _vd[r] += 0.1*Utils<float>::deadZone(_footPosition[r](2),-0.1f,0.1f)*_rEETrocar[r].normalized();
  }
  else
  {
    if(r==LEFT)
    {
      // _vd[r] += 2.0f*0.15f*_footPosition[r](2)/FOOT_INTERFACE_Y_RANGE*_wRb[r].col(2);
      float vi;
      vi = 0.15f*Utils<float>::deadZone(_footPosition[r](2)/(FOOT_INTERFACE_Y_RANGE/2.0f), -0.1f, 0.1f);
      _vd[r]+= vi*(_rEETrocar[r].normalized());
      std::cerr << "vi: " << vi << " vd: " << _vd[r].transpose() << std::endl;

    }
  }

  _vd[r] = Utils<float>::bound(_vd[r],0.3f);


  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));

  Eigen::Vector3f axis;  
  float angleErrorToTrocarPosition;
  Utils<float>::quaternionToAxisAngle(qe, axis, angleErrorToTrocarPosition);

  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;


  if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                     -MAX_ORIENTATION_ERROR,
                                                                     MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  _omegad[r] = Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r]);


  _nullspaceWrench[r].setConstant(0.0f);
  _nullspaceCommand[r].setConstant(0.0f);

  _wRb0[r] = _wRb[r];
  _xd0[r] = _x[r];
  _inputAlignedWithOrigin[r] = false;
  _xdTool[r] = _x[r];
  bou.setConstant(0.0f);
  // _desiredOffset[r].setConstant(0.0f);
  // _vdToolPast[r].setConstant(0.0f);
  // _vdToolFiltPast[r].setConstant(0.0f);

}


void SurgicalTask::trocarSpace(int r)
{

  std::cerr << "[SurgicalTask]: " << r << ": TROCAR SPACE" << std::endl;

  Eigen::Vector3f vdTool, gains, temp;

  if(!_useJoystick)
  {

    if(_mapping[r] == POSITION_VELOCITY)
    {
      gains << 2.0f*0.03f/FOOT_INTERFACE_PITCH_RANGE,
               2.0f*0.03f/FOOT_INTERFACE_ROLL_RANGE,
               2.0f*0.15f/FOOT_INTERFACE_Y_RANGE;

      temp(0) = Utils<float>::deadZone(_footPosition[r](0),-5.0f,5.0f);
      temp(1) = Utils<float>::deadZone(_footPosition[r](1),-5.0f,5.0f);
      temp(2) = Utils<float>::deadZone(_footPosition[r](2),-0.03f,0.03f);


      vdTool = _wRb[r]*(gains.cwiseProduct(temp));      


      if(r==LEFT)
      {
        pillarsAdaptation(r);
        vdTool = gains(2)*temp(2)*_wRb[r].col(2)+_vdC;
        std::cerr << "vz: " << gains(2)*temp(2) << " " <<  vdTool.dot((_wRb[r].col(2))) << std::endl;
      }


    }
    else if(_mapping[r]==POSITION_POSITION)
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

    if(_mapping[r]==POSITION_VELOCITY)
    {
      gains << 0.03f, 0.03f, 0.15f;
      Eigen::Vector3f temp;
      temp = gains.cwiseProduct(_footPosition[r].segment(0,3));
      // vdTool = _wRb[r]*(gains.cwiseProduct(_footPosition[r].segment(0,3)));
      if(_useSim)
      {
        vdTool = temp(0)*_wRb[r].col(1)+temp(1)*_wRb[r].col(0)+temp(2)*_wRb[r].col(2);
      }
      else
      {
        vdTool = _wRb[r]*temp;
      }


      if(r==LEFT)
      {
        pillarsAdaptation(r);
        vdTool = temp(2)*_wRb[r].col(2)+_vdC;
      }

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      // bool res = _qpSolverRCM.step(_ikJoints[r], _currentJoints[r], _trocarPosition[r],
      // _toolOffsetFromEE[r], _x[r], _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r]);

      _ikJoints[r] = qpSolver(_currentJoints[r], _trocarPosition[r],
      _toolOffsetFromEE[r], _x[r], _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r]);

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      // Eigen::VectorXd joints;
      // bool res = _cvxgenSolverRCM.step(joints, _currentJoints[r].cast<double>(), _trocarPosition[r].cast<double>(),
      // (double)_toolOffsetFromEE[r], _x[r].cast<double>(), (double) _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r].cast<double>());
      // _ikJoints[r] = joints.cast<float>();
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // std::cerr << "[SurgicalTask]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": error tool : " << (_x[r]-_xdTool[r]).norm() << std::endl; 
      // std::cerr << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      std::cerr << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;
      std::cerr << r << ": Error joints: " << (_ikJoints[r]-_currentJoints[r]).norm() << std::endl;
      // std::cerr << (int) res <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      std::cerr <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      // std::cerr << (int) res <<  " Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // // std::cerr << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // std::cerr <<  "xd: " << _xdTool[r].transpose() <<  "x: " << _x[r].transpose() << std::endl;

    }
    else if(_mapping[r]==POSITION_POSITION)
    {
      Eigen::Vector3f xd, desiredOffset, currentOffset;

      desiredOffset(2) = -0.2f*std::max(_footPosition[r](2),0.0f);
      desiredOffset(1) = desiredOffset(2)*std::tan(_footPosition[r](1)*45.0f*M_PI/180.0f);
      desiredOffset(0) = desiredOffset(2)*std::tan(_footPosition[r](0)*45.0f*M_PI/180.0f);

      currentOffset = _x[r]-_xd0[r];

      // if((desiredOffset-currentOffset).norm()<0.02f && _inputAlignedWithOrigin[r]==false)
      // {
      //   std::cerr << "Bring input to current robot position" << std::endl;
      //   _inputAlignedWithOrigin[r] = true;
      //   vdTool.setConstant(0.0f);
      // }
      // else
      {
        // if(currentOffset.norm()<0.01f && desiredOffset.norm()<0.01f)
        // {
        //   vdTool.setConstant(0.0f);
        // }
        // else
        {
          xd = _xd0[r]+desiredOffset;
          // std::cerr << desiredOffset.transpose() << std::endl;
          // std::cerr << _xd0[r] << std::endl;
          // xd(2) -= 0.01f;
          vdTool = 2.0f*(xd-_x[r])+0.15f*std::min(_footPosition[r](2),0.0f)*_wRb[r].col(2);          
          // _xdTool[r] += vdTool*_dt; 
          // _xdTool[r][0] =  Utils<float>::bound(_xdTool[r](0),_xd0[r](0)-0.1,_xd0[r](0)+0.1f);
          // _xdTool[r][1] =  Utils<float>::bound(_xdTool[r](1),_xd0[r](1)-0.1,_xd0[r](1)+0.1f);
          // _xdTool[r][2] =  Utils<float>::bound(_xdTool[r](2),_xd0[r](2)-0.2,_xd0[r](2)+0.2f);
          // _xdTool[r] = xd; 

        }
      }
      // Eigen::Matrix<float, 7, 1> joints;

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

      // bool res = _qpSolverRCM.step(_ikJoints[r], _currentJoints[r], _trocarPosition[r],
      // _toolOffsetFromEE[r], _xdTool[r], _currentJoints[r](6), 0.1f, _xRobotBaseOrigin[r]);
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // Eigen::VectorXd joints;
      // bool res = _cvxgenSolverRCM.step(joints, _currentJoints[r].cast<double>(), _trocarPosition[r].cast<double>(),
      // (double)_toolOffsetFromEE[r], _x[r].cast<double>(), (double) _currentJoints[r](6), 1.0f, _xRobotBaseOrigin[r].cast<double>());
      // _ikJoints[r] = joints.cast<float>();
      // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      // std::cerr << "[SurgicalTask]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
      // std::cerr << "[SurgicalTask]: " << r <<": error tool : " << (_x[r]-_xdTool[r]).norm() << std::endl; 
      // std::cerr << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
      // // std::cerr << r << ": Desired joints: " << _ikJoints[r].transpose() << std::endl;
      // std::cerr << r << ": Error joints: " << (_ikJoints[r]-_currentJoints[r]).norm() << std::endl;
      // std::cerr << (int) res <<  " Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
      // std::cerr << (int) res <<  " Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // // std::cerr << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
      // std::cerr <<  "xd: " << _xdTool[r].transpose() <<  "x: " << _x[r].transpose() << std::endl;
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
  A.block(0,3,3,3) = -Utils<float>::orthogonalProjector(_wRb[r].col(2))*Utils<float>::getSkewSymmetricMatrix(_rEERCM[r]);
  A.block(3,0,3,3) = Eigen::Matrix3f::Identity();
  A.block(3,3,3,3) = -Utils<float>::getSkewSymmetricMatrix(_toolOffsetFromEE[r]*_wRb[r].col(2));
  Eigen::Matrix<float,6,1> x, b;
  b.setConstant(0.0f);
  b.segment(3,3) = vdTool;

  x = A.fullPivHouseholderQr().solve(b);
  _vd[r] = x.segment(0,3);

  if(_useSim)
  {
    // bou = 0.9*bou+0.1*1000.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_trocarPosition[r]-_xRCM[r]);
    // _vd[r]+=bou;

    _vd[r]+=20.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_trocarPosition[r]-_xRCM[r]);
  }
  _vd[r] = Utils<float>::bound(_vd[r],0.3f);

  _omegad[r] = x.segment(3,3);
  _omegad[r] = Utils<float>::bound(_omegad[r],1.0f);

  _nullspaceWrench[r].setConstant(0.0f);
  // _nullspaceWrench[r].segment(0,3) = 50.0f*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);
  // _nullspaceCommand[r] = 50.0f*jacobianRCM(_currentJoints[r],_trocarPosition[r][indexMax]).transpose()*(_trocarPosition[r][indexMax]-_xRCM[r][indexMax]);



  Eigen::Vector4f qe;
  qe = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(_wRb[r].col(2),_rEETrocar[r]));


  Eigen::Vector3f axis;  
  float angleErrorToTrocarPosition;
  Utils<float>::quaternionToAxisAngle(qe,axis,angleErrorToTrocarPosition);

  std::cerr << "[SurgicalTask]: " << r << ": angleErrorToTrocarPosition: " << angleErrorToTrocarPosition << std::endl;

  if(std::fabs(angleErrorToTrocarPosition)>MAX_ORIENTATION_ERROR)
  {
    qe = Utils<float>::axisAngleToQuaterion(axis,Utils<float>::bound(angleErrorToTrocarPosition,
                                                                     -MAX_ORIENTATION_ERROR,
                                                                     MAX_ORIENTATION_ERROR));
  }

  // Compute final quaternion on plane
  _qd[r] = Utils<float>::quaternionProduct(qe,_q[r]);

  float selfRotationCommand = 0.0f;
  if(!_useJoystick)
  {
    selfRotationCommand = -2.0f*1.0f*Utils<float>::deadZone(_footPose[r](YAW),-5.0f,5.0f)/FOOT_INTERFACE_YAW_RANGE;
  }
  else
  {
    selfRotationCommand = 1.0f*_footPose[r](YAW);
  }

  _omegad[r] += Utils<float>::quaternionToAngularVelocity(_q[r],_qd[r])+selfRotationCommand*_wRb[r].col(2);
  
  // std::cerr << "omega: " <<_omegad[r].transpose() << std::endl;
}





void SurgicalTask::pillarsAdaptation(int r)
{
  ////////////////////////////////////
  // Compute human desired velocity //
  ////////////////////////////////////

  // Eigen::Vector3f gains;
  // if(_useJoystick)
  // {
  //   gains << 0.03f, 0.03f, 0.15f;
    
  // }
  // else
  // {    
  //   gains << 2.0f/FOOT_INTERFACE_PITCH_RANGE,
  //            2.0f/FOOT_INTERFACE_ROLL_RANGE,
  //            2.0f/FOOT_INTERFACE_Y_RANGE;
  // }

  // Eigen::Vector3f vH, temp2;
  // // temp2 = gains.cwiseProduct(_footPosition[r].segment(0,3));
  // temp2(0) = Utils<float>::deadZone(_footPosition[r](0),-5.0f,5.0f);
  // temp2(1) = Utils<float>::deadZone(_footPosition[r](1),-5.0f,5.0f);
  // temp2(2) = Utils<float>::deadZone(_footPosition[r](2),-0.03f,0.03f);
  // temp2 = temp2.cwiseProduct(gains);
  // vH = temp2(1)*_wRb[r].col(1)+temp2(0)*_wRb[r].col(0);
  // vH(2) = 0.0f;
  // // vH *= 20;

  // std::cerr << "input: " << temp2(1) << " " << temp2(0) << " VH: " << vH.transpose() <<std::endl;

  // /////////////////////
  // // Task adaptation //
  // /////////////////////
  // _vdC.setConstant(0.0f);
  // Eigen::MatrixXf vdP;
  // vdP.resize(_pillarsId.size(),3);
  // for(int k = 0; k < _pillarsId.size(); k++)
  // {
  //   // vdP.row(k) = 2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_pillarsPosition.row(k).transpose()-_x[r]);
  //   vdP.row(k) = 2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_pillarsPosition.row(k).transpose()-_x[r]);
  //   // vdP(k,2) = 0.0f;
  //   if (vdP.row(k).norm()>1e-4)
  //   {
  //     // a = 0.0f;
  //     _vdC+=_beliefsC(k)*vdP.row(k).normalized();
  //   }
  //   // else
  //   // {
  //   //   a = 10*adaptationRate*((vH-_vdC).dot(vdP.row(k).normalized()));
  //   // }

  // }

  // float adaptationRate = 10.0f;

  // Eigen::MatrixXf::Index indexMax;

  // float a, b;
  // // Eigen::Vector3f bou;
  // for(int k = 0; k < _pillarsId.size(); k++)
  // {
  //   if (vdP.row(k).norm()<1e-4)
  //   {
  //     a = 0.0f;
  //   }
  //   else
  //   {
  //     a = adaptationRate*((vH-_vdC).dot(vdP.row(k).normalized()));
  //   }
  //   // a =  10*adaptationRate*((vH-_vdC).dot(vdP.row(k))); 
  //   // b = adaptationRate*(0.5*(_beliefsC(k)-0.5f)*vdP.row(k).squaredNorm());
  //   b = (_beliefsC(k)-0.5f);

  //   _dbeliefsC(k) = a+b;
  //   std::cerr << r <<": Dbeliefs " << k << ": " << a << " " << b <<  " " << a+b << std::endl;
  // }



  Eigen::Vector3f gains;
  if(_useJoystick)
  {
    gains << 0.03f, 0.03f, 0.15f;
    gains.setConstant(1.0f);
    
  }
  else
  {    
    gains << 2.0f*0.03f/FOOT_INTERFACE_PITCH_RANGE,
             2.0f*0.03f/FOOT_INTERFACE_ROLL_RANGE,
             2.0f*0.15f/FOOT_INTERFACE_Y_RANGE;
  }

  Eigen::Vector3f vH, temp2;

  // temp2 = gains.cwiseProduct(_footPosition[r].segment(0,3));
  if(!_useJoystick)
  {
    temp2(0) = Utils<float>::deadZone(_footPosition[r](0),-5.0f,5.0f);
    temp2(1) = Utils<float>::deadZone(_footPosition[r](1),-5.0f,5.0f);
    temp2(2) = Utils<float>::deadZone(_footPosition[r](2),-0.03f,0.03f);
    temp2 = temp2.cwiseProduct(gains);
    vH = temp2(1)*_wRb[r].col(1)+temp2(0)*_wRb[r].col(0);
  }
  else
  {
    temp2 = gains.cwiseProduct(_footPosition[r].segment(0,3));
    vH = temp2(1)*_wRb[r].col(0)+temp2(0)*_wRb[r].col(1);

  }
  
  // vH(2) = 0.0f;
  // vH *= 20;
// 
  std::cerr << "input: " << temp2(1) << " " << temp2(0) << std::endl;

  /////////////////////
  // Task adaptation //
  /////////////////////
  int nbTasks;

  nbTasks = _pillarsId.size();
  if(_useRobot[RIGHT])
  {
    nbTasks+=1;
  }
  _vdC.setConstant(0.0f);
  Eigen::MatrixXf vdP;
  vdP.resize(nbTasks,3);

  for(int k = 0; k < nbTasks; k++)
  {

    if(_useRobot[RIGHT] && k == nbTasks-1)
    {
      vdP.row(k) =  2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_x[RIGHT]-_x[r]); 
    }
    else
    {
      vdP.row(k) = 2.0f*Utils<float>::orthogonalProjector(_wRb[r].col(2))*(_pillarsPosition.row(k).transpose()-_x[r]);
    }
    // vdP(k,2) = 0.0f;
    _vdC+=_beliefsC(k)*vdP.row(k);
    // _vdC+=_beliefsC(k)*vdP.row(k).normalized();
  }


  Eigen::Vector3f d;
  if(_vdC.norm()<1e-4f)
  {
    d.setConstant(0.0f);
  }
  else
  {
    d = _vdC.normalized();
  }

  float adaptationRate = 100.0f;

  Eigen::MatrixXf::Index indexMax;

  float a, b;



  for(int k = 0; k < nbTasks; k++)
  {
    // if(_vdC.norm()<1e-4f)
    // {
    //   a = 0;
    // }
    // else
    // {
    //   Eigen::Vector3f g;
    //   g = (vdP.row(k).transpose()*_vdC.norm()-_vdC.dot(vdP.row(k))*_vdC.normalized())/_vdC.squaredNorm();
    //   a = adaptationRate*(vH).dot(g);
    //   // std::cerr << "g: " << g.transpose() << std::endl;
    // }
      Eigen::Vector3f temp = vdP.row(k);
      // if(temp.norm()<1e-2)
      // {
      //   temp.setConstant(0.0f);
      // }
      // else
      // {
        // temp.normalize();
      // }
      a = adaptationRate*(vH).dot(temp);

    // a = adaptationRate*((vH-_vdC).dot(vdP.row(k)));
    // if(vH.norm()<1e-3)
    // {
    //   vH.setConstant(0.0f);
    // }
    // else
    // {
    //   vH.normalize();
    // }

    // a = adaptationRate*(vH.dot(vdP.row(k)));
    // a = 2*(vH-_vdC).dot(vdP.row(k).normalized());
    b = adaptationRate*((_beliefsC(k)-0.5f)*vdP.row(k).squaredNorm());
    // b = (_beliefsC(k)-0.5f);
    // b = std::min((_beliefsC(k)-0.5f)/vdP.row(k).norm(),1.0f);

    float c = 1*std::exp(-10*(vdP.row(k)).norm());

      // a = adaptationRate*(vH.dot(temp.normalized());
      // b = (_beliefsC(k)-0.5f);
      // c = std::exp(-10*(vdP.row(k)).norm());


    // c = 0.0f;
    _dbeliefsC(k) = a+b+c;
    std::cerr << r << ": Dbeliefs " << k << ": " << a << " " << b <<  " " << c << " " << a+b+c << std::endl;
  }



  // std::cerr << r << ": a: " << _dbeliefsC.transpose() << std::endl;
  float dbmax = _dbeliefsC.array().maxCoeff(&indexMax);

  if(std::fabs(1.0f-_beliefsC(indexMax))< FLT_EPSILON && std::fabs(_beliefsC.sum()-1)<FLT_EPSILON)
  {
    _dbeliefsC.setConstant(0.0f);
  }
  else
  {
    Eigen::VectorXf temp;
    temp.resize(nbTasks-1);
    temp.setConstant(0.0f);
    int m = 0;
    for(int k = 0; k < nbTasks; k++)
    {
      if(k!=indexMax)
      {
        temp(m) = _dbeliefsC(k);
        m++;
      }
    }
    float db2max = temp.array().maxCoeff();
      std::cerr << db2max << std::endl;

    float z = (dbmax+db2max)/2.0f;
    _dbeliefsC.array() -= z;
    std::cerr << r << ": Before Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

    float S = 0.0f;
    for(int k = 0; k < nbTasks; k++)
    {
      // if(fabs(_beliefsC(k))>FLT_EPSILON || _dbeliefsC(k) > 0)
      {
        S += _dbeliefsC(k);
      }
    }
    _dbeliefsC(indexMax)-=S;

  }
  std::cerr << r << ": After Dbeliefs: " << _dbeliefsC.transpose() << std::endl;

  std::cerr << _dbeliefsC.sum() << std::endl;

  _beliefsC+=_dt*_dbeliefsC;
  for(int k = 0; k < nbTasks; k++)
  {
    _beliefsC(k) = Utils<float>::bound(_beliefsC(k),0.0f,1.0f);
  }

  _beliefsC /= _beliefsC.sum();

  std::cerr << r << ": Beliefs: " << _beliefsC.transpose() << std::endl;

  _vdC.setConstant(0.0f);
  for(int k = 0; k < nbTasks; k++)
  {
    _vdC += _beliefsC(k)*vdP.row(k);
  }

  // _vdC = Utils<float>::bound(_fx[r],0.3f);
}




void* SurgicalTask::startIkLoop(void* ptr)
{
    reinterpret_cast<SurgicalTask *>(ptr)->ikLoop(); 
}

void SurgicalTask::ikLoop()
{
  while(_startThread)
  {

    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_robotMode[r]==TROCAR_SPACE)
      {
          Eigen::VectorXf joints(7);

          std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

          bool res = _qpSolverRCM.step(joints, _currentJoints[r], _trocarPosition[r],
          _toolOffsetFromEE[r], _xdTool[r], _currentJoints[r](6), 0.1f, _xRobotBaseOrigin[r]);

          _ikJoints[r] = joints;
          std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

          std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        // std::cerr << "[thread]: " << r <<": Desired offset: " << desiredOffset.transpose() << std::endl; 
        // std::cerr << "[thread]: " << r <<": Current offset: " << currentOffset.transpose() << std::endl; 
        // std::cerr << "[thread]: " << r <<": vd: " << vdTool.transpose() << std::endl; 
        std::cerr << "[thread]: " << r << ": Current joints: " << _currentJoints[r].transpose() << std::endl;
        std::cerr << "[thread]: " << r << ": Desired joints: " << joints.transpose() << std::endl;
        std::cerr << "[thread]: " << r << ": Error joints: " << (joints-_currentJoints[r]).norm() << std::endl;
        std::cerr << "[thread]: " << (int) res <<  " Elasped time in [micro s]: " << duration.count() << std::endl;
        std::cerr << "[thread]: " << (int) res <<  "Tocar position: " << _trocarPosition[r].transpose() << std::endl;
        std::cerr << "[thread]: " <<  "x: " << _x[r].transpose() << std::endl;
        int dt = int(_dt*1e6)-duration.count();
        std::this_thread::sleep_for(std::chrono::microseconds(int(std::max(dt,0))));
      }
      else
      {
        _ikJoints[r] = _currentJoints[r];
      }
    }
  }
  std::cerr << "END thread" << std::endl;
}



void SurgicalTask::humanInputTransformation()
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
    R << -1.0f, 0.0f, 0.0f,
          0.0f, 1.0f, 0.0f,
          0.0f, 0.0f, 1.0f;
    _footPosition[RIGHT] = R*_footPose[RIGHT].segment(0,3);
    _footPosition[LEFT] = R*_footPose[LEFT].segment(0,3);
    // std::cerr << "[SurgicalTask]: " << 0 << ": foot position: " << temp.transpose() << std::endl;


  }
  
}


void SurgicalTask::logData()
{

}


void SurgicalTask::publishData()
{
  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {

      // if(_robotMode[r]==TROCAR_SPACE)
      // {
      //   std_msgs::Float64MultiArray msg;
      //   msg.data.resize(7);
      //   for(int m = 0; m < 7; m++)
      //   {
      //     msg.data[m] = _ikJoints[r](m);
      //   }
      //   _pubDesiredJoints[r].publish(msg);
      // }
      // else
      {


        std_msgs::Float64MultiArray msg;
        msg.data.resize(7);
        if(_robotMode[r]==TROCAR_SPACE)
        {
          for(int m = 0; m < 7; m++)
          {
            msg.data[m] = _ikJoints[r](m);
          }
        }
        else
        {
          for(int m = 0; m < 7; m++)
          {
            msg.data[m] = _currentJoints[r](m);
          }
        }
        _pubDesiredJoints[r].publish(msg);
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
        
      }

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


void SurgicalTask::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _xEE[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _xEE[k]+_toolOffsetFromEE[k]*_wRb[k].col(2);


  // if(k==(int)LEFT)
  // {
  //   _xEE[k] += _leftRobotOrigin;
  //   _x[k] += _leftRobotOrigin;
  // }

  if(_firstRobotBaseFrame[k])
  {
    _xEE[k] += _xRobotBaseOrigin[k];
    _x[k] += _xRobotBaseOrigin[k];
  }

  if(!_firstRobotPose[k])
  {
    if(_firstRobotBaseFrame[k])
    {
      _firstRobotPose[k] = true;
      _xd[k] = _x[k];
      _qd[k] = _q[k];
      _vd[k].setConstant(0.0f);
    }
  }
}


void SurgicalTask::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void SurgicalTask::updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int k)
{
  _footPose[k].setConstant(0.0f);
  _footPose[k](X) = msg->axes[0];
  _footPose[k](Y) = msg->axes[1];
  _footPose[k](PITCH) = msg->axes[4];
  _footPose[k](YAW) = (-msg->axes[5]+1.0f)/2.0f-(-msg->axes[2]+1.0f)/2.0f;
  
  if(!_firstJoystick[k])
  {
    _firstJoystick[k]= true;
  }
}

void SurgicalTask::updateFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg, int k)
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

void SurgicalTask::updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k)
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
      std::cerr << "[SurgicalTask]: Bias " << k << ": " <<_wrenchBias[k].transpose() << std::endl;
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
void SurgicalTask::updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k) 
{
  if(!_firstDampingMatrix[k])
  {
    _firstDampingMatrix[k] = true;
  }

  _D[k] << msg->data[0],msg->data[1],msg->data[2],
           msg->data[3],msg->data[4],msg->data[5],
           msg->data[6],msg->data[7],msg->data[8];
}


void SurgicalTask::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k) 
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