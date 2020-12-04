#include "RobotsTaskGeneration.h"
#include "Utils.h"

RobotsTaskGeneration* RobotsTaskGeneration::me = NULL;

RobotsTaskGeneration::RobotsTaskGeneration(ros::NodeHandle &n, double frequency):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{
  me = this;

  _toolOffsetFromEE = 0.4f;

  for(int k= 0; k < NB_ROBOTS; k++)
  {
    _x[k].setConstant(0.0f);
    _q[k].setConstant(0.0f);
    
    _xd[k].setConstant(0.0f);
    _vd[k].setConstant(0.0f);
    _omegad[k].setConstant(0.0f);
    _qd[k].setConstant(0.0f);
    _qdPrev[k].setConstant(0.0f);
    _xTrocar[k].setConstant(0.0f);

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
  }

  _stop = false;
  _leftRobotOriginReceived = false;
  _leftTrocarFrameReceived = false;
  _rightTrocarFrameReceived = false;
  _leftCameraFrameReceived = false;
  
  _xLeftRobotOrigin.setConstant(0.0f);

  // _strategy = PRIMARY_TASK;
  _strategy = VIRTUAL_RCM;
  _humanInput = FOOT;




  // _msgMarker.header.frame_id = "torso_upper_base_link";
  // _msgMarker.header.stamp = ros::Time();
  // _msgMarker.ns = "marker_test_triangle_list";
  // _msgMarker.id = 0;
  // _msgMarker.type = visualization_msgs::Marker::CUBE;
  // _msgMarker.action = visualization_msgs::Marker::ADD;
  // _msgMarker.pose.position.x = 0.0f;
  // _msgMarker.pose.position.y = 0.15f;
  // _msgMarker.pose.position.z = 0.1f;
  // _msgMarker.pose.orientation.x = 0.0;
  // _msgMarker.pose.orientation.y = 1.0;
  // _msgMarker.pose.orientation.z = 0.0;
  // _msgMarker.pose.orientation.w = 0.0;
  // _msgMarker.scale.x = 0.3;
  // _msgMarker.scale.y = 0.5;
  // _msgMarker.scale.z = 0.2;
  // _msgMarker.color.a = 1.0;
  // _msgMarker.color.r = 1.0;
  // _msgMarker.color.g = 0.0;
  // _msgMarker.color.b = 0.0;

  // for(int k = 0; k < NB_ROBOTS; k++)
  // {
  //   _footPose[k].setConstant(0.0f);
  //   _footWrench[k].setConstant(0.0f);
  //   _footTwist[k].setConstant(0.0f);
  //   _footDesiredWrench[k].setConstant(0.0f);
  //   _footPosition[k].setConstant(0.0f);
  //   _firstFootOutput[k] = false;
  // }
}


bool RobotsTaskGeneration::init() 
{
  // Subscriber definitions
  _subRobotPose[RIGHT] = _nh.subscribe<geometry_msgs::Pose>("/right_lwr/ee_pose", 1, boost::bind(&RobotsTaskGeneration::updateRobotPose,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[RIGHT] = _nh.subscribe<geometry_msgs::Twist>("/right_lwr/joint_controllers/twist", 1, boost::bind(&RobotsTaskGeneration::updateRobotTwist,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subJoystick[RIGHT] = _nh.subscribe<sensor_msgs::Joy>("/right_lwr/joy", 1, boost::bind(&RobotsTaskGeneration::updateJoystick,this,_1,RIGHT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[RIGHT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Right",1, boost::bind(&RobotsTaskGeneration::updateFootOutput,this,_1,RIGHT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  _subRobotPose[LEFT] = _nh.subscribe<geometry_msgs::Pose>("/left_lwr/ee_pose", 1, boost::bind(&RobotsTaskGeneration::updateRobotPose,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subRobotTwist[LEFT] = _nh.subscribe<geometry_msgs::Twist>("/left_lwr/joint_controllers/twist", 1, boost::bind(&RobotsTaskGeneration::updateRobotTwist,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subJoystick[LEFT] = _nh.subscribe<sensor_msgs::Joy>("/left_lwr/joy", 1, boost::bind(&RobotsTaskGeneration::updateJoystick,this,_1,LEFT),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
  _subFootOutput[LEFT] = _nh.subscribe<custom_msgs::FootOutputMsg>("/FI_Output/Left",1, boost::bind(&RobotsTaskGeneration::updateFootOutput,this,_1,LEFT), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist[RIGHT] = _nh.advertise<geometry_msgs::Twist>("/right_lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[RIGHT] = _nh.advertise<geometry_msgs::Quaternion>("/right_lwr/joint_controllers/passive_ds_command_orient", 1);

  _pubDesiredTwist[LEFT] = _nh.advertise<geometry_msgs::Twist>("/left_lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation[LEFT] = _nh.advertise<geometry_msgs::Quaternion>("/left_lwr/joint_controllers/passive_ds_command_orient", 1);

  _pubFootInput[RIGHT] = _nh.advertise<custom_msgs::FootInputMsg>("/FI_Input/Right", 1);
  _pubFootInput[LEFT] = _nh.advertise<custom_msgs::FootInputMsg>("/FI_Input/Left", 1);
  // _pubMarker = _nh.advertise<visualization_msgs::Marker>("RobotsTaskGeneration/marker", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&RobotsTaskGeneration::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  _dynRecServer.getConfigDefault(_config);
   
  // Get default conveyor belt configuration from dynamic reconfigure 
  _config.rightTargetXOffset = 0.0f;
  _config.rightTargetYOffset = 0.0f;
  _config.rightTargetZOffset = 0.0f;
  _config.leftTargetXOffset = 0.0f;
  _config.leftTargetYOffset = 0.0f;
  _config.leftTargetZOffset = 0.0f;
  _dynRecServer.updateConfig(_config);

  signal(SIGINT,RobotsTaskGeneration::stopNode);

  if (_nh.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("[RobotsTaskGeneration]: The RobotsTaskGeneration node is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("[RobotsTaskGeneration]: The RobotsTaskGeneration node has a problem.");
    return false;
  }
}

void RobotsTaskGeneration::run()
{
  while (!_stop) 
  {
    if(allSubscribersOK() && allFramesReceived())
    {
      _mutex.lock();

      receiveFrames();

      if(_alignedWithTrocar[LEFT] && _alignedWithTrocar[RIGHT])
      {
        trackTarget();
      }
      else
      {
        alignWithTrocar();
      }
      
      computeDesiredFootWrench();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }
    else
    {
      receiveFrames();
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
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();
}


void RobotsTaskGeneration::stopNode(int sig)
{
  me->_stop = true;
}


bool RobotsTaskGeneration::allSubscribersOK()
{
  return _firstRobotPose[RIGHT] && _firstRobotPose[LEFT] && _firstRobotTwist[RIGHT] && _firstRobotTwist[LEFT]; 
}

bool RobotsTaskGeneration::allFramesReceived()
{
  return _leftRobotOriginReceived && _leftTrocarFrameReceived && _rightTrocarFrameReceived && _leftCameraFrameReceived; 
}

void RobotsTaskGeneration::receiveFrames()
{
  if(_leftRobotOriginReceived == false)
  {  
    try
    { 
      _lr.waitForTransform("/right_lwr_base_link", "/left_lwr_base_link", ros::Time(0), ros::Duration(3.0));
      _lr.lookupTransform("/right_lwr_base_link", "/left_lwr_base_link",ros::Time(0), _transform);
      _xLeftRobotOrigin << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
      _qLeftRobotOrigin << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRl = Utils<float>::quaternionToRotationMatrix(_qLeftRobotOrigin); 
      _leftRobotOriginReceived = true;
      std::cerr << "[RobotsTaskGeneration]: Left robot origin received: " << _xLeftRobotOrigin.transpose() << std::endl;
    } 
    catch (tf::TransformException ex)
    {
    }
  }  
  if(_leftTrocarFrameReceived == false)
  {  
    try
    { 
      _lr.waitForTransform("/right_lwr_base_link", "/left_lwr_trocar_frame", ros::Time(0), ros::Duration(3.0));
      _lr.lookupTransform("/right_lwr_base_link", "/left_lwr_trocar_frame",ros::Time(0), _transform);
      _xTrocar[LEFT] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
      Eigen::Vector4f qtemp;
      qtemp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRt[LEFT] = Utils<float>::quaternionToRotationMatrix(qtemp); 
      _leftTrocarFrameReceived = true;
      std::cerr << "[RobotsTaskGeneration]: Left trocar frame received" << _xTrocar[LEFT].transpose() << std::endl;
    } 
    catch (tf::TransformException ex)
    {
    }
  }

  if(_rightTrocarFrameReceived == false)
  {  
    try
    { 
      _lr.waitForTransform("/right_lwr_base_link", "/right_lwr_trocar_frame",ros::Time(0), ros::Duration(3.0));
      _lr.lookupTransform("/right_lwr_base_link", "/right_lwr_trocar_frame",ros::Time(0), _transform);
      _xTrocar[RIGHT] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
      Eigen::Vector4f qtemp;
      qtemp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRt[RIGHT] = Utils<float>::quaternionToRotationMatrix(qtemp); 
      _rightTrocarFrameReceived = true;
      std::cerr << "[RobotsTaskGeneration]: Right trocar frame received" << _xTrocar[RIGHT].transpose() << std::endl;
    } 
    catch (tf::TransformException ex)
    {
    }
  }

  if(_leftCameraFrameReceived == false)
  {  
    try
    { 
      _lr.waitForTransform("/right_lwr_base_link", "/left_lwr_camera_link",ros::Time(0), ros::Duration(3.0));
      _lr.lookupTransform("/right_lwr_base_link", "/left_lwr_camera_link",ros::Time(0), _transform);
      _qLeftCameraOrigin << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRc = Utils<float>::quaternionToRotationMatrix(_qLeftCameraOrigin); 
      _rRcp = _rRc;
      _leftCameraFrameReceived = true;
      std::cerr << "[RobotsTaskGeneration]: Left camera link received" << std::endl;
    } 
    catch (tf::TransformException ex)
    {
    }
  }

  if(_rightTrocarFrameReceived && _leftTrocarFrameReceived && _leftCameraFrameReceived)
  {
    try
    { 
      _lr.lookupTransform("/right_lwr_base_link", "/left_lwr_trocar_frame",ros::Time(0), _transform);
      _xTrocar[LEFT] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
      Eigen::Vector4f qtemp;
      qtemp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRt[LEFT] = Utils<float>::quaternionToRotationMatrix(qtemp); 

    } 
    catch (tf::TransformException ex)
    {
    }

    try
    { 
      _lr.lookupTransform("/right_lwr_base_link", "/right_lwr_trocar_frame",ros::Time(0), _transform);
      _xTrocar[RIGHT] << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
      Eigen::Vector4f qtemp;
      qtemp << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRt[RIGHT] = Utils<float>::quaternionToRotationMatrix(qtemp); 
    } 
    catch (tf::TransformException ex)
    {
    }

    try
    { 
      _lr.lookupTransform("/right_lwr_base_link", "/left_lwr_camera_link",ros::Time(0), _transform);
      _qLeftCameraOrigin << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
      _rRcp = _rRc;
      _rRc = Utils<float>::quaternionToRotationMatrix(_qLeftCameraOrigin); 
    } 
    catch (tf::TransformException ex)
    {
    }
  }


}

void RobotsTaskGeneration::alignWithTrocar()
{

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    _vd[k] = 2.0f*(_xTrocar[k]-_x[k]);

    // Compute rotation error between current orientation and plane orientation using Rodrigues' law
    Eigen::Vector3f w;
    Eigen::Vector3f zBd;
    zBd = (_xTrocar[k]-(_x[k]-_toolOffsetFromEE*_wRb[k].col(2))).normalized();
    if((int) k == LEFT)
    {
      std::cerr << _x[k].transpose() << std::endl;
      std::cerr << _wRb[k].col(2).transpose() << std::endl;
      std::cerr << zBd.transpose() << std::endl;
    }
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

    // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
    _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(5.0f*_vd[k].norm()));

    // Compute needed angular velocity to perform the desired quaternion
    Eigen::Vector4f qcurI, wq;
    qcurI(0) = _q[k](0);
    qcurI.segment(1,3) = -_q[k].segment(1,3);
    wq = 2.0f*Utils<float>::quaternionProduct(qcurI,_qd[k]-_q[k]);
    Eigen::Vector3f omegaTemp = _wRb[k]*wq.segment(1,3);
    _omegad[k] = omegaTemp; 

    if(angle < 0.05f && (_xTrocar[k]-_x[k]).norm()<0.04)
    {
      _alignedWithTrocar[k] = true;
    }

    std::cerr << "[RobotsTaskGeneration]: " << k << " position error: " << _vd[k].norm() << " angular error: " << angle << std::endl;
    _xd[k] = _x[k];
  }
}


void RobotsTaskGeneration::computeAttractors()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Vector3f rToolTrocar;
    rToolTrocar = _x[k]-_xTrocar[k];

    float arcLength;
    arcLength = 60.0f*M_PI/180.0f*rToolTrocar.norm();

    if(k==(int)LEFT)
    {
      Eigen::Vector3f xDtemp;
      xDtemp = arcLength*(_rRc.col(1)*_joyAxes[k](0)+_rRc.col(2)*_joyAxes[k](1));
      xDtemp += _rRc.col(0)*(0.25f*std::max(0.0f,_joyAxes[k](4)));
      Eigen::Vector3f a,b;
      a = xDtemp;
      a(2) -=0.05f;
      // a = rToolTrocar;
      b = -_rRt[k].col(2);
      if(a.dot(b)<0)
      {
        b = -b;
      }
      float angle = std::acos(a.dot(b)/(a.norm()*b.norm())); 
      std::cerr << "a1: " << angle*180.0f/M_PI << " " << rToolTrocar.norm() <<std::endl;

      if(std::fabs(angle)<60.0f*M_PI/180.0f)
      {
        _xdOffset[k] = xDtemp;
      }
      else
      {
        Eigen::Vector3f w = (-_rRt[k].col(2)).cross(_rRc.col(0));
        float c = (-_rRt[k].col(2)).transpose()*_rRc.col(0);  
        float s = w.norm();
        w /= s;
        Eigen::Vector3f v;
        if(a.norm()>0.3f)
        {
          v = -_rRt[k].col(2)*0.3f;
        }
        else
        {
          v = -_rRt[k].col(2)*a.norm();
        }
        v = -_rRt[k].col(2)*rToolTrocar.norm();
        angle = 60.0f*M_PI/180.0f;
        c= cos(angle);
        s=sin(angle);
        _xdOffset[k] = c*v+s*w.cross(v)+(1-c)*w.dot(v)*w;
        std::cerr << std::acos(_xdOffset[k].dot(-_rRt[k].col(2))/_xdOffset[k].norm())*180.0f/M_PI << std::endl;
        _xdOffset[k](2) += 0.05f;
    
    // Eigen::Matrix3f K;
    // K << Utils<float>::getSkewSymmetricMatrix(w);
        // _xdOffset[k] =  _rRc.col(0)*(0.25f*std::max(0.0f,_joyAxes[k](4)));
        // a = xDtemp;
        // a(2) -=0.05f;
        // if(a.dot(b)<0)
        // {
        //   b = -b;
        // }
        // angle = std::acos(a.dot(b)/(a.norm()*b.norm())); 
        // if(std::fabs(angle)<50.0f*M_PI/180.0f)
        // {
        //   _xdOffset[k] = xDtemp;
        // }
        // else
        // {
        //   _xdOffset[k] = rToolTrocar;
        //   _xdOffset[k](2)+=0.05;
        // }  
      }


      std::cerr << "a2: " << angle*180.0f/M_PI << std::endl;
      // std::cerr << _xdOffset[k].transpose() << " " << angle*180.0f/M_PI << std::endl;
    } 
    else
    {
      Eigen::Vector3f xDtemp;
      xDtemp = arcLength*(_wRb[k].col(1)*_joyAxes[k](0)-_wRb[k].col(0)*_joyAxes[k](1));
      xDtemp += _wRb[k].col(2)*(0.15f*std::max(0.0f,_joyAxes[k](4)));
      Eigen::Vector3f a,b;
      a = xDtemp;
      a(2) -=0.05f;
      b = -_rRt[k].col(2);
      if(a.dot(b)<0)
      {
        b = -b;
      }
      if(std::fabs(std::acos(a.dot(b)/(a.norm()*b.norm())))<50.0f*M_PI/180.0f)
      {
        _xdOffset[k] = xDtemp;
      }
    }
  }

  for(int k = 0; k < NB_ROBOTS; k++)
  {
    if(k==(int)LEFT)
    {
      _vdOffset[k] = 0.2f*(_rRc.col(1)*_joyAxes[k](0)+_rRc.col(2)*_joyAxes[k](1)+_rRc.col(0)*_joyAxes[k](4));
    }
    else
    {
      // _vdOffset[k] = 0.2f*(_wRb[k].col(1)*_joyAxes[k](0)-_wRb[k].col(0)*_joyAxes[k](1)+_wRb[k].col(2)*_joyAxes[k](4));
      _vdOffset[k] = 0.2f*(_rRc.col(1)*_joyAxes[k](0)+_rRc.col(2)*_joyAxes[k](1)+_rRc.col(0)*_joyAxes[k](4));

    }
  }
}

void RobotsTaskGeneration::footPositionMapping()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    Eigen::Vector3f rToolTrocar;
    rToolTrocar = _x[k]-_xTrocar[k];

    float arcLength;
    arcLength = 50.0f*M_PI/180.0f*rToolTrocar.norm();

    if(k==(int)LEFT)
    {
      Eigen::Vector3f xDtemp;
      xDtemp = arcLength*(-_rRc.col(1)*2.0f*_footPose[k](0)/FOOT_INTERFACE_X_RANGE_LEFT+_rRc.col(2)*2.0f*_footPose[k](1)/FOOT_INTERFACE_Y_RANGE_LEFT);
      xDtemp += _rRc.col(0)*(0.25f*2.0f*std::max(0.0f,-2.0f*_footPose[k](3))/FOOT_INTERFACE_PHI_RANGE_LEFT);
      // _xdOffset[k] = xDtemp;
      Eigen::Vector3f a,b;
      a = xDtemp;
      a(2) -=0.05f;
      b = -_rRt[k].col(2);
      if(a.dot(b)<0)
      {
        b = -b;
      }
      float angle = std::acos(a.dot(b)/(a.norm()*b.norm()));
      if(std::fabs(angle)<50.0f*M_PI/180.0f)
      {
        _xdOffset[k] = xDtemp;
      }


    } 
    else
    {
      Eigen::Vector3f xDtemp;
      xDtemp = arcLength*(_wRb[k].col(0)*2.0f*_footPose[k](0)/FOOT_INTERFACE_X_RANGE_RIGHT+_wRb[k].col(1)*2.0f*_footPose[k](1)/FOOT_INTERFACE_Y_RANGE_RIGHT);
      xDtemp += _wRb[k].col(2)*(0.15f*2.0f*std::max(0.0f,-2.0f*_footPose[k](3))/FOOT_INTERFACE_PHI_RANGE_LEFT);
      Eigen::Vector3f a,b;
      a = xDtemp;
      a(2) -=0.05f;
      b = -_rRt[k].col(2);
      if(a.dot(b)<0)
      {
        b = -b;
      }
      if(std::fabs(std::acos(a.dot(b)/(a.norm()*b.norm())))<50.0f*M_PI/180.0f)
      {
        _xdOffset[k] = xDtemp;
      }
    }
  }
    for(int k = 0; k < NB_ROBOTS; k++)
  {

    Eigen::Vector3f x;
    x << -2.0f*_footPosition[k](2)/FOOT_INTERFACE_PHI_RANGE_LEFT,-2.0f*_footPosition[k](0)/FOOT_INTERFACE_X_RANGE_LEFT,2.0f*_footPosition[k](1)/FOOT_INTERFACE_Y_RANGE_LEFT;
    // if(k==(int)LEFT)
    // {
      _vdOffset[k] = 0.2f*_rRc*x;
    // }
    // else
    // {
    //   // _vdOffset[k] = 0.2f*(_wRb[k].col(1)*_joyAxes[k](0)-_wRb[k].col(0)*_joyAxes[k](1)+_wRb[k].col(2)*_joyAxes[k](4));
    //   _vdOffset[k] = 0.2f*(-_rRc.col(1)*2.0f*_footPose[k](0)/FOOT_INTERFACE_X_RANGE_LEFT+_rRc.col(2)*2.0f*_footPose[k](1)/FOOT_INTERFACE_Y_RANGE_LEFT
    //   -_rRc.col(0)*2.0f*_footPose[k](3)/FOOT_INTERFACE_PHI_RANGE_LEFT);

    // }
    std::cerr << k << " : " << x.transpose() << std::endl;
    // _vdOffset[k].setConstant(0.0f);
  }
}

void RobotsTaskGeneration::trackTarget()
{

  // computeAttractors();
  footPositionMapping();
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // _xd[k] = _xTrocar[k]+_xdOffset[k];
    // _xd[k](2) -= 0.05f;

    Eigen::Vector3f vdTool, rToolEE,rTrocarEE, xEE;
    // vdTool = 3.0f*(_xd[k]-_x[k]);

    // if(fabs(_joyAxes[k](0))<FLT_EPSILON && fabs(_joyAxes[k](1))<FLT_EPSILON && fabs(_joyAxes[k](3))<FLT_EPSILON) 
    // {
    //   vdTool = (_xd[k]-_x[k]);
    // }
    // else
    // {
      vdTool = _vdOffset[k];
    //   _xd[k] = _x[k];
    // }
    // if(vdTool.norm()>0.3f)
    // {
    //   vdTool *= 0.3f/vdTool.norm();
    // }

    xEE = _x[k]-_toolOffsetFromEE*_wRb[k].col(2);  
    rToolEE = _wRb[k].col(2)*_toolOffsetFromEE;
    rTrocarEE = _xTrocar[k]-xEE;

    Eigen::Vector3f xRCM, rRCMEE, vdRCM;
    xRCM = xEE+rTrocarEE.dot(_wRb[k].col(2))*_wRb[k].col(2);
    rRCMEE = xRCM-xEE;
    if(rRCMEE.norm()>_toolOffsetFromEE && vdTool.dot(_wRb[k].col(2))<0.0f)
    {
      vdTool.setConstant(0.0f);
    }

    if(_strategy==VIRTUAL_RCM)
    {
      _omegad[k] = (rToolEE-rRCMEE).cross((Eigen::Matrix3f::Identity()-_wRb[k].col(2)*_wRb[k].col(2).transpose())*vdTool)/(rToolEE-rRCMEE).squaredNorm();
      if (_omegad[k].norm() > 1.5f) 
      {
        _omegad[k] = 1.5f*_omegad[k]/_omegad[k].norm();
      }
      vdRCM = 20.0f*(_xTrocar[k]-xRCM);
      _vd[k] = vdTool-_omegad[k].cross(rToolEE)+vdRCM;
      _omegad[k] += _selfRotationCommand[k]*_wRb[k].col(2); 
    }
    else
    {
      _omegad[k] = (rToolEE-rTrocarEE).cross((Eigen::Matrix3f::Identity()-_wRb[k].col(2)*_wRb[k].col(2).transpose())*vdTool)/(rToolEE-rTrocarEE).squaredNorm();
      if (_omegad[k].norm() > 1.5f) 
      {
        _omegad[k] = 1.5f*_omegad[k]/_omegad[k].norm();
      }
      _vd[k] = vdTool-_omegad[k].cross(rToolEE);

      Eigen::Vector3f dir;
      dir = _wRb[k].transpose()*rTrocarEE;
      float distance = dir.norm();
      dir.normalize();
      
      Eigen::Matrix3f P = Eigen::Matrix3f::Identity()-dir*dir.transpose();
      
      Eigen::Matrix3f S;
      S = Utils<float>::getSkewSymmetricMatrix(dir);
    
      Eigen::Matrix<float,3,6> L;
      L.block(0,0,3,3) = -P/distance;
      L.block(0,3,3,3) = S;

      Eigen::Matrix<float,6,3> W;
      W.setConstant(0.0f);
      Eigen::JacobiSVD<Eigen::MatrixXf> svd;
      Eigen::MatrixXf singularValues;
      svd = Eigen::JacobiSVD<Eigen::MatrixXf>(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
      singularValues = svd.singularValues();

      float tolerance = 1e-6f*std::max(L.rows(),L.cols())*singularValues.array().abs().maxCoeff();
    
      for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
      {
        if(singularValues(m,0)>tolerance)
        {
          W(m,m) = 1.0f/singularValues(m,0);
        }
        else
        {
          W(m,m) = 0.0f;
        }
      }
      
      Eigen::Matrix<float,6,3> Linv;
      Linv = svd.matrixV()*W*svd.matrixU().adjoint();
    
      Eigen::Vector3f sd;
      sd << 0.0f,0.0f,1.0f;

      Eigen::Matrix<float,6,1> twistB,gains;
      gains << 10.0f*Eigen::Vector3f::Ones(), 10.0f*Eigen::Vector3f::Ones();  
      twistB = gains.cwiseProduct(Linv*(sd-dir));
      _vd[k] += _wRb[k]*twistB.segment(0,3);
      _omegad[k] += _wRb[k]*twistB.segment(3,3);
      _omegad[k] += _selfRotationCommand[k]*_wRb[k].col(2); 

      // // // Compute nullspace basis
      // Eigen::Vector3f ex, ey;
      // ex << 1.0f, 0.0f, 0.0f;
      // ey << 0.0f, 1.0f, 0.0f;

      // ex = -_wRb[k].transpose()*ex;
      // ey = -_wRb[k].transpose()*ey;

      // Eigen::Matrix<float, 6,1> n1,n2,n3,n4;
      // n1.setConstant(0.0f);
      // n2.setConstant(0.0f);
      // n3.setConstant(0.0f);
      // n4.setConstant(0.0f);

      // n1.segment(0,3) = s;
      // n2.segment(3,3) = s;
      // n3.segment(0,3) = -S*ey;
      // n3.segment(3,3) = -P*ey/distance;
      // n4.segment(0,3) = S*ex;
      // n4.segment(3,3) = P*ex/distance;

      // Eigen::MatrixXf N;
      // N.resize(6,4);
      // N.col(0) = n1;
      // N.col(1) = n2;
      // N.col(2) = n3;
      // N.col(3) = n4;

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

      // std::cerr << (_wRb*(N*(Ninv*X)).segment(3,3)).transpose() << std::endl;
      // std::cerr << (_vd).transpose() << std::endl;
      // std::cerr << "primary task: " << (_wRb*twistB.segment(0,3)).transpose() << std::endl;
      // std::cerr << "nullspace task: " << _vd.transpose() << std::endl;
      // _vd = _wRb*twistB.segment(0,3)+_vd;
      // _omegad = _wRb*(twistB.segment(3,3))+_omegad;
      // twistB += N*(Ninv*X);
      // _vd[k] = _wRb[k]*twistB.segment(0,3);
      // _omegad[k] = _wRb[k]*(twistB.segment(3,3));
      // _vd[k] += _wRb[k]*twistB.segment(0,3);
      // _omegad[k] += _wRb[k]*twistB.segment(3,3);
    }

    // Eigen::Vector3f w;
    Eigen::Vector3f zBd;
    zBd = (_xTrocar[k]-(_x[k]-_toolOffsetFromEE*_wRb[k].col(2))).normalized();
    // w = (_wRb[k].col(2)).cross(zBd);
    // float c = (_wRb[k].col(2)).transpose()*zBd;  
    // float s = w.norm();
    // w /= s;
    
    // Eigen::Matrix3f K;
    // K << Utils<float>::getSkewSymmetricMatrix(w);

    Eigen::Matrix3f Re;
    // if(fabs(s)< FLT_EPSILON)
    // {
    //   Re = Eigen::Matrix3f::Identity();
    // }
    // else
    // {
    //   Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
    // }
    Eigen::Matrix3f Rd;
    Rd.col(2) = zBd;
    Rd.col(1) = ((Eigen::Matrix3f::Identity()-zBd*zBd.transpose())*_wRb[k].col(1)).normalized();
    Rd.col(0) = Rd.col(1).cross(Rd.col(2));
    Rd.col(0).normalize();
    // Re = Rd*_wRb[k].transpose();

    // // Convert rotation error into axis angle representation
    // Eigen::Vector3f omega;
    // float angle;
    // Eigen::Vector4f qtemp = Utils<float>::rotationMatrixToQuaternion(Re);
    // Utils<float>::quaternionToAxisAngle(qtemp,omega,angle);

    // // Compute final quaternion on plane
    // Eigen::Vector4f qf = Utils<float>::quaternionProduct(qtemp,_q[k]);

    // // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the surface
    // // _qd[k] = Utils<float>::slerpQuaternion(_q[k],qf,1.0f-std::tanh(5.0f*_vd[k].norm()));
    // _qd[k] = qf;
    _qd[k] = Utils<float>::rotationMatrixToQuaternion(Rd);

    if(_qd[k].dot(_q[k])<0.0f)
    {
      _qd[k] *=-1.0f;
    }
    // Eigen::Vector4f wq;
    // wq << 0.0f, _selfRotationCommand[k]*(_wRb[k].transpose()).col(2);
    // _qd[k] += Utils<float>::quaternionProduct(_qd[k],wq);
  // qcurI(0) = _q(0);
  // qcurI.segment(1,3) = -_q.segment(1,3);
  // wq = 5.0f*Utils<float>::quaternionProduct(qcurI,_qd-_q);
  // Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  // _omegad = omegaTemp; 

    // if (_vd[k].norm() > 0.3f) 
    // {
    //   _vd[k] = 0.3f*_vd[k]/_vd[k].norm();
    // }
    
    // std::cerr << "[RobotsTaskGeneration]: " << k << " xd: " << _xd[k].transpose() << std::endl;
    // std::cerr << "[RobotsTaskGeneration]: " << k << " vd: " << _vd[k].transpose() << std::endl;
    // std::cerr << "[RobotsTaskGeneration]: " << k << " x: " << _x[k].transpose() << std::endl;
    // std::cerr << _xdOffset[k].transpose() << std::endl;
    // if(k==(int)LEFT)
    // {
    //   std::cerr << _xdOffset[LEFT] << std::endl;
    //   std::cerr << _rRc << std::endl;
    // }
    _qdPrev[k] = _qd[k];
  }  
}







void RobotsTaskGeneration::computeDesiredOrientation()
{

}


// void RobotsTaskGeneration::footDataTransformation()
// {
//   _footPosition[RIGHT](0) = _footPose[RIGHT](1);
//   _footPosition[RIGHT](1) = -_footPose[RIGHT](0);
//   _footPosition[RIGHT](2) = _footPose[RIGHT](3);
//   _footPosition[LEFT](0) = _footPose[LEFT](1);
//   _footPosition[LEFT](1) = -_footPose[LEFT](0);
//   _footPosition[LEFT](2) = _footPose[LEFT](3);

//   // std::cerr << "Before transformation: " <<_footPose.segment(0,4).transpose() << std::endl;
//   // std::cerr << "After transformation: " << _footPosition.transpose() << std::endl;
// }


// void RobotsTaskGeneration::positionPositionMapping()
// {

//   Eigen::Vector3f gains[NB_ROBOTS];
//   // _xyPositionMapping = (_toolOffsetFromEE-_trocarOffsetFromEE)*sqrt(2)/2;
//   // _zPositionMapping = _xyPositionMapping;
//   _xyPositionMapping = 0.14f;
//   _zPositionMapping = 0.10f;
//   gains[RIGHT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_RIGHT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_RIGHT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_RIGHT;
//   gains[LEFT] << 2*_xyPositionMapping/FOOT_INTERFACE_Y_RANGE_LEFT, 2*_xyPositionMapping/FOOT_INTERFACE_X_RANGE_LEFT, 2*_zPositionMapping/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
//   std::cerr << gains[RIGHT].transpose() << std::endl;
//   std::cerr << _footPosition[RIGHT].transpose() << std::endl;
//   for(int k = 0; k < NB_ROBOTS; k++)
//   {
//     if(_footPosition[k](2)>FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f)
//     {
//       _footPosition[k](2) = FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f; 
//     }
//     else if(_footPosition[k](2)<-FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f)
//     {
//       _footPosition[k](2) = -FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f; 
//     }

//     _xh[k] = _xd+gains[k].cwiseProduct(_footPosition[k]);
//     // _xh[k](2)+=(_zPositionMapping/2-0.03f);
//     std::cerr << "Master position " << k << " : " <<_xh[k].transpose() << std::endl;
//   }
// }

// void RobotsTaskGeneration::positionVelocityMapping()
// {

//   _velocityLimit = 0.3f;
//   Eigen::Vector3f gains[NB_ROBOTS];
//   gains[RIGHT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_RIGHT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_RIGHT;
//   gains[LEFT] << 2.0f*_velocityLimit/FOOT_INTERFACE_Y_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_X_RANGE_LEFT, 2.0f*_velocityLimit/FOOT_INTERFACE_PHI_RANGE_LEFT;
  
//   for(int k = 0; k < NB_ROBOTS; k++)
//   {
//     _vh[k] = gains[k].cwiseProduct(_footPosition[k]);
//     if(_vh[k].norm()>_velocityLimit)
//     {
//       _vh[k] *= _velocityLimit/_vh[k].norm();
//     }    
//     // std::cerr << "Master velocity " << k << " : " <<_vh[k].transpose() << std::endl;
//   }
// }

void RobotsTaskGeneration::computeDesiredFootWrench()
{
  // Eigen::Vector3f temp;
  // temp = _wRb[RIGHT]*_filteredWrench[RIGHT].segment(0,3);

  // // temp.setConstant(0.0f);
  // _desiredFootWrench[RIGHT](1) = temp(0);
  // _desiredFootWrench[RIGHT](0) = -temp(1);
  // _desiredFootWrench[RIGHT](3) = temp(2)*0.205/5;
  for(int k = 0; k < NB_ROBOTS; k++)
  { 
    _desiredFootWrench[k](0) = -_kxy*_footPose[k](0)-_dxy*_footTwist[k](0);
    _desiredFootWrench[k](1) = -_kxy*_footPose[k](1)-_dxy*_footTwist[k](1);
    _desiredFootWrench[k](3) = -_kphi*_footPose[k](3)-_dphi*_footTwist[k](3);
  }
  std::cerr << _kxy << " " << _dxy << " " << _kphi << " " << _dphi << std::endl;  
  std::cerr << _desiredFootWrench[LEFT].transpose() << std::endl;

  for(int k = 0 ; k < 3; k++)
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
    if(_desiredFootWrench[RIGHT](k+3)>0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+3) = 0.187f*40/9.15;
    }
    else if(_desiredFootWrench[RIGHT](k+3)<-0.187f*40/9.15)
    {
      _desiredFootWrench[RIGHT](k+3) = -0.187f*40/9.15;
    }

    if(_desiredFootWrench[LEFT](k+3)>0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+3) = 0.212f*40/9.15;
    }
    else if(_desiredFootWrench[LEFT](k+3)<-0.212f*40/9.15)
    {
      _desiredFootWrench[LEFT](k+3) = -0.212f*40/9.15;
    }

  }
}

void RobotsTaskGeneration::logData()
{
  // _outputFile << ros::Time::now() << " "
  //             << _x.transpose() << " "
  //             << _v.transpose() << " "
  //             << _fxc.transpose() << " "
  //             << _fxr.transpose() << " "
  //             << _fxp.transpose() << " "
  //             << _vd.transpose() << " "
  //             << _n.transpose() << " "
  //             << _wRb.col(2).transpose() << " "
  //             << (_markersPosition.col(P1)-_markersPosition.col(ROBOT_BASIS)).transpose() << " "
  //             << _normalDistance << " "
  //             << _normalForce << " "
  //             << _Fd << " "
  //             << _lambdaf << " "
  //             << _sequenceID << " "
  //             << _s << " " 
  //             << _pd << " " 
  //             << _pc << " " 
  //             << _pr << " " 
  //             << _pf << " " 
  //             << _alpha << " "
  //             << _betac << " "
  //             << _betacp << " "
  //             << _betar << " "
  //             << _betarp << " "
  //             << _gamma << " "
  //             << _gammap << " "
  //             << _dW << " " << std::endl;
}


void RobotsTaskGeneration::publishData()
{
  for(int k = 0; k < NB_ROBOTS; k++)
  {
    // Publish desired twist (passive ds controller)
    if(k == (int) LEFT)
    {
      _vd[k] = _rRl.transpose()*_vd[k];
      _omegad[k] = _rRl.transpose()*_omegad[k];
      Eigen::Matrix3f Rd;
      Rd = Utils<float>::quaternionToRotationMatrix(_qd[k]);
      _qd[k] = Utils<float>::rotationMatrixToQuaternion(_rRl.transpose()*Rd);
    }
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

    _msgFootInput.FxDes = _desiredFootWrench[k](0);
    _msgFootInput.FyDes = _desiredFootWrench[k](1);
    _msgFootInput.TphiDes = _desiredFootWrench[k](3);
    _msgFootInput.TthetaDes = _desiredFootWrench[k](4);
    _msgFootInput.TpsiDes = _desiredFootWrench[k](5);
    _msgFootInput.stateDes = 2;
    _pubFootInput[k].publish(_msgFootInput);
  }

  // _pubMarker.publish(_msgMarker);
}

void RobotsTaskGeneration::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k)
{
  //////////////////////////////
  // TRANSFORM FOR LEFT ROBOT //
  //////////////////////////////

  Eigen::Vector3f temp = _x[k];

  // Update end effecotr pose (position+orientation)
  _x[k] << msg->position.x, msg->position.y, msg->position.z;
  _q[k] << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
  _wRb[k] = Utils<float>::quaternionToRotationMatrix(_q[k]);
  _x[k] = _x[k]+_toolOffsetFromEE*_wRb[k].col(2);

  if(k==(int)LEFT)
  {
    _x[k] = _rRl*_x[k]+_xLeftRobotOrigin;
    _wRb[k] = _rRl*_wRb[k];
    _q[k] = Utils<float>::rotationMatrixToQuaternion(_wRb[k]);
  }

  if(!_firstRobotPose[k])
  {
    _firstRobotPose[k] = true;
    _xd[k] = _x[k];
    _qd[k] = _q[k];
    _qdPrev[k] = _qd[k];
    _vd[k].setConstant(0.0f);
  }
}


void RobotsTaskGeneration::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k)
{
  _v[k] << msg->linear.x, msg->linear.y, msg->linear.z;
  _w[k] << msg->angular.x, msg->angular.y, msg->angular.z;

  if(!_firstRobotTwist[k])
  {
    _firstRobotTwist[k] = true;
  }
}
 

void RobotsTaskGeneration::updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int k)
{

  if(_alignedWithTrocar[LEFT] && _alignedWithTrocar[RIGHT])
  {
    _joyAxes[k] << msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], 
                   msg->axes[4], msg->axes[5], msg->axes[6], msg->axes[7]; 


     if(_humanInput==JOYSTICK)
     {
      _selfRotationCommand[k] = _joyAxes[k](2)-_joyAxes[k](5);
      
     }
     else 
     {
      _selfRotationCommand[LEFT] = -_joyAxes[k](0);
      _selfRotationCommand[RIGHT] = -_joyAxes[k](3);
     }
    _joystickSequenceID[k] = msg->header.seq;
  }
}

void RobotsTaskGeneration::updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k)
{

  // _footPose[k] << msg->x, msg->y,0.0f, msg->phi, msg->theta, msg->psi;
  _footPose[k] << msg->x, msg->y,0.0f, msg->phi, msg->theta, msg->psi;
  _footWrench[k] << msg->Fx_m, msg->Fy_m,0.0f, msg->Tphi_m, msg->Ttheta_m, msg->Tpsi_m;
  _footTwist[k] << msg->vx, msg->vy, 0.0f, msg->wphi, msg->wtheta, msg->wpsi;
  _footState[k] = msg->state;

  _footPosition[k](0) = Utils<float>::deadZone(_footPose[k](0),-0.05,0.05);
  _footPosition[k](1) = Utils<float>::deadZone(_footPose[k](1),-0.05,0.05);
  _footPosition[k](2) = Utils<float>::deadZone(_footPose[k](3),-5.0f,5.0f);

  if(_footPose[k](3)>FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f)
  {
    _footPose[k](3) = FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f; 
  }
  else if(_footPose[k](3)<-FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f)
  {
    _footPose[k](3) = -FOOT_INTERFACE_PHI_RANGE_RIGHT/2.0f; 
  }

  // if(!_firstFootOutput[k])
  // {
  //   _firstFootOutput[k] = true;
  //   _footPose0[k] = _footPose[k];
  // }
  // _selfRotationCommand[k] = -2.0f*Utils<float>::deadZone(msg->psi-_footPose0[k](5),-5.0f,5.0f)/FOOT_INTERFACE_PSI_RANGE;
  // if(_selfRotationCommand[k]>1.0f)
  // {
  //   _selfRotationCommand[k]=1.0f;
  // }
  // else if(_selfRotationCommand[k]<-1.0f)
  // {
  //   _selfRotationCommand[k]=-1.0f;
  // }
}


void RobotsTaskGeneration::dynamicReconfigureCallback(surgical_simulator::robotsTaskGeneration_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _velocityLimit = config.velocityLimit;
  _xdOffset[RIGHT] << config.rightTargetXOffset, config.rightTargetYOffset, config.rightTargetZOffset;
  _xdOffset[LEFT] << config.leftTargetXOffset, config.leftTargetYOffset, config.leftTargetZOffset;
  _kxy = config.kxy;
  _dxy = config.dxy;
  _kphi = config.kphi;
  _dphi = config.dphi;
}


void RobotsTaskGeneration::pseudo_inverse(Eigen::Matrix3f &M_, Eigen::Matrix3f &M_pinv_)
{ 
  double lambda_ = 0.2f;

  Eigen::JacobiSVD<Eigen::Matrix3f> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::Matrix3f>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::Matrix3f S_ = M_; // copying the dimensions of M_, its content is not needed.
  S_.setZero();
  // std::cerr << sing_vals_.transpose() << std::endl;
    for (int i = 0; i < sing_vals_.size(); i++)
    {
      if(sing_vals_(i)>1e-6f)
      {
        S_(i,i) = 1.0f/sing_vals_(i);
        // S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);
      }
    }

    M_pinv_ = Eigen::Matrix3f(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}
