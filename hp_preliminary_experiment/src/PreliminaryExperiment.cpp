#include "PreliminaryExperiment.h"

PreliminaryExperiment* PreliminaryExperiment::me = NULL;

PreliminaryExperiment::PreliminaryExperiment(ros::NodeHandle &n, double frequency):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
{
  me = this;
  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.046f;

  _x.setConstant(0.0f);
  _q.setConstant(0.0f);

  _vd.setConstant(0.0f);
  _xd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd.setConstant(0.0f);

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _firstRobotPoseReceived = false;
  _allMarkersPositionReceived = false;
  _stop = false;
  _calibrationOK = false;
  _doCalibration = false;

  _markersCount = 0;
  _calibrationCount = 0;

  _outputFile.open("src/hasler_project/hp_preliminary_experiment/data.txt");
}


bool PreliminaryExperiment::init() 
{
  // Subscriber definitions
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &PreliminaryExperiment::updateRobotPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/joint_controllers/twist", 1, &PreliminaryExperiment::updateRobotTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  // _subOptitrackHip = _n.subscribe("/optitrack/hip/pose", 1, &PreliminaryExperiment::updateHipPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackThigh = _n.subscribe("/optitrack/thigh/pose", 1, &PreliminaryExperiment::updateThighPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackKnee = _n.subscribe("/optitrack/knee/pose", 1, &PreliminaryExperiment::updateKneePose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackTibia = _n.subscribe("/optitrack/tibia/pose", 1, &PreliminaryExperiment::updateTibiaPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackAnkle = _n.subscribe("/optitrack/ankle/pose", 1, &PreliminaryExperiment::updateAnklePose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackHeel = _n.subscribe("/optitrack/heel/pose", 1, &PreliminaryExperiment::updateHeelPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackToe = _n.subscribe("/optitrack/toe/pose", 1, &PreliminaryExperiment::updateToePose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("PreliminaryExperiment/taskAttractor", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("PreliminaryExperiment/markers", 1);


  _dynRecCallback = boost::bind(&PreliminaryExperiment::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);


  signal(SIGINT,PreliminaryExperiment::stopNode);

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The preliminary experiment is ready.");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void PreliminaryExperiment::run()
{
  while (!_stop) 
  {
    if(_allMarkersPositionReceived)
    {

      _mutex.lock();

      // Compute angles from marker positions after calibration
      if(!_calibrationOK && _doCalibration)
      {
        calibration();
      }
      else
      {
        computeAngles();
      }

      // Publish data to topics
      publishData();

      // Log data
      logData();

      _mutex.unlock();
    }

    if(_markersCount == NB_MARKERS)
    {
      _allMarkersPositionReceived = true;
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vd.setConstant(0.0f);
  _omegad.setConstant(0.0f);
  _qd = _q;

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  _outputFile.close();
  
  ros::shutdown();
}


void PreliminaryExperiment::stopNode(int sig)
{
  me->_stop = true;
}


void PreliminaryExperiment::calibration()
{
  if(_calibrationCount< CALIBRATION_COUNT)
  {
    _markersPosition0 = (_calibrationCount*_markersPosition0+_markersPosition)/(_calibrationCount+1);
    _calibrationCount++;
    _calibrationOK = false;
    if(_calibrationCount == 1)
    {
      std::cerr << "init calibration" << std::endl;
    }
    else if(_calibrationCount == CALIBRATION_COUNT)
    {
      std::cerr << "end calibration" << std::endl;      
    }
  }
  else
  {
    _calibrationOK = true;
  }
}


void PreliminaryExperiment::computeAngles()
{
  Eigen::Vector3f BA;
  Eigen::Vector3f BC;
  BA = _markersPosition.col(TOE)-_markersPosition.col(ANKLE);
  BC = _markersPosition.col(TIBIA)-_markersPosition.col(ANKLE);

  float angle = std::acos(BA.dot(BC)/(BA.norm()*BC.norm()));

  // std::cerr << angle*180.0f/M_PI << std::endl;
  
}


void PreliminaryExperiment::publishData()
{

  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vd(0);
  _msgDesiredTwist.linear.y  = _vd(1);
  _msgDesiredTwist.linear.z  = _vd(2);
  _msgDesiredTwist.angular.x = _omegad(0);
  _msgDesiredTwist.angular.y = _omegad(1);
  _msgDesiredTwist.angular.z = _omegad(2);
  _pubDesiredTwist.publish(_msgDesiredTwist);

  // // Publish desired orientation (passive ds controller)
  _msgDesiredOrientation.w = _qd(0);
  _msgDesiredOrientation.x = _qd(1);
  _msgDesiredOrientation.y = _qd(2);
  _msgDesiredOrientation.z = _qd(3);
  _pubDesiredOrientation.publish(_msgDesiredOrientation);

  // _msgTaskAttractor.header.frame_id = "world";
  // _msgTaskAttractor.header.stamp = ros::Time::now();
  // _msgTaskAttractor.point.x = _taskAttractor(0);
  // _msgTaskAttractor.point.y = _taskAttractor(1);
  // _msgTaskAttractor.point.z = _taskAttractor(2);
  // _pubTaskAttractor.publish(_msgTaskAttractor);

  // _msgSurfaceMarker.header.frame_id = "world";
  // _msgSurfaceMarker.header.stamp = ros::Time();
  // Eigen::Vector3f center;
  // if(_useOptitrack)
  // {
  //   center = _p1+0.5f*(_p2-_p1)+0.5f*(_p3-_p1); 
  // }
  // else
  // {
  //   center << -0.4f, 0.0f, 0.186f;
  // }
  // _msgSurfaceMarker.pose.position.x = center(0);
  // _msgSurfaceMarker.pose.position.y = center(1);
  // _msgSurfaceMarker.pose.position.z = center(2);
  // Eigen::Vector3f u,v,n;
  // u = _p3-_p1;
  // v = _p2-_p1;
  // u /= u.norm();
  // v /= v.norm();
  // n = u.cross(v);
  // Eigen::Matrix3f R;
  // R.col(0) = u;
  // R.col(1) = v;
  // R.col(2) = n;
  // Eigen::Vector4f q = rotationMatrixToQuaternion(R);


  // _msgSurfaceMarker.pose.orientation.x = q(1);
  // _msgSurfaceMarker.pose.orientation.y = q(2);
  // _msgSurfaceMarker.pose.orientation.z = q(3);
  // _msgSurfaceMarker.pose.orientation.w = q(0);

  // _pubMarker.publish(_msgSurfaceMarker);

  // _msgArrowMarker.points.clear();
  // geometry_msgs::Point p1, p2;
  // p1.x = _x(0);
  // p1.y = _x(1);
  // p1.z = _x(2);
  // p2.x = _x(0)+0.3f*_e1(0);
  // p2.y = _x(1)+0.3f*_e1(1);
  // p2.z = _x(2)+0.3f*_e1(2);
  // _msgArrowMarker.points.push_back(p1);
  // _msgArrowMarker.points.push_back(p2);
  // _pubMarker.publish(_msgArrowMarker);
}

void PreliminaryExperiment::logData()
{
  _outputFile << ros::Time::now() << " "
              << _markersPosition.col(TOE).transpose() << " "
              << _markersPosition.col(ANKLE).transpose() << " "
              << _markersPosition.col(TIBIA).transpose() << " "
              << _markersTracked.transpose() << " "
              << _markersSequenceID(TOE) << std::endl;
}

void PreliminaryExperiment::updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  // Update end effecotr pose (position+orientation)
  _x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_q);
  _x = _x+_toolOffset*_wRb.col(2);

  if(!_firstRobotPoseReceived)
  {
    _firstRobotPoseReceived = true;
    _xd = _x;
    _qd = _q;
    _vd.setConstant(0.0f);
  }
}


void PreliminaryExperiment::updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;
}


void PreliminaryExperiment::updateToePose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstToe = false;

  if(!firstToe)
  {
    std::cerr << "Get first toe" << std::endl;
    _markersCount++;
    firstToe = true;
  }

  _markersSequenceID(TOE) = msg->header.seq;
  _markersTracked(TOE) = checkTrackedMarker(_markersPosition.col(TOE)(0),msg->pose.position.x);
  _markersPosition.col(TOE) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  // std::cerr <<  msg->header.seq << std::endl;

}


void PreliminaryExperiment::updateHeelPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstHeel = false;


  if(!firstHeel)
  {
    std::cerr << "Get first heel" << std::endl;
    _markersCount++;
    firstHeel = true;
  }

  _markersSequenceID(HEEL) = msg->header.seq;
  _markersTracked(HEEL) = checkTrackedMarker(_markersPosition.col(HEEL)(0),msg->pose.position.x);
  _markersPosition.col(HEEL) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}


void PreliminaryExperiment::updateAnklePose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstAnkle = false;
  if(!firstAnkle)
  {

    std::cerr << "Get first ankle" << std::endl;
    _markersCount++;
    firstAnkle = true;
  }

  _markersSequenceID(ANKLE) = msg->header.seq;
  _markersTracked(ANKLE) = checkTrackedMarker(_markersPosition.col(ANKLE)(0),msg->pose.position.x);
  _markersPosition.col(ANKLE) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}


void PreliminaryExperiment::updateTibiaPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstTibia = false;
  if(!firstTibia)
  {
    std::cerr << "Get first tibia" << std::endl;
    _markersCount++;
    firstTibia = true;
  }

  _markersSequenceID(TIBIA) = msg->header.seq;
  _markersTracked(TIBIA) = checkTrackedMarker(_markersPosition.col(TIBIA)(0),msg->pose.position.x);
  _markersPosition.col(TIBIA) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}


void PreliminaryExperiment::updateKneePose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstKnee = false;
  if(!firstKnee)
  {
    std::cerr << "Get first knee" << std::endl;
    _markersCount++;
    firstKnee = true;
  }

  _markersSequenceID(KNEE) = msg->header.seq;
  _markersTracked(KNEE) = checkTrackedMarker(_markersPosition.col(KNEE)(0),msg->pose.position.x);
  _markersPosition.col(KNEE) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}


void PreliminaryExperiment::updateThighPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstThigh = false;
  if(!firstThigh)
  {
    std::cerr << "Get first thigh" << std::endl;
    _markersCount++;
    firstThigh = true;
  }

  _markersSequenceID(THIGH) = msg->header.seq;
  _markersTracked(THIGH) = checkTrackedMarker(_markersPosition.col(THIGH)(0),msg->pose.position.x);
  _markersPosition.col(THIGH) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}

uint16_t PreliminaryExperiment::checkTrackedMarker(float a, float b)
{
  if(fabs(a-b)< FLT_EPSILON)
  {
    return 0;
  }
  else
  {
    return 1;

  }
}

void PreliminaryExperiment::updateHipPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static bool firstHip = false;
  if(!firstHip)
  {
    std::cerr << "Get first hip" << std::endl;
    _markersCount++;
    firstHip = true;
  }

  _markersPosition.col(HIP) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _markersSequenceID(HIP) = msg->header.seq;
}


void PreliminaryExperiment::dynamicReconfigureCallback(hp_preliminary_experiment::preliminaryExperiment_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");
  _doCalibration = config.doCalibration;
}



Eigen::Matrix3f PreliminaryExperiment::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}
