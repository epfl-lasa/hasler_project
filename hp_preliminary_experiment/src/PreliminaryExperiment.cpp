#include "PreliminaryExperiment.h"

PreliminaryExperiment* PreliminaryExperiment::me = NULL;

PreliminaryExperiment::PreliminaryExperiment(ros::NodeHandle &n, double frequency, bool calibration):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_calibration(calibration)
{
  me = this;

  _markersPosition.setConstant(0.0f);
  _markersPosition0.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _allMarkersPositionReceived = false;
  _stop = false;
  _initializationOK = false;
  _facingScreen = false;

  _markersCount = 0;
  _averageCount = 0;
  _currentSequenceID = 0;
  _planeData.resize(0);

  _cr.c = 0;
  _cr.n.setConstant(0.0f);
  _cr.u << 1.0f,0.0f,0.0f;
  _cr.v << 0.0f,1.0f,0.0f;
  _cr.Pcenter.setConstant(0.0f);

  _chaserPosition.setConstant(0.0f);



}


bool PreliminaryExperiment::init() 
{
  // Subscriber definitions
  // _subOptitrackHip = _n.subscribe("/optitrack/hip/pose", 1, &PreliminaryExperiment::updateHipPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackThigh = _n.subscribe("/optitrack/thigh/pose", 1, &PreliminaryExperiment::updateThighPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackKnee = _n.subscribe("/optitrack/knee/pose", 1, &PreliminaryExperiment::updateKneePose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackTibia = _n.subscribe("/optitrack/tibia/pose", 1, &PreliminaryExperiment::updateTibiaPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackAnkle = _n.subscribe("/optitrack/ankle/pose", 1, &PreliminaryExperiment::updateAnklePose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackHeel = _n.subscribe("/optitrack/heel/pose", 1, &PreliminaryExperiment::updateHeelPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackToe = _n.subscribe("/optitrack/toe/pose", 1, &PreliminaryExperiment::updateToePose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubChaserPose = _n.advertise<geometry_msgs::PoseStamped>("/chaser/pose", 1);

  signal(SIGINT,PreliminaryExperiment::stopNode);

  if(_calibration)
  {
    _outputFile.open("src/hasler_project/hp_preliminary_experiment/data.txt");
  }
  else
  {
    _inputFile.open("src/hasler_project/hp_preliminary_experiment/result.txt");
    if(!_inputFile.is_open())
    {
      ROS_INFO("Cannot open file with calibration result");
      return false;
    }
    else
    {
      _inputFile >> _cr.c >> 
                    _cr.n(0) >> _cr.n(1) >> _cr.n(2) >>
                    _cr.u(0) >> _cr.u(1) >> _cr.u(2) >>
                    _cr.v(0) >> _cr.v(1) >> _cr.v(2) >>
                    _cr.Pcenter(0) >> _cr.Pcenter(1) >> _cr.Pcenter(2);

      std::cerr << _cr.c << std::endl;
      std::cerr << _cr.n.transpose() << std::endl;
      std::cerr << _cr.u.transpose() << std::endl;
      std::cerr << _cr.v.transpose() << std::endl;
      std::cerr << _cr.Pcenter.transpose() << std::endl;
    }
  }

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

      // Compute angles from marker positions after initialization
      if(!_initializationOK)
      {
        initializeData();
      }
      else if(_initializationOK &&_calibration)
      {
        // Compute angles
        computeAngles();

        // Add plane fitting data
        addPlaneFittingData();

        // Log data
        logCalibrationData();
      }
      else if(_initializationOK && !_calibration)
      {
        // Compute chaser pose to be send to RVIZ
        computeChaserPose();
      
        // Publish data to topics
        publishData();
      }
      
      _mutex.unlock();
    }

    if(_markersCount == NB_MARKERS)
    {
      _allMarkersPositionReceived = true;
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();

  if(_calibration)
  {
    // Close file of calibration data
    if(_outputFile.is_open())
    {
      _outputFile.close();
    }
    // Compute plane from calibration data
    computePlane();
    // Log calibration result
    logCalibrationResult();
  }
  else
  {
    _inputFile.close();
  }
}


void PreliminaryExperiment::stopNode(int sig)
{
  me->_stop = true;
}


void PreliminaryExperiment::initializeData()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
    _averageCount++;
    _initializationOK = false;
    if(_averageCount == 1)
    {
      std::cerr << "init initialization" << std::endl;
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      std::cerr << "end initialization" << std::endl;      
    }
  }
  else
  {
    _initializationOK = true;
  }
}


void PreliminaryExperiment::computeAngles()
{
  Eigen::Vector3f BA;
  Eigen::Vector3f BC;
  BA = _markersPosition.col(TOE)-_markersPosition.col(ANKLE);
  BC = _markersPosition.col(TIBIA)-_markersPosition.col(ANKLE);

  float angle = std::acos(BA.dot(BC)/(BA.norm()*BC.norm()));  
}


void PreliminaryExperiment::computeChaserPose()
{
  Eigen::Matrix3f wRp;

  float scaleX, scaleY; 
  Eigen::Vector3f u,v;
  u = _cr.u.normalized();
  v = _cr.v.normalized();
  scaleX = 10.0f/_cr.u.norm();
  scaleY = 10.0f/_cr.v.norm();

  Eigen::Vector3f temp, tempProj;
  temp = _markersPosition.col(TOE);
  tempProj = temp-(temp.dot(_cr.n)+_cr.c)*_cr.n/_cr.n.squaredNorm();
  // std::cerr << (tempProj-_cr.Pcenter).dot(_cr.n) << std::endl;
  // std::cerr << _cr.u.norm() << std::endl;
  // std::cerr << _cr.v.norm() << std::endl;

  _chaserPosition.setConstant(0.0f);
  _chaserPosition(0) = scaleX*(tempProj-_cr.Pcenter).dot(u);
  _chaserPosition(1) = scaleY*(tempProj-_cr.Pcenter).dot(v);

  std::cerr << _chaserPosition.transpose() << std::endl;

}


void PreliminaryExperiment::publishData()
{
  _msgChaserPose.header.frame_id = "world";
  _msgChaserPose.header.stamp = ros::Time::now();
  _msgChaserPose.pose.position.x = _chaserPosition(0);
  _msgChaserPose.pose.position.y = _chaserPosition(1);
  _msgChaserPose.pose.position.z = 0.0f;
  _msgChaserPose.pose.orientation.x = 0.0f;
  _msgChaserPose.pose.orientation.y = 0.0f;
  _msgChaserPose.pose.orientation.z = 0.0f;
  _msgChaserPose.pose.orientation.w = 1.0f;
  _pubChaserPose.publish(_msgChaserPose);
}


void PreliminaryExperiment::logCalibrationData()
{
  _outputFile << ros::Time::now() << " "
              << _markersPosition.col(TOE).transpose() << " "
              << _markersPosition.col(ANKLE).transpose() << " "
              << _markersPosition.col(TIBIA).transpose() << " "
              << _markersTracked.transpose() << " "
              << _markersSequenceID(TOE) << std::endl;
}


void PreliminaryExperiment::logCalibrationResult()
{

  _outputFile.open("src/hasler_project/hp_preliminary_experiment/result.txt");
  
  _outputFile << _cr.c << std::endl
              << _cr.n.transpose() << std::endl
              << _cr.u.transpose() << std::endl
              << _cr.v.transpose() << std::endl
              << _cr.Pcenter.transpose() << std::endl;

  _outputFile.close();
}


void PreliminaryExperiment::addPlaneFittingData()
{
  if(_markersTracked.sum()==NB_MARKERS)
  {
    if(_planeData.size()==0)
    {
      _planeData.push_back(_markersPosition.col(TOE));  
      _currentSequenceID = _markersSequenceID(TOE);
    }
    else
    {
      if(_markersSequenceID(TOE)!= _currentSequenceID)
      {
        _planeData.push_back(_markersPosition.col(TOE));
        _currentSequenceID = _markersSequenceID(TOE);
      }
    }
  }
}


void PreliminaryExperiment::computePlane()
{
  // Plane equation: ax+by+c=z
  // A = [x1 y1 1
  //      x2 y2 1
  //      |  |  |
  //      xn yn 1]

  // x = [a b c]'
  // B = [z1 z2 - zn]'
  // x = inv(A'A)*A'B

  std::cerr << "Start plane fitting ..." << std::endl;
  std::cerr << "Number of points: " << _planeData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,3> A;
  A.resize(_planeData.size(),3);
  Eigen::Vector3f x;
  Eigen::VectorXf B;
  B.resize(_planeData.size());
  for(uint32_t k = 0; k < _planeData.size(); k++)
  {
    A.row(k) << _planeData[k](0),_planeData[k](1), 1.0f;
    B(k) = _planeData[k](2);
  }

  // A.col(0).array() -= A.col(0).mean();
  // A.col(1).array() -= A.col(1).mean();
  // B.array() -= B.mean();

  x = ((A.transpose()*A).inverse())*A.transpose()*B;
  float a = x(0);
  float b = x(1);
  float c = x(2);


  std::cerr << "Coefficients: a = " << a << " b = " << b << " c = " << c << std::endl;
  Eigen::Vector3f n;
  n << a, b, -1.0f;
  std::cerr << "Normal vector: " << n.normalized().transpose() << std::endl;

  float xmin = A.col(0).minCoeff();
  float xmax = A.col(0).maxCoeff();
  float ymin = A.col(1).minCoeff();
  float ymax = A.col(1).maxCoeff();

  Eigen::Vector3f P1,P2,P3,P4;
  P1 << xmin, ymin, a*xmin+b*ymin+c;
  P2 << xmin, ymax, a*xmin+b*ymax+c;
  P3 << xmax, ymax, a*xmax+b*ymax+c;
  P4 << xmax, ymin, a*xmax+b*ymin+c;

  std::cerr << "Plane corners:" << std::endl;
  std::cerr << "P1: " << P1.transpose() << std::endl;
  std::cerr << "P2: " << P2.transpose() << std::endl;
  std::cerr << "P3: " << P3.transpose() << std::endl;
  std::cerr << "P4: " << P4.transpose() << std::endl;

  Eigen::Vector3f xmean, xmeanProj, Pcenter;
  xmean << A.col(0).mean(),A.col(1).mean(),B.mean();
  xmeanProj = xmean-(xmean.dot(n)+c)*n/n.squaredNorm();
  
  Pcenter = (P1+P2+P3+P4)/4.0f;
  std::cerr << "Plane center: " << Pcenter.transpose() << std::endl;

  _cr.c = c;
  _cr.n = n;
  if(!_facingScreen)
  {
    _cr.u  = P1-P2;
    _cr.v  = P3-P2;
  }
  else
  {
    _cr.u  = P2-P3;
    _cr.v  = P4-P3;    
  }
  _cr.Pcenter = Pcenter;

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