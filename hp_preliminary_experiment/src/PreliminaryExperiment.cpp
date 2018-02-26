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
  _footData.resize(0);

  _pcr.c = 0;
  _pcr.n.setConstant(0.0f);
  _pcr.u << 1.0f,0.0f,0.0f;
  _pcr.v << 0.0f,1.0f,0.0f;
  _pcr.Pcenter.setConstant(0.0f);

  _chaserPosition.setConstant(0.0f);

  _fittingMethod = PLANE;

}


bool PreliminaryExperiment::init() 
{
  // Subscriber definitions
  _subOptitrackHip = _n.subscribe("/optitrack/hip/pose", 1, &PreliminaryExperiment::updateHipPose,this,ros::TransportHints().reliable().tcpNoDelay());
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
    if(_fittingMethod == PLANE)
    {
      _inputFile.open("src/hasler_project/hp_preliminary_experiment/result_plane.txt");
    }
    else
    {
      _inputFile.open("src/hasler_project/hp_preliminary_experiment/result_sphere.txt");
    }

    if(!_inputFile.is_open())
    {
      ROS_INFO("Cannot open file with calibration result");
      return false;
    }
    else
    {
      if(_fittingMethod == PLANE)
      {
        _inputFile >> _pcr.c >> 
                      _pcr.n(0) >> _pcr.n(1) >> _pcr.n(2) >>
                      _pcr.u(0) >> _pcr.u(1) >> _pcr.u(2) >>
                      _pcr.v(0) >> _pcr.v(1) >> _pcr.v(2) >>
                      _pcr.Pcenter(0) >> _pcr.Pcenter(1) >> _pcr.Pcenter(2);

        std::cerr << _pcr.c << std::endl;
        std::cerr << _pcr.n.transpose() << std::endl;
        std::cerr << _pcr.u.transpose() << std::endl;
        std::cerr << _pcr.v.transpose() << std::endl;
        std::cerr << _pcr.Pcenter.transpose() << std::endl;
      }
      else if(_fittingMethod == SPHERE)
      {
        _inputFile >> _scr.center(0) >> _scr.center(1) >> _scr.center(2) >>
                      _scr.radius >>
                      _scr.phiMean >>
                      _scr.thetaMean >>
                      _scr.arcLengthX >>
                      _scr.arcLengthY;

        std::cerr << _scr.center.transpose() << std::endl;
        std::cerr << _scr.radius << std::endl;
        std::cerr << _scr.phiMean << std::endl;
        std::cerr << _scr.thetaMean << std::endl;
        std::cerr << _scr.arcLengthX << std::endl;
        std::cerr << _scr.arcLengthY << std::endl;
      }

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
        // computeAngles();

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

    // Compute plane and sphere fitting
    computePlane();

    computeSphere();

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


  float scaleX, scaleY; 
  _chaserPosition.setConstant(0.0f);

  if(_fittingMethod == PLANE)
  {
    Eigen::Vector3f u,v;
    u = _pcr.u.normalized();
    v = _pcr.v.normalized();
    scaleX = 10.0f/_pcr.u.norm();
    scaleY = 10.0f/_pcr.v.norm();

    Eigen::Vector3f temp, tempProj;
    temp = _markersPosition.col(TOE);
    tempProj = temp-(temp.dot(_pcr.n)+_pcr.c)*_pcr.n/_pcr.n.squaredNorm();

    _chaserPosition(0) = scaleX*(tempProj-_pcr.Pcenter).dot(u);
    _chaserPosition(1) = scaleY*(tempProj-_pcr.Pcenter).dot(v);
  }
  else if(_fittingMethod == SPHERE)
  {
    scaleX = 10.0f/_scr.arcLengthX;
    scaleY = 10.0f/_scr.arcLengthY;
    Eigen::Vector3f proj = _scr.center+_scr.radius*(_markersPosition.col(TOE)-_scr.center).normalized();
    Eigen::Vector3f e = proj-_scr.center;
    float phi = std::atan2(e.segment(0,2).norm(),e(2));
    float theta = std::atan2(e(1),e(0));

    _chaserPosition(0) = -scaleX*_scr.radius*std::sin(phi)*(theta-_scr.thetaMean);
    _chaserPosition(1) = -scaleY*_scr.radius*(phi-_scr.phiMean);
  }

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
  // _outputFile << ros::Time::now() << " "
  //             << _markersPosition.col(TOE).transpose() << " "
  //             << _markersPosition.col(ANKLE).transpose() << " "
  //             << _markersPosition.col(TIBIA).transpose() << " "
  //             << _markersTracked.transpose() << " "
  //             << _markersSequenceID(TOE) << std::endl;

  _outputFile << ros::Time::now() << " "
              << _markersPosition.col(TOE).transpose() << " "
              // << _markersPosition.col(ANKLE).transpose() << " "
              // << _markersPosition.col(TIBIA).transpose() << " "
              << _markersTracked.transpose() << " "
              << _markersSequenceID(TOE) << std::endl;
}


void PreliminaryExperiment::logCalibrationResult()
{

  _outputFile.open("src/hasler_project/hp_preliminary_experiment/result_plane.txt");
  
  _outputFile << _pcr.c << std::endl
              << _pcr.n.transpose() << std::endl
              << _pcr.u.transpose() << std::endl
              << _pcr.v.transpose() << std::endl
              << _pcr.Pcenter.transpose() << std::endl;

  _outputFile.close();

  _outputFile.open("src/hasler_project/hp_preliminary_experiment/result_sphere.txt");

  _outputFile << _scr.center.transpose() << std::endl
                << _scr.radius << std::endl
                << _scr.phiMean << std::endl
                << _scr.thetaMean << std::endl
                << _scr.arcLengthX << std::endl
                << _scr.arcLengthY << std::endl;

  _outputFile.close();
}


void PreliminaryExperiment::addPlaneFittingData()
{
  if(_markersTracked.sum()==NB_MARKERS)
  {
    if(_footData.size()==0)
    {
      _footData.push_back(_markersPosition.col(TOE));  
      _currentSequenceID = _markersSequenceID(TOE);
    }
    else
    {
      if(_markersSequenceID(TOE)!= _currentSequenceID)
      {
        _footData.push_back(_markersPosition.col(TOE));
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
  std::cerr << "Number of points: " << _footData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,3> A;
  A.resize(_footData.size(),3);
  Eigen::Vector3f x;
  Eigen::VectorXf B;
  B.resize(_footData.size());
  for(uint32_t k = 0; k < _footData.size(); k++)
  {
    A.row(k) << _footData[k](0),_footData[k](1), 1.0f;
    B(k) = _footData[k](2);
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

  _pcr.c = c;
  _pcr.n = n;
  if(!_facingScreen)
  {
    _pcr.u  = P1-P2;
    _pcr.v  = P3-P2;
  }
  else
  {
    _pcr.u  = P2-P3;
    _pcr.v  = P4-P3;    
  }
  _pcr.Pcenter = Pcenter;

}


void PreliminaryExperiment::computeSphere()
{
  // Sphere equation: (x-x0)^2+(y-y0)^2+(z-z0)^2 = R^2
  // A = [2x1 2y1 2z1 1
  //      2x2 2y2 2z2 1
  //      |  |  |     |
  //      2xn 2yn 2zn 1]

  // x = [x0 y0 z0  R^2-x0^2-y0^2-z0^2]'
  // B = [x1^2+y1^2+z1^2 x2^2+y2^2+z2^2 - xn^2+yn^2+zn^2]'
  // x = inv(A'A)*A'B

  std::cerr << "Start plane fitting ..." << std::endl;
  std::cerr << "Number of points: " << _footData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,4> A;
  A.resize(_footData.size(),4);
  Eigen::Vector4f x;
  Eigen::VectorXf B;
  B.resize(_footData.size());
  for(uint32_t k = 0; k < _footData.size(); k++)
  {
    A.row(k) << 2.0f*_footData[k](0), 2.0f*_footData[k](1), 2.0f*_footData[k](2), 1.0f;
    B(k) = _footData[k].squaredNorm();
  }

  // std::cout << A << std::endl;
  // std::cout << B << std::endl;
  // A.col(0).array() -= A.col(0).mean();
  // A.col(1).array() -= A.col(1).mean();
  // B.array() -= B.mean();

  x = A.colPivHouseholderQr().solve(B);
  // x = ((A.transpose()*A).inverse())*A.transpose()*B;
  Eigen::Vector3f center;
  float radius;
  center = x.segment(0,3);
  radius = std::sqrt(x(3)+center.squaredNorm());


  // Find phi theta range in spherical coordinates
  Eigen::VectorXf phi, theta;
  phi.resize(_footData.size());
  theta.resize(_footData.size());

  for(int k = 0; k < _footData.size(); k++)
  {
    Eigen::Vector3f proj;
    proj = center+radius*(_footData[k]-center).normalized();
    Eigen::Vector3f e = proj-center;
    phi(k) = std::atan2(e.segment(0,2).norm(),e(2));
    theta(k) = std::atan2(e(1),e(0));
  }

  float phiMax = phi.maxCoeff();
  float phiMin = phi.minCoeff();
  float phiMean = (phiMax+phiMin)/2.0f;
  float thetaMax = theta.maxCoeff();
  float thetaMin = theta.minCoeff();
  float thetaMean = (thetaMax+thetaMin)/2.0f;

  float rho1 = radius*std::sin(phiMin);
  float rho2 = radius*std::sin(phiMax);

  float dtheta = thetaMax-thetaMin;
  float dphi = phiMax-phiMin;
  float arcLengthX = std::min(rho1*dtheta,rho2*dtheta);
  float arcLengthY = radius*dphi;

  _scr.center = center;
  _scr.radius = radius;
  _scr.phiMean = phiMean;
  _scr.thetaMean = thetaMean;
  _scr.arcLengthX = arcLengthX;
  _scr.arcLengthY = arcLengthY;
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