#include "KinectRecording.h"

KinectRecording* KinectRecording::me = NULL;

KinectRecording::KinectRecording(ros::NodeHandle &n, double frequency, bool calibration):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_calibration(calibration),
_filter(3,3,9,1.0f/frequency)
{
  me = this;

  _markersPosition0.setConstant(0.0f);
  _markersPosition.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _allMarkersReceived = false;
  _stop = false;
  _initializationOK = false;
  _facingScreen = false;

  _averageCount = 0;
  _currentSequenceID = 0;
  _surfaceData.resize(0);

  _cr.c = 0;
  _cr.n.setConstant(0.0f);
  _cr.u << 1.0f,0.0f,0.0f;
  _cr.v << 0.0f,1.0f,0.0f;
  _cr.Pcenter.setConstant(0.0f);

  _chaserPosition.setConstant(0.0f);

  _R.setIdentity();

  _nbData = 0;



}


bool KinectRecording::init() 
{
  // Subscriber definitions
  // _subOptitrackHip = _n.subscribe("/optitrack/hip/pose", 1, &KinectRecording::updateHipPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subKinectToe = _n.subscribe("/ar_pose_marker", 1, &KinectRecording::updateMarkersPose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubChaserPose = _n.advertise<geometry_msgs::PoseStamped>("/chaser/pose", 1);

  signal(SIGINT,KinectRecording::stopNode);

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
                    _cr.Pcenter(0) >> _cr.Pcenter(1) >> _cr.Pcenter(2) >>
                    _cr.R(0,0) >> _cr.R(0,1) >> _cr.R(0,2) >>
                    _cr.R(1,0) >> _cr.R(1,1) >> _cr.R(1,2) >>
                    _cr.R(2,0) >> _cr.R(2,1) >> _cr.R(2,2);

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


void KinectRecording::run()
{
  while (!_stop) 
  {
    if(_allMarkersReceived)
    {

      _mutex.lock();

      // Compute angles from marker positions after initialization
      if(!_initializationOK)
      {
        initializeData();
      }
      else if(_initializationOK &&_calibration)
      {
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

    ros::spinOnce();

    _loopRate.sleep();
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  ros::shutdown();

  if(_initializationOK &&_calibration)
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


void KinectRecording::stopNode(int sig)
{
  me->_stop = true;
}


void KinectRecording::initializeData()
{
  if(_averageCount< AVERAGE_COUNT)
  {
    if(_markersTracked.segment(1,3).sum() == NB_MARKERS-1)
    {
      _markersPosition0 = (_averageCount*_markersPosition0+_markersPosition)/(_averageCount+1);
      _averageCount++;
    }

    if(_averageCount == 1)
    {
      std::cerr << "init initialization" << std::endl;
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      std::cerr << "end initialization" << std::endl;  
      std::cerr << _markersPosition0 << std::endl;
      Eigen::Vector3f e1 = _markersPosition0.col(PB)-_markersPosition0.col(PA); 
      e1.normalize();   
      Eigen::Vector3f e2 = _markersPosition0.col(PC)-_markersPosition0.col(PA);  
      e2.normalize();
      Eigen::Vector3f e3 = e1.cross(e2); 
      e3.normalize();
      _R.col(0) = e1;
      _R.col(1) = e2;
      _R.col(2) = e3;
      std::cerr << _R.transpose() << std::endl;

    }
  }
  else
  {
    _initializationOK = true;
  }
}


void KinectRecording::computeChaserPose()
{

  static bool _first = false;
  Eigen::Matrix3f wRp;

  float scaleX, scaleY; 
  Eigen::Vector3f u,v;
  u = _cr.u.normalized();
  v = _cr.v.normalized();
  scaleX = 10.0f/_cr.u.norm();
  scaleY = 10.0f/_cr.v.norm();

  SGF::Vec tempF(3);
  tempF = _markersPosition.col(TOE);
  _filter.AddData(_markersPosition.col(TOE));

  Eigen::Vector3f temp, tempProj;
  _filter.GetOutput(0,tempF);
  temp = tempF;
  temp = _cr.R.transpose()*temp;
  temp = _cr.R.transpose()*_markersPosition.col(TOE);
  tempProj = temp-(temp.dot(_cr.n)+_cr.c)*_cr.n/_cr.n.squaredNorm();

  // if(!_first)
  // {
  //   _first = true;
  _chaserPosition(0) = scaleX*(tempProj-_cr.Pcenter).dot(u);
  _chaserPosition(1) = scaleY*(tempProj-_cr.Pcenter).dot(v);
  _chaserPosition(2) = 0.0f;

  // }

  // float alpha = 0;
  // _chaserPosition(0) = _chaserPosition(0)*alpha+(1.0f-alpha)*scaleX*(tempProj-_cr.Pcenter).dot(u);
  // _chaserPosition(1) = _chaserPosition(1)*alpha+(1.0f-alpha)*scaleY*(tempProj-_cr.Pcenter).dot(v);
  // _chaserPosition(2) = 0.0f;

  std::cerr << _chaserPosition.transpose() << std::endl;

}


void KinectRecording::publishData()
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


void KinectRecording::logCalibrationData()
{
  _outputFile << ros::Time::now() << " "
              << (_R.transpose()*_markersPosition.col(TOE)).transpose() << " "
              << _markersPosition.col(PA).transpose() << " "
              << _markersPosition.col(PB).transpose() << " "
              << _markersPosition.col(PC).transpose() << " "
              << _markersTracked.transpose() << " "
              << _markersSequenceID(TOE) << std::endl;
}


void KinectRecording::logCalibrationResult()
{

  _outputFile.open("src/hasler_project/hp_preliminary_experiment/result.txt");
  
  _outputFile << _cr.c << std::endl
              << _cr.n.transpose() << std::endl
              << _cr.u.transpose() << std::endl
              << _cr.v.transpose() << std::endl
              << _cr.Pcenter.transpose() << std::endl
              << _cr.R << std::endl;

  _outputFile.close();
}


void KinectRecording::addPlaneFittingData()
{
  if(_markersTracked(TOE))
  {
    if(_surfaceData.size()==0)
    {
      _surfaceData.push_back(_markersPosition.col(TOE));  
      _currentSequenceID = _markersSequenceID(TOE);
    }
    else
    {
      if(_markersSequenceID(TOE)!= _currentSequenceID)
      {
        _surfaceData.push_back(_markersPosition.col(TOE));
        _currentSequenceID = _markersSequenceID(TOE);
        std::cerr << "Nb Data: " << _surfaceData.size() << std::endl;
      }
    }
  }
}


void KinectRecording::computePlane()
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
  std::cerr << "Number of points: " << _surfaceData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,3> A;
  A.resize(_surfaceData.size(),3);
  Eigen::Vector3f x;
  Eigen::VectorXf B;
  B.resize(_surfaceData.size());
  Eigen::Vector3f temp;
  for(uint32_t k = 0; k < _surfaceData.size(); k++)
  {
    temp = _surfaceData[k];
    temp = _R.transpose()*temp;
    A.row(k) << temp(0),temp(1), 1.0f;
    B(k) = temp(2);
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
  _cr.u  = P4-P1;
  _cr.v  = P2-P1;
  _cr.Pcenter = Pcenter;
  _cr.R = _R;
}


void KinectRecording::updateMarkersPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if(!_allMarkersReceived && msg->markers.size()>0)
  {
    std::cerr << "Get first toe" << std::endl;
    _allMarkersReceived = true;
  }

  _markersTracked.setConstant(0);
  for(int k = 0; k < msg->markers.size(); k++) 
  {
    int m = -1;
    if(msg->markers[k].id == 0)
    {
      m = TOE;
    }
    else if(msg->markers[k].id == 3)
    {
      m = PA;
    }
    else if(msg->markers[k].id == 6)
    {
      m = PB;
    }
    else if(msg->markers[k].id == 4)
    {
      m = PC;
    }

    if(m !=-1)
    {
      _markersSequenceID(m) = msg->header.seq;
      _markersTracked(m)= checkTrackedMarker(_markersPosition.col(m)(0),msg->markers[k].pose.pose.position.x);
      _markersPosition.col(m) << msg->markers[k].pose.pose.position.x, msg->markers[k].pose.pose.position.y, msg->markers[k].pose.pose.position.z;
    }
  }
}


uint16_t KinectRecording::checkTrackedMarker(float a, float b)
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

