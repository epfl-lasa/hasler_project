#include "KinectRecording.h"

KinectRecording* KinectRecording::me = NULL;

KinectRecording::KinectRecording(ros::NodeHandle &n, double frequency, std::string subjectName, ExecutionMode executionMode, FittingMethod fittingMethod):
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_subjectName(subjectName),
_executionMode(executionMode),
_fittingMethod(fittingMethod),
_filter(3,1,4,1.0f/frequency)
{
  me = this;

  _markersPosition0.setConstant(0.0f);
  _markersPosition.setConstant(0.0f);
  _markersSequenceID.setConstant(0);
  _markersTracked.setConstant(0);

  _allMarkersReceived = false;
  _stop = false;
  _initializationOK = false;
  _useFiltering = true;

  _averageCount = 0;
  _currentSequenceID = 0;
  _calibrationData.resize(0);

  _pcr.c = 0;
  _pcr.n.setConstant(0.0f);
  _pcr.u << 1.0f,0.0f,0.0f;
  _pcr.v << 0.0f,1.0f,0.0f;
  _pcr.Pcenter.setConstant(0.0f);

  _chaserPosition.setConstant(0.0f);

  _R.setIdentity();
}


bool KinectRecording::init() 
{
  // Subscriber definitions
  // _subOptitrackHip = _n.subscribe("/optitrack/hip/pose", 1, &KinectRecording::updateHipPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subKinectToe = _n.subscribe("/ar_pose_marker", 1, &KinectRecording::updateMarkersPose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubChaserPose = _n.advertise<geometry_msgs::PoseStamped>("/chaser/pose", 1);

  signal(SIGINT,KinectRecording::stopNode);

  if(_executionMode == CALIBRATION)
  {
    ROS_INFO("Calibration execution mode");
    _outputFile.open(ros::package::getPath("hp_preliminary_experiment")+"/data/"+_subjectName+"_calibration_data.txt");  
    if(!_outputFile.is_open())
    {
      ROS_ERROR("Cannot open file calibration result file");
      return false;
    }
  }
  else if(_executionMode == GAME)
  {
    ROS_INFO("Game execution mode");
    if(_fittingMethod == PLANE)
    {
      _inputFile.open(ros::package::getPath("hp_preliminary_experiment")+"/data/"+_subjectName+"_result_plane.txt");
    }
    else
    {
      _inputFile.open(ros::package::getPath("hp_preliminary_experiment")+"/data/"+_subjectName+"_result_sphere.txt");
    }

    if(!_inputFile.is_open())
    {
      ROS_ERROR("Cannot open calibration result file");
      return false;
    }
    else
    {
      bool isEmpty = _inputFile.peek() == EOF;

      std::cerr <<(int) isEmpty << std::endl;

      if(_fittingMethod == PLANE)
      {
        ROS_INFO("Use plane fitting method");
        _inputFile >> _pcr.c >> 
                      _pcr.n(0) >> _pcr.n(1) >> _pcr.n(2) >>
                      _pcr.u(0) >> _pcr.u(1) >> _pcr.u(2) >>
                      _pcr.v(0) >> _pcr.v(1) >> _pcr.v(2) >>
                      _pcr.Pcenter(0) >> _pcr.Pcenter(1) >> _pcr.Pcenter(2) >>
                      _pcr.R(0,0) >> _pcr.R(0,1) >> _pcr.R(0,2) >>
                      _pcr.R(1,0) >> _pcr.R(1,1) >> _pcr.R(1,2) >>
                      _pcr.R(2,0) >> _pcr.R(2,1) >> _pcr.R(2,2);

        std::cerr << _pcr.c << std::endl;
        std::cerr << _pcr.n.transpose() << std::endl;
        std::cerr << _pcr.u.transpose() << std::endl;
        std::cerr << _pcr.v.transpose() << std::endl;
        std::cerr << _pcr.Pcenter.transpose() << std::endl;
        std::cerr << _pcr.R << std::endl;
      }
      else if(_fittingMethod == SPHERE)
      {

        ROS_INFO("Use sphere fitting method");
        _inputFile >> _scr.center(0) >> _scr.center(1) >> _scr.center(2) >>
                      _scr.radius >>
                      _scr.phiMean >>
                      _scr.thetaMean >>
                      _scr.arcLengthX >>
                      _scr.arcLengthY >>
                      _scr.R(0,0) >> _scr.R(0,1) >> _scr.R(0,2) >>
                      _scr.R(1,0) >> _scr.R(1,1) >> _scr.R(1,2) >>
                      _scr.R(2,0) >> _scr.R(2,1) >> _scr.R(2,2);

        std::cerr << _scr.center.transpose() << std::endl;
        std::cerr << _scr.radius << std::endl;
        std::cerr << _scr.phiMean << std::endl;
        std::cerr << _scr.thetaMean << std::endl;
        std::cerr << _scr.arcLengthX << std::endl;
        std::cerr << _scr.arcLengthY << std::endl;
        std::cerr << _scr.R << std::endl;
      }
    }
  }
  else
  {
    ROS_ERROR("Execution mode not recognized");
    return false;
  }

  if (_n.ok()) 
  { 
    // Wait for poses being published
    ros::spinOnce();
    ROS_INFO("The kinect recording is ready.");
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
      else 
      {
        switch(_executionMode)
        {
          case CALIBRATION:
          {
            // Compute angles
            // computeAngles();

            // Add plane fitting data
            addSurfaceFittingData();

            // Log data
            logCalibrationData();

            break; 
          }
          case GAME:
          {
            // Compute chaser pose to be send to RVIZ
            computeChaserPose();

            // Publish data to topics
            publishData();

            break;
          }
          default:
          {
            break;
          }
        }
      }

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  publishData();
  ros::spinOnce();
  _loopRate.sleep();

  if(_executionMode == CALIBRATION)
  {
    // Close file of calibration data
    if(_outputFile.is_open())
    {
      _outputFile.close();
    }

    if(_calibrationData.size()> 0)
    {
      ROS_INFO("Fit surfaces !");

      // Compute plane and sphere fitting
      planeEigenSolverFitting();

      sphereLeastSquareFitting();

      // Log calibration result
      logCalibrationResult();
    }
    else
    {
      ROS_INFO("No calibration data saved !");
    }

  }
  else if(_executionMode == GAME)
  {
    if(_inputFile.is_open())
    {
      _inputFile.close();
    }
  }

  ros::shutdown();
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
      std::cerr << "Initialization count: " << _averageCount << std::endl;
    }

    if(_averageCount == 1)
    {
      ROS_INFO("Initialization starting ...");
    }
    else if(_averageCount == AVERAGE_COUNT)
    {
      Eigen::Vector3f e1 = _markersPosition0.col(PB)-_markersPosition0.col(PA); 
      e1.normalize();   
      Eigen::Vector3f e2 = _markersPosition0.col(PC)-_markersPosition0.col(PA);  
      e2.normalize();
      Eigen::Vector3f e3 = e1.cross(e2); 
      e3.normalize();
      _R.col(0) = e1;
      _R.col(1) = e2;
      _R.col(2) = e3;
      std::cerr << "World frame basis described in kinect frame" << std::endl;
      std::cerr << _R << std::endl;
      ROS_INFO("Initialization done !");
    }
  }
  else
  {
    _initializationOK = true;
  }
}


void KinectRecording::computeChaserPose()
{


  if(_markersSequenceID(TOE)!= _currentSequenceID)
  {

    float scaleX, scaleY; 
    Eigen::Vector3f u,v;
    _chaserPosition.setConstant(0.0f);
    
    if(_fittingMethod == PLANE)
    {
      u = _pcr.u.normalized();
      v = _pcr.v.normalized();
      scaleX = 10.0f/_pcr.u.norm();
      scaleY = 10.0f/_pcr.v.norm();

      Eigen::Vector3f temp;

      // Try filtering  
      SGF::Vec tempF(3);
      tempF = _markersPosition.col(TOE);
      _filter.AddData(_markersPosition.col(TOE));
      _filter.GetOutput(0,tempF);

      if(_useFiltering)
      {
        temp = tempF;
        temp = _pcr.R.transpose()*temp;
      }
      else
      {
        temp = _pcr.R.transpose()*_markersPosition.col(TOE);
      }
      
      //Override filtering
      // temp = _pcr.R.transpose()*_markersPosition.col(TOE);

      Eigen::Vector3f proj = temp-(temp.dot(_pcr.n)+_pcr.c)*_pcr.n/_pcr.n.squaredNorm();

      _chaserPosition(0) = scaleX*(proj-_pcr.Pcenter).dot(u);
      _chaserPosition(1) = scaleY*(proj-_pcr.Pcenter).dot(v);
    // if(!_first)
    }
    else if(_fittingMethod == SPHERE)
    {
      scaleX = 10.0f/_scr.arcLengthX;
      scaleY = 10.0f/_scr.arcLengthY;

      Eigen::Vector3f temp;

      // Try filtering  
      SGF::Vec tempF(3);
      tempF = _markersPosition.col(TOE);
      _filter.AddData(_markersPosition.col(TOE));
      _filter.GetOutput(0,tempF);

      if(_useFiltering)
      {
        temp = tempF;
        temp = _scr.R.transpose()*temp;
      }
      else
      {
        temp = _scr.R.transpose()*_markersPosition.col(TOE);
      }


      Eigen::Vector3f proj = _scr.center+_scr.radius*(temp-_scr.center).normalized();
      Eigen::Vector3f e = proj-_scr.center;
      float phi = std::atan2(e.segment(0,2).norm(),e(2));
      float theta = std::atan2(e(1),e(0));

      _chaserPosition(0) = -scaleX*_scr.radius*std::sin(phi)*(theta-_scr.thetaMean);
      _chaserPosition(1) = -scaleY*_scr.radius*(phi-_scr.phiMean);
    }

    _currentSequenceID = _markersSequenceID(TOE);
  }
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

  _outputFile.open(ros::package::getPath("hp_preliminary_experiment")+"/data/"+_subjectName+"_result_plane.txt");
  
  _outputFile << _pcr.c << std::endl
              << _pcr.n.transpose() << std::endl
              << _pcr.u.transpose() << std::endl
              << _pcr.v.transpose() << std::endl
              << _pcr.Pcenter.transpose() << std::endl
              << _pcr.R << std::endl;

  _outputFile.close();

  _outputFile.open(ros::package::getPath("hp_preliminary_experiment")+"/data/"+_subjectName+"_result_sphere.txt");

  _outputFile << _scr.center.transpose() << std::endl
                << _scr.radius << std::endl
                << _scr.phiMean << std::endl
                << _scr.thetaMean << std::endl
                << _scr.arcLengthX << std::endl
                << _scr.arcLengthY << std::endl
                << _scr.R << std::endl;

  _outputFile.close();
}


void KinectRecording::addSurfaceFittingData()
{
  if(_markersTracked(TOE))
  {
    if(_calibrationData.size()==0)
    {
      _calibrationData.push_back(_markersPosition.col(TOE));  
      _currentSequenceID = _markersSequenceID(TOE);
    }
    else
    {
      if(_markersSequenceID(TOE)!= _currentSequenceID)
      {
        _calibrationData.push_back(_markersPosition.col(TOE));
        _currentSequenceID = _markersSequenceID(TOE);
        std::cerr << "Number of calibration data: " << _calibrationData.size() << std::endl;
      }
    }
  }
}


void KinectRecording::planeLeastSquareFitting()
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
  std::cerr << "Number of points: " << _calibrationData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,3> A;
  A.resize(_calibrationData.size(),3);
  Eigen::Vector3f x;
  Eigen::VectorXf B;
  B.resize(_calibrationData.size());
  Eigen::Vector3f temp;
  for(uint32_t k = 0; k < _calibrationData.size(); k++)
  {
    temp = _calibrationData[k];
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

  _pcr.c = c;
  _pcr.n = n;
  _pcr.u  = P4-P1;
  _pcr.v  = P2-P1;
  _pcr.Pcenter = Pcenter;
  _pcr.R = _R;
}


void KinectRecording::planeEigenSolverFitting()
{
  // Plane equation: f(x,y,z) = u*x+v*y+w*z+c
  // Compute mean of data xmean
  // Compute covariance matrix of X-xmean
  // Compute eigen value decomposition
  // The eigenvalue with the shortest value is the one corresponding to the normal vector
  // c = -<n,xmean>

  std::cerr << "Start plane fitting ..." << std::endl;
  std::cerr << "Number of points: " << _calibrationData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,3> X;
  X.resize(_calibrationData.size(),3);
  
  for(uint32_t k = 0; k < _calibrationData.size(); k++)
  {
    X.row(k) << (_R.transpose()*_calibrationData[k]).transpose();
  }

  Eigen::Vector3f Xmean;
  Xmean(0) = X.col(0).array().mean();
  Xmean(1) = X.col(1).array().mean();
  Xmean(2) = X.col(2).array().mean();

  X.col(0) -= Xmean(0)*Eigen::VectorXf::Ones(_calibrationData.size());
  X.col(1) -= Xmean(1)*Eigen::VectorXf::Ones(_calibrationData.size());
  X.col(2) -= Xmean(2)*Eigen::VectorXf::Ones(_calibrationData.size());

  Eigen::Matrix3f C;
  C = X.transpose()*X/_calibrationData.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(C);

  if(solver.info() != Eigen::Success)
  {
    std::cerr << "Cannot find eigen decomposition !" << std::endl;
  }

  Eigen::Vector3f lambda = solver.eigenvalues();
  Eigen::Matrix3f V = solver.eigenvectors();

  std::cerr << "Eigenvalues: " << std::endl;
  std::cerr << lambda.transpose() << std::endl;
  std::cerr << "Eigenvectors: " << std::endl;
  std::cerr << V << std::endl;

  Eigen::Vector3f::Index index;
  lambda.array().abs().minCoeff(&index);
  
  Eigen::Vector3f n = V.col(index); // smallest eigenvalue
  float c = -n.dot(Xmean);

  // Find out if plaine is mainly vertical or horizontal
  n.array().abs().maxCoeff(&index);
  
  Eigen::Vector3f P1,P2,P3,P4;
  if(index==0)
  {
    std::cerr << "Normal vector mainly along x axis" << std::endl;
    float ymin = X.col(1).minCoeff()+Xmean(1);
    float ymax = X.col(1).maxCoeff()+Xmean(1);
    float zmin = X.col(2).minCoeff()+Xmean(2);
    float zmax = X.col(2).maxCoeff()+Xmean(2);
    P1 << (-n(1)*ymax-n(2)*zmin-c)/n(0), ymax, zmin;
    P2 << (-n(1)*ymin-n(2)*zmin-c)/n(0), ymin, zmin;
    P3 << (-n(1)*ymin-n(2)*zmax-c)/n(0), ymin, zmax;
    P4 << (-n(1)*ymax-n(2)*zmax-c)/n(0), ymax, zmax;

  }
  else if(index==1)
  {
    std::cerr << "Normal vector mainly along y axis" << std::endl;
    float xmin = X.col(0).minCoeff()+Xmean(0);
    float xmax = X.col(0).maxCoeff()+Xmean(0);
    float zmin = X.col(2).minCoeff()+Xmean(2);
    float zmax = X.col(2).maxCoeff()+Xmean(2);
    P1 << xmax, (-n(0)*xmax-n(2)*zmin-c)/n(1), zmin;
    P2 << xmin, (-n(0)*xmin-n(2)*zmin-c)/n(1), zmin;
    P3 << xmin, (-n(0)*xmin-n(2)*zmax-c)/n(1), zmax;
    P4 << xmax, (-n(0)*xmax-n(2)*zmax-c)/n(1), zmax;
  }
  else if(index == 2)
  {
    std::cerr << "Normal vector mainly along z axis" << std::endl;
    float xmin = X.col(0).minCoeff()+Xmean(0);
    float xmax = X.col(0).maxCoeff()+Xmean(0);
    float ymin = X.col(1).minCoeff()+Xmean(1);
    float ymax = X.col(1).maxCoeff()+Xmean(1);
    P1 << xmax, ymin, (-n(0)*xmax-n(1)*ymin-c)/n(2);
    P2 << xmin, ymin, (-n(0)*xmin-n(1)*ymin-c)/n(2);
    P3 << xmin, ymax, (-n(0)*xmin-n(1)*ymax-c)/n(2);
    P4 << xmax, ymax, (-n(0)*xmax-n(1)*ymax-c)/n(2);
  }
  else
  {
    std::cerr << "There is a problem" << std::endl;
  }

  std::cerr << "Plane corners:" << std::endl;
  std::cerr << "P1: " << P1.transpose() << std::endl;
  std::cerr << "P2: " << P2.transpose() << std::endl;
  std::cerr << "P3: " << P3.transpose() << std::endl;
  std::cerr << "P4: " << P4.transpose() << std::endl;

  Eigen::Vector3f Pcenter;
  
  Pcenter = (P1+P2+P3+P4)/4.0f;
  std::cerr << "Plane center: " << Pcenter.transpose() << std::endl;

  _pcr.c = c;
  _pcr.n = n;
  _pcr.u  = P1-P2;
  _pcr.v  = P3-P2;
  _pcr.Pcenter = Pcenter;
  _pcr.R = _R;


  std::cerr << "Plane fitting done !" << std::endl;

}


void KinectRecording::sphereLeastSquareFitting()
{
  // Sphere equation: (x-x0)^2+(y-y0)^2+(z-z0)^2 = R^2
  // A = [2x1 2y1 2z1 1
  //      2x2 2y2 2z2 1
  //      |  |  |     |
  //      2xn 2yn 2zn 1]

  // x = [x0 y0 z0  R^2-x0^2-y0^2-z0^2]'
  // B = [x1^2+y1^2+z1^2 x2^2+y2^2+z2^2 - xn^2+yn^2+zn^2]'
  // x = inv(A'A)*A'B

  std::cerr << "Start sphere fitting ..." << std::endl;
  std::cerr << "Number of points: " << _calibrationData.size() <<std::endl;

  Eigen::Matrix<float,Eigen::Dynamic,4> A;
  A.resize(_calibrationData.size(),4);
  Eigen::Vector4f x;
  Eigen::VectorXf B;
  B.resize(_calibrationData.size());
  for(uint32_t k = 0; k < _calibrationData.size(); k++)
  {
    _calibrationData[k] = _R.transpose()*_calibrationData[k];
    A.row(k) << 2.0f*_calibrationData[k](0), 2.0f*_calibrationData[k](1), 2.0f*_calibrationData[k](2), 1.0f;
    B(k) = _calibrationData[k].squaredNorm();
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
  phi.resize(_calibrationData.size());
  theta.resize(_calibrationData.size());

  for(int k = 0; k < _calibrationData.size(); k++)
  {
    Eigen::Vector3f proj;
    proj = center+radius*(_calibrationData[k]-center).normalized();
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
  float arcLengthX = (rho1*dtheta+rho2*dtheta)/2.0f;
  float arcLengthY = radius*dphi;

  std::cerr << "Center: " << std::endl;
  std::cerr << center.transpose() << std::endl;
  std::cerr << "Radius: " << radius << std::endl;
  std::cerr << "Phi mean: " << phiMean << " Theta mean: " << thetaMean << std::endl;
  std::cerr << "arcLengthX: " << arcLengthX << "arcLengthY: " << arcLengthY << std::endl;
  _scr.center = center;
  _scr.radius = radius;
  _scr.phiMean = phiMean;
  _scr.thetaMean = thetaMean;
  _scr.arcLengthX = arcLengthX;
  _scr.arcLengthY = arcLengthY;
  _scr.R = _R;


  std::cerr << "Sphere fitting done !" << std::endl;
}


void KinectRecording::updateMarkersPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  if(!_allMarkersReceived && msg->markers.size()>0)
  {
    ROS_INFO("Get first toe");
    _allMarkersReceived = true;
  }

  _markersTracked.setConstant(0);
  for(int k = 0; k < msg->markers.size(); k++) 
  {
    int m = -1;
    if(msg->markers[k].id == 1)
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

