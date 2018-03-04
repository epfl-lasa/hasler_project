#include "foot_control_xy/Protocol.h"


Protocol* Protocol::me = NULL;

Protocol::Protocol(ros::NodeHandle &n, double frequency, Eigen::Vector3f initTargetPosition, Strategy strategy, std::string subjectName): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_targetPosition(initTargetPosition),
_strategy(strategy),
_subjectName(subjectName)
{
  me=this;

  _chaserPosition.setConstant(0.0f);
  _previousChaserPosition.setConstant(0.0f);
  _initialChaserPosition.setConstant(0.0f);

  _targetDirectionID = PLUS_X;
  _dsResults.resize(0);
  _trackingError.resize(0);
  _trajectoryLength = 0.0f;

  _stop = false;
  _firstChaserPoseReceived = false;
  _chaserReady = false;
  _targetReached = false;
  _setTargetToHome = false;
  _firstTarget = true;
  _timeout = false;

  _movingTargetBoundary = 0.0f;

  _sequenceID = 0;
  _previousSequenceID = 0;

}

bool Protocol::init()
{
  
  //Subscriber definitions
  _subChaserPose = _n.subscribe("/chaser/pose",1,&Protocol::updateChaserPose,this,ros::TransportHints().reliable().tcpNoDelay());
  
  //Publisher definitions
  _pubTargetPose = _n.advertise<geometry_msgs::PoseStamped>("target/pose", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&Protocol::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,Protocol::stopNode);

  if(_strategy==DISCRETE)
  {
    _outputFile.open("src/hasler_project/foot_control_xy/"+_subjectName+"_discrete_strategy_data.txt");
  }
  else if(_strategy == CONTINUOUS)
  {
    _outputFile.open("src/hasler_project/foot_control_xy/"+_subjectName+"_continuous_strategy_data.txt");
  }

  if (_n.ok()) 
  { 
    ros::spinOnce();
    ROS_INFO("The Protocol node is about to start");
    return true;
  }
  else 
  {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}


void Protocol::stopNode(int sig)
{
  me->_stop= true;
}


void Protocol::run()
{
  srand(time(NULL));

  _keepingDirectionTime = _minKeepingDirectionTime+(_maxKeepingDirectionTime-_minKeepingDirectionTime)*((float)std::rand()/RAND_MAX);


  while (!_stop && !_timeout) 
  {
    _currentTime = ros::Time::now().toSec();

    if(_firstChaserPoseReceived)
    {
      _mutex.lock();

      // Wait for the chaser to reach the first target before starting the strategies
      if(isChaserReady())
      {
        switch(_strategy)
        {
          case DISCRETE:
          {
            // Check if target reached
            checkIfTargetReached();

            // Update target pose
            updateTargetPose();

            break;
          }
          case CONTINUOUS:
          {
            // Generate moving target
            generateMovingTarget();

            break;
          }
        }

        // Log raw data to text file
        logData(); 

        // Check timeout
        checkTimeout(); 
      }

      // Publish data
      publishData();
      
      _mutex.unlock();
    }


    ros::spinOnce();
    _loopRate.sleep();
  }

  if(_outputFile.is_open())
  {
    _outputFile.close();
  }

  // Log strategies result to text file
  logResult();
  
  ros::shutdown();
}


bool Protocol::isChaserReady()
{
  float error = (_targetPosition-_chaserPosition).segment(0,2).norm();
  if(error<_targetToleranceRadius && !_chaserReady)
  {
    _chaserReady = true;
    _targetReached = true;
    _startingTime = ros::Time::now().toSec();
    _initialTime = _startingTime;
    _reachedTime = _startingTime;

  }

  return _chaserReady;
}


void Protocol::checkIfTargetReached()
{
  float error = (_targetPosition-_chaserPosition).segment(0,2).norm();

  if(error<_targetToleranceRadius && !_targetReached)
  {
    _targetReached = true;
    _reachedTime = ros::Time::now().toSec();
  }
}


void Protocol::updateTargetPose()
{
  // Update trajectory length when new data 
  if(_sequenceID != _previousSequenceID)
  {
    _trajectoryLength += (_chaserPosition-_previousChaserPosition).segment(0,2).norm();
    _previousSequenceID = _sequenceID;
    _previousChaserPosition = _chaserPosition;
  }

  // Update target if:
  // The exceeded time exceeded the authorized one
  // OR The target is reached and the tolerance time has passed
  if((!_targetReached && _currentTime-_initialTime > _targetElapsedTime) || (_targetReached && _currentTime-_reachedTime > _targetToleranceTime))
  {
    // Fill result structure
    if(_firstTarget)
    {
      _firstTarget = false;
    }
    else
    {
      DiscreteStrategyResult result;
      result.targetPosition = _targetPosition;
      result.targetReached = (int) _targetReached;
      if(_targetReached)
      {
        result.elapsedTime = float(_reachedTime-_initialTime);
        std::cerr << "Target reached" << std::endl;
      }
      else
      {
        result.elapsedTime = _targetElapsedTime;
        std::cerr << "Target missed" << std::endl;
      }
      result.accuracy = (_chaserPosition-_targetPosition).norm();
      result.normalizedTrajectoryLength = _trajectoryLength/((_targetPosition-_initialChaserPosition).norm());
      _dsResults.push_back(result);
    }

    // Manage switching between home and random target
    _setTargetToHome = !_setTargetToHome;

    // Update target position
    if(_setTargetToHome)
    {
      _targetPosition.setConstant(0.0f);
    }
    else
    {
      _targetPosition(0) = (_minTargetDistance+(_maxTargetDistance-_minTargetDistance)*((float)std::rand()/RAND_MAX))*(-1.0f+2.0f*float(std::rand()%2));
      _targetPosition(1) = (_minTargetDistance+(_maxTargetDistance-_minTargetDistance)*((float)std::rand()/RAND_MAX))*(-1.0f+2.0f*float(std::rand()%2));
      _targetPosition(2) = 0.0f; 
    }

    _initialTime = ros::Time::now().toSec();
    _initialChaserPosition = _chaserPosition;
    _trajectoryLength = 0.0f;
    _targetReached = false;
  }
}


void Protocol::generateMovingTarget()
{

  // Check to update moving target boundary
  if(fabs(_movingTargetBoundary-_config.movingTargetBoundary)>FLT_EPSILON)
  {
    if(_targetPosition.segment(0,2).array().abs().maxCoeff() < _config.movingTargetBoundary)
    {
      _movingTargetBoundary = _config.movingTargetBoundary;
    }
  }

  // Update trajectory length when new data 
  if(_sequenceID != _previousSequenceID)
  {
    _trackingError.push_back((_targetPosition-_chaserPosition).segment(0,2).norm());
    _previousSequenceID = _sequenceID;
  }

  int newTargetDirectionID = _targetDirectionID;

  // Change direction if time elapsed exceeded the duration of the current target or if there is collision with boundaries
  if(_currentTime-_initialTime>_keepingDirectionTime || checkIfCollisionWithBoundaries(_targetPosition,newTargetDirectionID))
  {
    newTargetDirectionID = (int)(4.0f*(float)std::rand()/RAND_MAX);
    while(checkIfCollisionWithBoundaries(_targetPosition,newTargetDirectionID) ||
          checkIfOppositeDirection(_targetDirectionID,newTargetDirectionID) || 
          checkIfSameDirection(_targetDirectionID,newTargetDirectionID))
    {
      newTargetDirectionID = (int)(4.0f*(float)std::rand()/RAND_MAX);
    }
    _keepingDirectionTime = _minKeepingDirectionTime+(_maxKeepingDirectionTime-_minKeepingDirectionTime)*((float)std::rand()/RAND_MAX);
    _initialTime = ros::Time::now().toSec();
    std::cerr << "Duration: " << _keepingDirectionTime << " Direction: " << newTargetDirectionID << std::endl;
  }  

  // Update target position
  _targetDirectionID = DirectionID(newTargetDirectionID); 
  _targetPosition += _dt*_movingTargetVelocity*getTargetDirection(_targetDirectionID);
}


Eigen::Vector3f Protocol::getTargetDirection(int directionID)
{
  Eigen::Vector3f dir;
  switch(directionID)
  {
    case PLUS_X:
    {
      dir << 1.0f,0.0f,0.0f;
      break;
    }
    case MINUS_X:
    {
      dir << -1.0f,0.0f,0.0f;
      break;
    }
    case PLUS_Y:
    {
      dir << 0.0f,1.0f,0.0f;
      break;
    }
    case MINUS_Y:
    {
      dir << 0.0f,-1.0f,0.0f;
      break;
    }
  }

  return dir;
}


bool Protocol::checkIfCollisionWithBoundaries(Eigen::Vector3f position, int directionID)
{
  Eigen::Vector3f temp;
  temp = position +_dt*_movingTargetVelocity*getTargetDirection(DirectionID(directionID));
  if(temp.array().abs().maxCoeff()>_movingTargetBoundary)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool Protocol::checkIfSameDirection(int currentDirectionID, int newDirectionID)
{
  if(currentDirectionID == newDirectionID)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool Protocol::checkIfOppositeDirection(int currentDirectionID, int newDirectionID)
{
  Eigen::Vector3f v1 = getTargetDirection(currentDirectionID);
  Eigen::Vector3f v2 = getTargetDirection(newDirectionID);

  if((v1.dot(v2)+1.0f) < FLT_EPSILON)
  {
    return true;
  }
  else
  {
    return false;
  }
}


void Protocol::logData()
{
  if(_strategy == DISCRETE)
  {
    _outputFile << ros::Time::now() << " "
                << _chaserPosition.transpose() << " "
                << _targetPosition.transpose() << " "
                << _dsResults.size() << " "
                << (int) _targetReached << " "
                << _sequenceID << std::endl;
  }
  else if(_strategy == CONTINUOUS)
  {
    _outputFile << ros::Time::now() << " "
                << _chaserPosition.transpose() << " "
                << _targetPosition.transpose() << " "
                << _sequenceID << std::endl;
  }
}


void Protocol::checkTimeout()
{
  if(_strategy == DISCRETE)
  {
    if(float(_currentTime-_startingTime) > _discreteStrategyDuration)
    {
      _timeout = true;
    }
  }
  else if(_strategy == CONTINUOUS)
  {
    if(float(_currentTime-_startingTime) > _continuousStrategyDuration)
    {
      _timeout = true;
    }
  }
}

void Protocol::logResult()
{
  if(_strategy == DISCRETE)
  {
    _outputFile.open("src/hasler_project/foot_control_xy/"+_subjectName+"_discrete_strategy_result.txt");
    for(int k = 0; k < _dsResults.size(); k++)
    {
      _outputFile << _dsResults[k].targetPosition.transpose() << " "
                  << (int) _dsResults[k].targetReached << " "
                  << _dsResults[k].elapsedTime << " "
                  << _dsResults[k].accuracy << " "
                  << _dsResults[k].normalizedTrajectoryLength << " " << std::endl;
    }
    _outputFile.close();
  }
  else if(_strategy == CONTINUOUS)
  {
    _outputFile.open("src/hasler_project/foot_control_xy/"+_subjectName+"_continuous_strategy_result.txt");

    if(_trackingError.size()>0)
    {
      Eigen::VectorXf temp;
      temp.resize(_trackingError.size());
      memcpy(temp.data(),_trackingError.data(),_trackingError.size());
      _outputFile << std::sqrt(temp.cwiseAbs2().mean()) << std::endl;
    }

    _outputFile.close();
  }
}


void Protocol::publishData()
{
  _msgTargetPose.header.frame_id = "world";
  _msgTargetPose.header.stamp = ros::Time::now();
  _msgTargetPose.pose.position.x = _targetPosition(0);
  _msgTargetPose.pose.position.y = _targetPosition(1);
  _msgTargetPose.pose.position.z = _targetPosition(2);
  _msgTargetPose.pose.orientation.x = 0.0f;
  _msgTargetPose.pose.orientation.y = 0.0f;
  _msgTargetPose.pose.orientation.z = 0.0f;
  _msgTargetPose.pose.orientation.w = 1.0f;
  _pubTargetPose.publish(_msgTargetPose);
}


void Protocol::updateChaserPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  _chaserPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  _sequenceID = msg->header.seq;

  if(!_firstChaserPoseReceived)
  {
    _firstChaserPoseReceived = true;
    _initialTime = ros::Time::now().toSec();
    _initialChaserPosition = _chaserPosition;
    _previousChaserPosition = _chaserPosition;
  }
}


void Protocol::dynamicReconfigureCallback(foot_control_xy::protocol_paramsConfig &config, uint32_t level)
{

  static bool first = false;

  ROS_INFO("Reconfigure request bou. Updatig the parameters ...");
  
  _config = config;

  _discreteStrategyDuration = config.discreteStrategyDuration;
  _targetToleranceRadius = config.targetToleranceRadius;
  _minTargetDistance = config.minTargetDistance;
  _maxTargetDistance = config.maxTargetDistance;
  _targetElapsedTime = config.targetElapsedTime;
  _targetToleranceTime = config.targetToleranceTime;
  _continuousStrategyDuration = config.continuousStrategyDuration;
  if(!first)
  {
    _movingTargetBoundary = config.movingTargetBoundary;
    first = true;
  }
  _movingTargetVelocity = config.movingTargetVelocity;
  _minKeepingDirectionTime = config.minKeepingDirectionTime;
  _maxKeepingDirectionTime = config.maxKeepingDirectionTime;
}