#include "SurgicalTask.h"
#include "Utils.h"
#include <chrono>
#include <thread>
#include "qpSolver.h"

SurgicalTask* SurgicalTask::me = NULL;


SurgicalTask::SurgicalTask(ros::NodeHandle &n, double frequency):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{
  
}


bool SurgicalTask::init() 
{
  // Read configuration parameters
  if(!readConfigurationParameters())
  {
    return false;
  }

  // Initialize subscribers and publishers
  initializeSubscribersAndPublishers();

  // Initialize task parameters
  initializeTaskParameters();

  // Create log file
  _outputFile.open(ros::package::getPath(std::string("robotic_experiments"))+"/data_foot/surgical_task.txt");

  // Create callback to kill node via CTRL+C
  signal(SIGINT,SurgicalTask::stopNode);

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
    if(_allSubscribersOK && _allFramesOK && _trocarsRegistered[LEFT] && _trocarsRegistered[RIGHT])
    {

      static bool everythingOK = false;
      if(!everythingOK)
      {
        everythingOK = true;
        std::cerr << "[SurgicalTask]: Starting the task !!!" << std::endl;

      }

      // Check for update of the DS-impedance controller gain
      if(!_useFranka && _useRobot[LEFT])
      {
        ros::param::getCached("/left_lwr/joint_controllers/ds_param/damping_eigval0",_d1[LEFT]);
      }
      if(!_useFranka && _useRobot[RIGHT])
      {
        ros::param::getCached("/right_lwr/joint_controllers/ds_param/damping_eigval0",_d1[RIGHT]);
      }
          
      // Compute control command
      step();

      // Publish data to topics
      publishData();

      // Log data
      // logData();
    }
    else
    {
      if(!_allFramesOK)
      {
        // Receive frames
        if(_useSim)
        {
          receiveFrames();
        }

        // Check all frames are received
        checkAllFrames();
      }

      if(!_allSubscribersOK)
      {
        checkAllSubscribers();
      }          

      if(!_usePredefinedTrocars)
      {
        registerTrocars();
      }
    }

    ros::spinOnce();
    _loopRate.sleep();
  }

  // Safely stop the robots
  for(int r = 0; r < NB_ROBOTS; r++)
  {
    _vd[r].setConstant(0.0f);
    _omegad[r].setConstant(0.0f);
    _qd[r] = _q[r];  
    _desiredFootWrench[r].setConstant(0.0f);  
    _stiffness[r].setConstant(0.0f);
    _desiredGripperPosition[r] = 0.0f;
    for(int m = 0; m < 7; m++)
    {
      _ikJoints[r][m] = _currentJoints[r](m);
    }
  }

  _msgGripperInput.ros_dPosition = 0.0f;

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


void SurgicalTask::step()
{
  // Process human input
  processHumanInput();

  // Perform control step
  if(_humanInputMode == DOMINANT_INPUT_TWO_ROBOTS)
  {
    if(_debug)
    {
      std::cerr << "[SurgicalTask]: DOMINANT INPUT: " << (int) _currentRobot << std::endl;
    }
    
    if(_useRobot[_currentRobot])
    {
      robotControlStep(_currentRobot,_dominantInputID);
    }
  }
  else if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
  {
    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_useRobot[r])
      {
        robotControlStep(r, r);
      }
    }
  }
}


void SurgicalTask::logData()
{
 _outputFile << ros::Time::now() << " "
             << _markersPosition.col(LEFT_HUMAN_TOOL).transpose() << " "  
             << _markersQuaternion.col(LEFT_HUMAN_TOOL).transpose() << " "
             << _offsetTool.transpose() << std::endl;
}



