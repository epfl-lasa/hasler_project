#include "SurgicalTask.h"
#include "Utils.h"
#include <chrono>
#include <thread>
#include "qpSolver.h"

SurgicalTask* SurgicalTask::me = NULL;


SurgicalTask::SurgicalTask(ros::NodeHandle &n, double frequency, std::string fileName):
  _nh(n),
  _loopRate(frequency),
  _dt(1.0f/frequency),
  _fileName(fileName)
{
  
}


bool SurgicalTask::init() 
{
  // Read configuration parameters
  if(!readConfigurationParameters())
  {
    return false;
  }
  
  // Initialize task parameters
  initializeTaskParameters();

  // Initialize subscribers and publishers
  initializeSubscribersAndPublishers();


  // Create log file
  _outputFile.open(ros::package::getPath(std::string("surgical_task"))+"/data/"+_fileName+".txt");

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
    _desiredGripperPosition[r] = _gripperRange;
    for(int m = 0; m < 7; m++)
    {
      _ikJoints[r][m] = _currentJoints[r](m);
    }
    _FdFoot[r].setConstant(0.0f);
  }

  _msgGripperInput.ros_desAngle = 0.0f;

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
    
    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_useRobot[r] && r == _currentRobot)
      {
        robotControlStep(r,_dominantInputID);
      }
      else if(_useRobot[r] && r != _currentRobot)
      {
        // Update trocar information
        // updateRobotTaskState(r);
        if(_tool[r] == CAMERA && _useTaskAdaptation)
        {
          robotControlStep(r,_nonDominantInputID);
        }
        else
        {
          updateRobotTaskState(r);
        }
      }
    }
  }
  else if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
  {
    for(int r = 0; r < NB_ROBOTS; r++)
    {
      if(_useRobot[r])
      {
        robotControlStep(r, _humanInputID[r]);
      }
    }
  }
}


void SurgicalTask::logData()
{
 _outputFile << ros::Time::now() << " "
             << _currentJoints[LEFT].transpose() << " "
             << _currentJointVelocities[LEFT].transpose() << " "
             << _currentJointTorques[LEFT].transpose() << " "
             << _Fext[LEFT].transpose() << " "
             << _ikJoints[LEFT].transpose() << " "
             << _trocarPosition[LEFT].transpose() << " "
             << _depthGain[LEFT] << " " 
             << _vdTool[LEFT].transpose() << " "
             << _xd[LEFT].transpose() << " "
             << _selfRotationCommand[LEFT] << " "
             << _trocarInput[LEFT].transpose() << " " 
             << _footPose[LEFT].transpose() << " "
             << _footTwist[LEFT].transpose() << " "
             << _footWrenchM[LEFT].transpose() << " "
             << _footWrenchRef[LEFT].transpose() << " "
             << _footWrenchD[LEFT].transpose() << " "
             << (int) _footState[LEFT] << " "
             << _FdFoot[LEFT].transpose() << " "
             << _taud[LEFT] << " "
             << _desiredFootWrench[LEFT].transpose() << " "
             << _vH[LEFT].transpose() << " "
             << _tankH[LEFT] << " "
             << _alphaH[LEFT] << " "
             << (int) _controlPhase[LEFT] << " " 
             << (int) _qpResult[LEFT].eeCollisionConstraintActive << " "
             << (int) _qpResult[LEFT].toolCollisionConstraintActive << " "
             << (int) _qpResult[LEFT].workspaceCollisionConstraintActive << " "

             << _currentJoints[RIGHT].transpose() << " "
             << _currentJointVelocities[RIGHT].transpose() << " "
             << _currentJointTorques[RIGHT].transpose() << " "
             << _Fext[RIGHT].transpose() << " "
             << _ikJoints[RIGHT].transpose() << " "
             << _trocarPosition[RIGHT].transpose() << " "
             << _depthGain[RIGHT] << " " 
             << _vdTool[RIGHT].transpose() << " "
             << _xd[RIGHT].transpose() << " "
             << _selfRotationCommand[RIGHT] << " "
             << _trocarInput[RIGHT].transpose() << " " 
             << _footPose[RIGHT].transpose() << " "
             << _footTwist[RIGHT].transpose() << " "
             << _footWrenchM[RIGHT].transpose() << " "
             << _footWrenchRef[RIGHT].transpose() << " "
             << _footWrenchD[RIGHT].transpose() << " "
             << (int) _footState[RIGHT] << " "
             << _FdFoot[RIGHT].transpose() << " "
             << _taud[RIGHT] << " "
             << _desiredFootWrench[RIGHT].transpose() << " "
             << _vH[RIGHT].transpose() << " "
             << _tankH[RIGHT] << " "
             << _alphaH[RIGHT] << " "
             << (int) _controlPhase[RIGHT] << " " 
             << (int) _qpResult[RIGHT].eeCollisionConstraintActive << " "
             << (int) _qpResult[RIGHT].toolCollisionConstraintActive << " "
             << (int) _qpResult[RIGHT].workspaceCollisionConstraintActive << " "

             << (int) _allowTaskAdaptation << " "
             << (int) _useTaskAdaptation << " "
             << _vda.transpose() << " "
             << _beliefsC.transpose() << " "
             << _colorMarkersFilteredPosition.row(0) << " "
             << _colorMarkersFilteredPosition.row(1) << " "
             << _colorMarkersFilteredPosition.row(2) << " "
             << _colorMarkersStatus.transpose() << " "
        
             << _msgGripperOutput.gripper_position << " "
             << _msgGripperOutput.gripper_speed << " "
             << _msgGripperOutput.gripper_desPosition << " "
             << _msgGripperOutput.gripper_torqueCmd << " "
             << _msgGripperOutput.gripper_torqueMeas << " "
             << (int) _msgGripperOutput.gripper_machineState << " "

             << (int) _humanInputMode << " "
             << (int) _currentRobot << " "
             << (int) _clutching << " "
             << (int) _wait << " "

             << std::endl;
}



