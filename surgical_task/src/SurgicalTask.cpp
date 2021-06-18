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
  while(!_stop && !_taskFinished) 
  {
    if(_allSubscribersOK && _allFramesOK && _trocarsRegistered[LEFT] && _trocarsRegistered[RIGHT])
    {

      static bool everythingOK = false;
      if(!everythingOK)
      {
        if(_logData)
        {
          std::string folderPath;
          folderPath = ros::package::getPath(std::string("surgical_task"))+"/data/"+_fileName+"/";
          if(mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
          {
              if( errno == EEXIST ){
                 // alredy exists
              } 
              else 
              {
                // something else
                std::cout << "[SurgicalTask]: Cannot create folder:" << strerror(errno) << std::endl;
                throw std::runtime_error( strerror(errno) );
              }
          }

          // Create log file
          _outputFile[0].open(folderPath+_fileName+"_"+std::to_string(_taskId)+"_"+std::to_string(_taskCondition)+"_"+std::to_string(_repetitionId)+"_surgical_task_left_robot.txt");
          _outputFile[1].open(folderPath+_fileName+"_"+std::to_string(_taskId)+"_"+std::to_string(_taskCondition)+"_"+std::to_string(_repetitionId)+"_surgical_task_right_robot.txt");
          _outputFile[2].open(folderPath+_fileName+"_"+std::to_string(_taskId)+"_"+std::to_string(_taskCondition)+"_"+std::to_string(_repetitionId)+"_surgical_task_left_foot.txt");
          _outputFile[3].open(folderPath+_fileName+"_"+std::to_string(_taskId)+"_"+std::to_string(_taskCondition)+"_"+std::to_string(_repetitionId)+"_surgical_task_right_foot.txt");
          _outputFile[4].open(folderPath+_fileName+"_"+std::to_string(_taskId)+"_"+std::to_string(_taskCondition)+"_"+std::to_string(_repetitionId)+"_surgical_task_state.txt");          
        }
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
      if(_logData)
      {
        logData();
      }
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


  for(int k = 0; k < 3; k++)
  {
    _outputFile[k].close();
  }

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
  for(int r = 0; r < NB_ROBOTS; r++)
  {
    if(_useRobot[r])
    {
     _outputFile[r] << ros::Time::now() << " "
                    << _currentJoints[r].transpose() << " "
                    << _currentJointVelocities[r].transpose() << " "
                    << _currentJointTorques[r].transpose() << " "
                    << _toolOffsetFromEE[r].transpose() << " "
                    << _Fext[r].transpose() << " "
                    << _wrench[r].transpose() << " "
                    << _Fh[r].transpose() << " "
                    << _ikJoints[r].transpose() << " "
                    << _trocarPosition[r].transpose() << " "
                    << _depthGain[r] << " " 
                    << _vdTool[r].transpose() << " "
                    << _vtRef[r].transpose() << " "
                    << _vtd[r].transpose() << " "
                    << _xd[r].transpose() << " "
                    << _selfRotationCommand[r] << " "
                    << _trocarInput[r].transpose() << " " 
                    << _FdFoot[r].transpose() << " "
                    << _FmFoot[r].transpose() << " "
                    << _taud[r] << " "
                    << _desiredFootWrench[r].transpose() << " "
                    << _vHRef[r].transpose() << " "
                    << _vHd[r].transpose() << " "
                    << _tankH[r] << " "
                    << _alphaH[r] << " "
                    << (int) _controlPhase[r] << " " 
                    << (int) _tool[r] << " "
                    << (int) _humanInputID[r] << " "
                    << (int) _qpResult[r].eeCollisionConstraintActive << " "
                    << (int) _qpResult[r].toolCollisionConstraintActive << " "
                    << (int) _qpResult[r].workspaceCollisionConstraintActive << " "
                    << _dEECollision[r] << " "
                    << _dToolCollision[r] << " " << std::endl;
        
      int h;
      if(_humanInputMode == SINGLE_FOOT_SINGLE_ROBOT)
      {
        h = _humanInputID[r];
      }
      else
      {
        h = r;
      }

      _outputFile[2+h] << ros::Time::now() << " "
                       << _footPose[h].transpose() << " "
                       << _footTwist[h].transpose() << " "
                       << _footWrenchM[h].transpose() << " "
                       << _footWrenchRef[h].transpose() << " "
                       << _footWrenchD[h].transpose() << " "
                       << (int) _footState[h] << " "
                       << _footInputPosition[h].transpose() << " "
                       << _footInputFilterAxisForce[h].transpose() << " "
                       << _footInputKp[h].transpose() << " "
                       << _footInputKd[h].transpose() << " "
                       << _legJointPositions[h].transpose() << " "
                       << _legJointVelocities[h].transpose() << " "
                       << _legJointTorques[h].transpose() << " "
                       << _legFootBaseWrench[h].transpose() << " "
                       << _footHapticEfforts[h].transpose() << " "
                       << _footInertiaCoriolisCompensationTorques[h].transpose() << " "
                       << _legCompensationTorques[h].transpose() << " "
                       << _footWrenchModified[h].transpose() << " "
                       << _fh_haptEffLegIn[h].transpose() << " "
                       << _fh_jointLimCoeffs[h].transpose() << " "
                       << _fh_bckgndEffLeg[h].transpose() << " "
                       << _fh_haptEffLPF[h].transpose() << " "
                       << _fh_haptEffLPF_Proj[h].transpose() << " "
                       << _fh_haptEffHPF[h].transpose() << " "
                       << _fh_effFootOut[h].transpose() << " "
                       << _fh_vibFB[h].transpose() << " "
                       << _fh_maxPossGains[h].transpose() << " "
                       << _fh_effGainRaw[h] << std::endl;        
    }
  }

  _outputFile[4] << ros::Time::now() << " "
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
   << _gripperFeedback << " "
   << (int)_graspAssistanceOn << " "
   << (int) _msgGripperOutput.gripper_machineState << " "
   << (int) _humanInputMode << " "
   << (int) _currentRobot << " "
   << (int) _dominantInputID << " "
   << (int) _nonDominantInputID << " "
   << (int) _clutching <<  " "
   << (int) _wait << " " 
   << (int) _taskStarted << " " 
   << (int) _stopTime << " "
   << _imageId << std::endl;
}



