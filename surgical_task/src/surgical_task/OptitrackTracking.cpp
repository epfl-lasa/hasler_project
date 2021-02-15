#include "SurgicalTask.h"


void SurgicalTask::updateHumanToolPosition()
{
  Eigen::Vector3f toolBasePosition[2];

  // Get tool position on robot basis
  toolBasePosition[LEFT] = _markersPosition.col(LEFT_HUMAN_TOOL)-_markersPosition.col(RIGHT_ROBOT_BASIS);
  toolBasePosition[RIGHT] =  _markersPosition.col(RIGHT_HUMAN_TOOL)-_markersPosition.col(RIGHT_ROBOT_BASIS);
  Eigen::Vector3f offsetWL, offsetWR, offsetOL, offsetOR;


  // Initialize correction offset
  offsetWL.setConstant(0.0f);
  offsetWR.setConstant(0.0f);
  offsetOL.setConstant(0.0f);
  offsetOR.setConstant(0.0f);

  // VALID ONLY WHEN EE TIP matches TOOL TIP
  offsetWL = (_x[LEFT]-toolBasePosition[LEFT]);
  offsetWL(2) -= 0.0025f;
  offsetWR << (_x[LEFT]-toolBasePosition[RIGHT]);
  offsetWR(2) -= 0.0025f;

  offsetOL = (_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(LEFT_HUMAN_TOOL))).inverse()*offsetWL;
  offsetOR = (_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(RIGHT_HUMAN_TOOL))).inverse()*offsetWR;
  _offsetTool = offsetOL;


  // Compute learned correction offset
  Eigen::MatrixXf temp;
  temp.resize(50,1);

  temp = _p[0]*_markersQuaternion.col(LEFT_HUMAN_TOOL)+_p[1];
  for(int k =0; k < temp.rows(); k++)
  {
    if(temp(k)<0.0f)
    {
      temp(k) = 0.0f;
    }
  }
  offsetOL = _p[2]*temp+_p[3];
  // offsetOL.setConstant(0.0f);

  std::cerr << "OL: " << offsetOL.transpose() << std::endl;
  std::cerr << "OR: " << offsetOR.transpose() << std::endl;
  // offsetOL << 0.0124496,   0.025967, -0.0133567;
  offsetOR << -0.0014803, 0.00447115, 0.00305053;

  // Apply correction
  _humanToolPosition[LEFT] = toolBasePosition[LEFT]+_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(LEFT_HUMAN_TOOL))*offsetOL;
  _humanToolPosition[RIGHT] = toolBasePosition[RIGHT]+_Roptitrack*Utils<float>::quaternionToRotationMatrix(_markersQuaternion.col(RIGHT_HUMAN_TOOL))*offsetOR;

  std::cerr << "offset L:" << (_x[LEFT]-toolBasePosition[LEFT]).transpose() << std::endl;
  std::cerr << "offset R:" << (_x[LEFT]-toolBasePosition[RIGHT]).transpose() << std::endl;
}


uint16_t SurgicalTask::checkTrackedMarker(float a, float b)
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


void SurgicalTask::optitrackInitialization()
{
  if(_optitrackCount< NB_OPTITRACK_SAMPLES)
  {
    if(_markersTracked(RIGHT_ROBOT_BASIS) && _markersTracked(LEFT_ROBOT_BASIS))
    {
      _markersPosition0 = (_optitrackCount*_markersPosition0+_markersPosition)/(_optitrackCount+1);
      _optitrackCount++;
    }
    std::cerr << "[ObjectGrasping]: Optitrack Initialization count: " << _optitrackCount << std::endl;
    if(_optitrackCount == 1)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization starting ...");
    }
    else if(_optitrackCount == NB_OPTITRACK_SAMPLES)
    {
      ROS_INFO("[ObjectGrasping]: Optitrack Initialization done !");
      _xRobotBaseOrigin[LEFT] = _markersPosition0.col(LEFT_ROBOT_BASIS)-_markersPosition0.col(RIGHT_ROBOT_BASIS);
    }
  }
  else
  {
    _optitrackInitialized = true;
  }
}