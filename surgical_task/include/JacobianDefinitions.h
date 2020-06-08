#ifndef __JACOBIAN_DEFINITIONS_H__
#define __JACOBIAN_DEFINITIONS_H__

#include "Eigen/Core"


Eigen::Matrix<float,3,7> jacobianTool(Eigen::Matrix<float,7,1> joints, float toolOffset);

Eigen::Matrix<float,3,7> jacobianZ(Eigen::Matrix<float,7,1> joints);

Eigen::Matrix<float,3,7> jacobianRCM(Eigen::Matrix<float,7,1> joints, Eigen::Vector3f xTrocar);


#endif