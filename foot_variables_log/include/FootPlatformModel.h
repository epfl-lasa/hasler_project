#ifndef __FOOT_PLATFORM_MODELTROCAR_FEET_TELEMANIPULATION_H__
#define __FOOT_PLATFORM_MODELTROCAR_FEET_TELEMANIPULATION_H__

#include "Eigen/Eigen"


class FootPlatformModel 
{
	public:

	// Class constructor
	FootPlatformModel()
	{};

	static Eigen::Matrix4f forwardKinematics(Eigen::Matrix<float,5,1> joints, bool anglesInDegree = true);  

  static Eigen::Matrix<float,6,5> geometricJacobian(Eigen::Matrix<float,5,1> joints, bool anglesInDegree = true); 
};


#endif
