#include "FootPlatformModel.h"

const float a2i = 0.305f;
const float d5 = 0.465f - a2i - 0.025f; //! Distance between the pitch/roll/yaw RCM to the pedal
const float a6 = 0.0f;

Eigen::Matrix4f FootPlatformModel::forwardKinematics(Eigen::Matrix<float,5,1> joints, bool anglesInDegree)
{
  float d1 = joints(1);
  float d2 = joints(0);
  float theta,phi,psi; //!pitch, roll, yaw
  if(anglesInDegree)
  {
    theta = -joints(2)*M_PI/180.0f;
    phi = joints(3)*M_PI/180.0f;
    psi = joints(4)*M_PI/180.0f-M_PI_2;
  }
  else
  {
    theta = -joints(2);
    phi = joints(3);
    psi = joints(4) - M_PI_2;
  }
  Eigen::Matrix4f H;
  H.setConstant(0.0f);

  float c_theta = cos(theta);
  float c_phi = cos(phi);
  float c_psi = cos(psi);
  float s_theta = sin(theta);
  float s_phi = sin(phi);
  float s_psi = sin(psi);

  float s1 = s_phi*s_theta; //! 
  float s2 = c_phi;

  H(0,0) = - 1.0*s_psi*s2;
  H(1,0) = c_psi*c_theta+s_psi*s1;
  H(2,0) = c_theta*s_phi*s_psi-1*c_psi*s_theta;
  H(3,0) = 0.0;
  H(0,1) = -1*c_psi*s2;
  H(1,1) = c_psi*s1-1.0*c_theta*s_psi;
  H(2,1) = s_psi*s_theta+c_psi*c_theta*s_phi;
  H(3,1) = 0.0;
  H(0,2) = s_phi;
  H(1,2) = c_phi*s_theta;
  H(2,2) = c_phi*c_theta;
  H(3,2) = 0.0;
  H(0, 3) = d2 + d5 * s1 + a6 * H(0, 0);
  H(1, 3) = d1 - 1.0 * d5 * (- 1 * c_phi * s_theta) + a6 * H(1, 0) ;
  H(2,3) = d5*c_phi*c_theta - 1.0*a6*(c_psi*s_theta-1.0*c_theta*s_phi*s_psi)+a2i;
  H(3,3) = 1.0;

  return H;
}  


Eigen::Matrix<float,6,5> FootPlatformModel::geometricJacobian(Eigen::Matrix<float,5,1> joints, bool anglesInDegree)
{

  float d1 = joints(1);
  float d2 = joints(0);
  float theta,phi,psi;
  
  if(anglesInDegree)
  {
    theta = -joints(2)*M_PI/180.0f;
    phi = joints(3)*M_PI/180.0f;
    psi = joints(4)*M_PI/180.0f;
  }
  else
  {
    theta = -joints(2);
    phi = joints(3);
    psi = joints(4);    
  }
  Eigen::Matrix<float,6,5> J;
  J.setConstant(0.0f);
  float c_theta = cos(theta);
  float c_phi = cos(phi);
  float c_psi = cos(psi);
  float s_theta = sin(theta);
  float s_phi = sin(phi);
  float s_psi = sin(psi);

  float s1=d5*s_phi;
  float s2 = s_phi;
  float s5 = d5*c_phi*s_theta;
  float s3 = -s5;
  float s4 = -1.0*c_phi*s_theta;

  J(0,0) = 0.0;
  J(1,0) = 1.0;
  J(2,0) = 0.0;
  J(3,0) = 0.0;
  J(4,0) = 0.0;
  J(5,0) = 0.0;
  J(0,1) = 1.0;
  J(1,1) = 0.0;
  J(2,1) = 0.0;
  J(3,1) = 0.0;
  J(4,1) = 0.0;
  J(5,1) = 0.0;
  J(0,2) = 0.0;// aprox
  J(1,2) = d5*c_phi*c_theta;
  J(2,2) = -s5;
  J(3,2) = -1.0;
  J(4,2) = 0.0;
  J(5,2) = 0.0;
  J(0,3) = d5*c_phi*c_theta*c_theta-1.0f*s_theta*s3;
  J(1,3) = -1.0*s_theta*s1;
  J(2,3) = -1.0*c_theta*s1;
  J(3,3) = 0.0;
  J(4,3) = c_theta;
  J(5,3) = -1.0*s_theta;
  J(0,4) = c_phi*c_theta*s3-d5*c_phi*c_theta*s4;
  J(1, 4) = c_phi * c_theta * s1 - d5 * c_phi * c_theta * s2;
  J(2,4) = s4*s1-1.0*s3*s2;
  J(3,4) = s2;
  J(4,4) = c_phi*s_theta;
  J(5,4) = c_phi*c_theta;


  return J;
} 