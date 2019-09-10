#include "FootPlatformModel.h"

Eigen::Matrix4f FootPlatformModel::forwardKinematics(Eigen::Matrix<float,5,1> joints, bool anglesInDegree)
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
  Eigen::Matrix4f H;

  float t2 = cos(theta);
  float t3 = sin(psi);
  float t4 = cos(psi);
  float t5 = cos(phi);
  float t6 = sin(phi);
  float t7 = sin(theta);
  float t9 = t6*t7*1.224646799147353e-16;
  float t8 = t5-t9;
  float t10 = t5*1.224646799147353e-16;
  float t11 = t6*t7;
  float t12 = t10+t11;
  H(0,0) = t2*t4*1.224646799147353e-16-t3*t8;
  H(1,0) = t2*t4+t3*t12;
  H(2,0) = -t4*t7+t2*t3*t6;
  H(3,0) = 0.0;
  H(0,1) = t2*t3*(-1.224646799147353e-16)-t4*t8;
  H(1,1) = -t2*t3+t4*t12;
  H(2,1) = t3*t7+t2*t4*t6;
  H(3,1) = 0.0;
  H(0,2) = t6+t5*t7*1.224646799147353e-16;
  H(1,2) = t6*(-1.224646799147353e-16)+t5*t7;
  H(2,2) = t2*t5;
  H(3,2) = 0.0;
  H(0,3) = d2+t6*(1.0/1.0e1)+t2*t4*2.585841716399636e-17-t3*t8*2.1115e-1+t5*t7*1.224646799147353e-17;
  H(1,3) = d1-t6*1.224646799147353e-17+t2*t4*2.1115e-1+t5*t7*(1.0/1.0e1)+t3*t12*2.1115e-1+1.49223212476105e-17;
  H(2,3) = t2*t5*(1.0/1.0e1)-t4*t7*2.1115e-1+t2*t3*t6*2.1115e-1+9.0/4.0e1;
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

  float t2 = cos(theta);
  float t3 = sin(theta);
  float t4 = sin(phi);
  float t5 = cos(psi);
  float t6 = cos(phi);
  float t7 = sin(psi);
  float t8 = t2*t6*(1.0/1.0e1);
  float t9 = t2*t4*t7*2.1115e-1;
  float t18 = t3*t5*2.1115e-1;
  float t10 = t8+t9-t18;
  float t11 = t2*t5*2.1115e-1;
  float t12 = t3*t6*(1.0/1.0e1);
  float t13 = t6*1.224646799147353e-16;
  float t14 = t3*t4;
  float t15 = t13+t14;
  float t16 = t7*t15*2.1115e-1;
  float t25 = t4*1.224646799147353e-17;
  float t17 = t11+t12+t16-t25;
  float t19 = t4*(1.0/1.0e1);
  float t20 = t2*t5*2.585841716399636e-17;
  float t21 = t3*t6*1.224646799147353e-17;
  float t24 = t3*t4*1.224646799147353e-16;
  float t22 = t6-t24;
  float t26 = t7*t22*2.1115e-1;
  float t23 = t19+t20+t21-t26;
  float t27 = t3*t6*1.224646799147353e-16;
  float t28 = t4+t27;
  float t29 = t4*1.224646799147353e-16;
  float t31 = t3*t6;
  float t30 = t29-t31;
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
  J(0,2) = t2*t6*1.224646799147353e-17-t3*t5*2.585841716399636e-17+t2*t4*t7*2.585841716399636e-17;
  J(1,2) = t10;
  J(2,2) = -t16-t2*t5*2.1115e-1-t3*t6*1.0e-1+t7*t22*2.585841716399636e-17;
  J(3,2) = -1.0;
  J(4,2) = 1.224646799147353e-16;
  J(5,2) = 0.0;
  J(0,3) = t2*t10+t3*t17;
  J(1,3) = t2*t10*(-1.224646799147353e-16)-t3*t23;
  J(2,3) = t2*t17*1.224646799147353e-16-t2*t23;
  J(3,3) = t2*1.224646799147353e-16;
  J(4,3) = t2;
  J(5,3) = -t3;
  J(0,4) = -t10*t30-t2*t6*t17;
  J(1,4) = -t10*t28+t2*t6*t23;
  J(2,4) = t17*t28+t23*t30;
  J(3,4) = t28;
  J(4,4) = -t29+t31;
  J(5,4) = t2*t6;


  return J;
} 