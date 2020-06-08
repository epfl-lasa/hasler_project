#include "Utils.h"
#include "JacobianDefinitions.h"


Eigen::MatrixXf pseudoInverse(const Eigen::MatrixXf &M_, bool damped = true)
{ 
  double lambda_ = damped?0.1:0.0;

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXf S_ = M_; // copying the dimensions of M_, its content is not needed.
  S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

  Eigen::MatrixXf Mpinv;
  Mpinv = Eigen::MatrixXf(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
  return Mpinv;
}

bool checkConvergence(Eigen::Matrix<float,7,1> error)
{
	float errorRCM, errorTool, errorRotation;
	errorRCM = error.segment(0,3).norm();
	errorTool = error.segment(3,3).norm();
	errorRotation = std::fabs(error(6));

	// std::cerr << errorRCM << " " << errorTool << " " << errorRotation << std::endl;
	if(errorRCM<5e-3f && errorTool < 1e-3f && errorRotation < 1e-5f)
	{
		return true;
	}
	else
	{
		return false;
	}
}

Eigen::Matrix<float,7,1> rcmInverseKinematics(Eigen::Matrix<float,7,1> joints0, Eigen::Vector3f xTrocar, float toolOffset,
	                                          Eigen::Vector3f xdTool, float phi, Eigen::Vector3f xRobotBase = Eigen::Vector3f::Zero())
{

	Eigen::Vector3f x;
	Eigen::Vector3f xTool;
	Eigen::Matrix3f wRb;
	Eigen::Matrix4f H;
	Eigen::Vector3f xRCM;
	Eigen::Matrix<float,7,1> error;
	Eigen::Matrix<float,3,7> Jrcm;
	Eigen::Matrix<float,3,7> Jtool;
	Eigen::Matrix<float,7,7> J;
	Eigen::Matrix<float,7,1> joints, delta;

	joints = joints0;

	int count = 0;

	bool converged = false;

  while(count<1000)
  {
    count++;


	H = Utils<float>::getForwardKinematics(joints);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);

	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);

  error << xTrocar-xRCM,xdTool-xTool,phi-joints(6);
  // error << (xTrocar-x).normalized()-wRb.col(2),xdTool-xTool,phi-joints(6);
  // std::cerr << error.transpose() << std::endl;

	converged = checkConvergence(error);


  if(converged)
  {
      break;
  }
	// std::cerr << error.transpose() << std::endl;
      
    Jrcm = jacobianRCM(joints,xTrocar);
    // Jrcm = jacobianZ(joints);
  	Jtool = jacobianTool(joints,toolOffset);

    J.setConstant(0.0f);
    J.block(0,0,3,7) = Jrcm;
    J.block(3,0,3,7) = Jtool;
    J(6,6) = 1.0f;
    Eigen::Matrix<float,7,7> Jinv;
    Jinv = J.colPivHouseholderQr().solve(Eigen::Matrix<float,7,7>::Identity());
  	// delta = J.colPivHouseholderQr().solve(error);
  	// std::cerr << delta.transpose() << std::endl;


    // Singular value decomposition obect
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;

    // Weight matrix
    Eigen::Matrix<float,7,7> W;
    W.setConstant(0.0f);
    
    // Matrix of singular values
    Eigen::MatrixXf singularValues;

    // Inverse of the jacobian matrix of the forward kinematics
    Eigen::Matrix<float,7,7> jacobianInverse;

    svd = Eigen::JacobiSVD<Eigen::MatrixXf>(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    singularValues = svd.singularValues();


  	// Compute pseudo inverse jacobian matrix
    // float tolerance = 1e-6f*std::max(J.rows(),J.cols())*singularValues.array().abs().maxCoeff();
    float tolerance = 1e-4f;

    for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
    {
      if(singularValues(m,0)>tolerance)
      {
        W(m,m) = 1.0f/singularValues(m,0);
      }
      else
      {
        // If a singular value is close to zero, it usually means we are close to a singularity, which might gives big values in the computation
        // of the pseudo inverse matrix => we ignore this value by setting the inverse to zero
        W(m,m) = 0.0f;
      }
    }
      
    jacobianInverse = svd.matrixV()*W*svd.matrixU().adjoint();
  	// delta = Jinv*error;
  	// delta = Jinv*error+(Eigen::Matrix<float,7,7>::Identity()-Jinv*J)*(joints0-joints);
  	// delta = jacobianInverse*error;
    Eigen::Matrix<float,7,7> pinv;
    pinv = pseudoInverse(J);
  	delta = pinv*error;//+(Eigen::Matrix<float,7,7>::Identity()-pinv*J)*(joints0-joints);

	// std::cerr << xTool.transpose() << std::endl;
	// std::cerr << error.transpose() << std::endl;  	
  	// delta = jacobianInverse*error;

    // std::cerr << "a: " << joints.transpose() << std::endl;

    // std::cerr << "delta: " << delta.transpose() << std::endl;

  	for(int k = 0; k < 7; k++)
  	{
      joints(k) += delta(k);

      // joints(k) = fmod(joints(k),2.0f*M_PI);

	    if (joints(k) > M_PI)
	    {
		    joints(k) -= 2.0f*M_PI;
	    }
	    if (joints(k) < -M_PI)
	    {
		    joints(k) += 2.0f*M_PI;
	    }
    }
    // std::cerr << "b: " << joints.transpose() << std::endl;
    // std::cerr << "c: " <<joints.transpose() << std::endl;

	}

  // std::cerr << joints.transpose() << std::endl;

	std::cerr << count << std::endl;

	// std::cerr << "bou: " << joints.transpose() << std::endl;
	// H = Utils<float>::getForwardKinematics(joints);
	// x = H.block(0,3,3,1);
	// wRb = H.block(0,0,3,3);

	// xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	// xTool = x+toolOffset*wRb.col(2);
	// std::cerr << xTool.transpose() << std::endl;
	// std::cerr << xRCM.transpose() << std::endl;
	// error << xTrocar-xRCM,xdTool-xTool,phi-joints(6);
	// std::cerr << error.transpose() << std::endl;
  if(!converged)
  {
    return joints0;
  }

	return joints;
}


Eigen::Matrix<float,7,1> rcmInverseVelocity(Eigen::Matrix<float,7,1> joints0, Eigen::Vector3f xTrocar, float toolOffset,
	                                        Eigen::Vector3f vdTool, float vphi, float dt)
{
	Eigen::Vector3f x;
	Eigen::Vector3f xTool;
	Eigen::Matrix3f wRb;
	Eigen::Matrix4f H;
	Eigen::Vector3f xRCM;
	Eigen::Matrix<float,7,1> error;
	Eigen::Matrix<float,3,7> Jrcm;
	Eigen::Matrix<float,3,7> Jtool;
	Eigen::Matrix<float,7,7> J;
	Eigen::Matrix<float,7,1> joints, delta;

	joints = joints0;


	H = Utils<float>::getForwardKinematics(joints);
	x = H.block(0,3,3,1);
	wRb = H.block(0,0,3,3);

	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);
	error << xTrocar-xRCM,vdTool,vphi;
	// std::cerr << error.transpose() << std::endl;
      
  	Jrcm = jacobianRCM(joints,xTrocar);
  	Jtool = jacobianTool(joints,toolOffset);

    J.setConstant(0.0f);
    J.block(0,0,3,7) = Jrcm;
    J.block(3,0,3,7) = Jtool;
    J(6,6) = 1.0f;

    
    Eigen::Matrix<float,7,7> Jinv;
    Jinv = J.colPivHouseholderQr().solve(Eigen::Matrix<float,7,7>::Identity());
  	// delta = J.colPivHouseholderQr().solve(error);
  	delta = Jinv*error;
  	// delta = Jinv*error+(Eigen::Matrix<float,7,7>::Identity()-Jinv*J)*(joints0-joints);
  	
  	// delta = jacobianInverse*error;

  	for(int k = 0; k < 7; k++)
  	{
      joints(k) += dt*delta(k);

      // joints(k) = fmod(joints(k),2.0f*M_PI);

	    if (joints(k) > M_PI)
	    {
		    joints(k) -= 2.0f*M_PI;
	    }
	    if (joints(k) < -M_PI)
	    {
		    joints(k) += 2.0f*M_PI;
	    }
    }

	return joints;
}