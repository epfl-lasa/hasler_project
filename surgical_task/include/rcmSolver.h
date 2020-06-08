#include "JacobianDefinitions.h"
#include "Utils.h"

Eigen::Matrix<float,7,1> rcmSolve(Eigen::Matrix<float,7,1> joints0, Eigen::Vector3f xTrocar)
{

	Eigen::Vector3f x;
	Eigen::Matrix3f wRb;
	Eigen::Matrix4f H;
	Eigen::Vector3f xRCM;
	float errorNorm = 1000.0f; 
	Eigen::Vector3f error;
	Eigen::Matrix<float,3,7> J;
	Eigen::Matrix<float,7,1> joints, delta;

	joints = joints0;

	int count = 0;

  while(errorNorm > 1e-4f)
  {
    count++;

    if(count > 500)
    {
        break;
    }

		H = Utils<float>::getForwardKinematics(joints);
		x = H.block(0,3,3,1);
		wRb = H.block(0,0,3,3);

		xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);

		error = xTrocar-xRCM; 

       
  	J = jacobianRCM(joints,xTrocar);

   //  // Singular value decomposition obect
   //  Eigen::JacobiSVD<Eigen::MatrixXf> svd;

   //  // Weight matrix
   //  Eigen::Matrix<float,7,3> W;
   //  W.setConstant(0.0f);
    
   //  // Matrix of singular values
   //  Eigen::MatrixXf singularValues;

   //  // Inverse of the jacobian matrix of the forward kinematics
   //  Eigen::Matrix<float,7,3> jacobianInverse;

   //  svd = Eigen::JacobiSVD<Eigen::MatrixXf>(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
   //  singularValues = svd.singularValues();


  	// // Compute pseudo inverse jacobian matrix
   //  float tolerance = 1e-6f*std::max(J.rows(),J.cols())*singularValues.array().abs().maxCoeff();

   //  for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
   //  {
   //    if(singularValues(m,0)>tolerance)
   //    {
   //      W(m,m) = 1.0f/singularValues(m,0);
   //    }
   //    else
   //    {
   //      // If a singular value is close to zero, it usually means we are close to a singularity, which might gives big values in the computation
   //      // of the pseudo inverse matrix => we ignore this value by setting the inverse to zero
   //      W(m,m) = 0.0f;
   //    }
   //  }
      
   //  jacobianInverse = svd.matrixV()*W*svd.matrixU().adjoint();

    Eigen::Matrix<float,7,3> Jinv;
    Jinv = J.colPivHouseholderQr().solve(Eigen::Matrix3f::Identity());
  	// delta = J.colPivHouseholderQr().solve(error);
  	// delta = Jinv*error;
  	delta = Jinv*error+(Eigen::Matrix<float,7,7>::Identity()-Jinv*J)*(joints0-joints);
  	
  	// delta = jacobianInverse*error;

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

    errorNorm = error.norm();

    std::cerr << errorNorm << std::endl;
	}

	std::cerr << count << std::endl;

	return joints;
}