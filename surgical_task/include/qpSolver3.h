#include <qpOASES.hpp>
#include "JacobianDefinitions.h"
#include "Utils.h"

#define NB_JOINTS3 7
#define NB_TASKS3 7
#define NB_SLACKS3 0
#define NB_CONSTRAINTS3 14
#define RCM_TOLERANCE3 1e-3f
#define TOOL_TOLERANCE3 1e-3f
#define PHI_TOLERANCE3 1e-3f


// Eigen::MatrixXf pseudoInverse(const Eigen::MatrixXf &M_, bool damped = true)
// { 
//   double lambda_ = damped?0.1:0.0;

//   Eigen::JacobiSVD<Eigen::MatrixXf> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
//   Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType sing_vals_ = svd.singularValues();
//   Eigen::MatrixXf S_ = M_; // copying the dimensions of M_, its content is not needed.
//   S_.setZero();

//     for (int i = 0; i < sing_vals_.size(); i++)
//         S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

//   Eigen::MatrixXf Mpinv;
//   Mpinv = Eigen::MatrixXf(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
//   return Mpinv;
// }


Eigen::Matrix<float,NB_JOINTS3,1> qpSolver3(Eigen::Matrix<float,NB_JOINTS3,1> joints0, Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt)
{
	USING_NAMESPACE_QPOASES

	SQProblem sqp(NB_JOINTS3+NB_SLACKS3,NB_CONSTRAINTS3);

	auto options = sqp.getOptions();
  // options.printLevel = qpOASES::PL_NONE;
  options.printLevel = qpOASES::PL_LOW;
  // options.enableFarBounds = qpOASES::BT_TRUE;
  // options.enableFlippingBounds = qpOASES::BT_TRUE;
  options.enableRamping = qpOASES::BT_FALSE;
  options.enableNZCTests = qpOASES::BT_FALSE;
  // options.enableRegularisation = qpOASES::BT_TRUE;
  options.enableDriftCorrection = 0;
  options.terminationTolerance = 1e-6;
  options.boundTolerance = 1e-4;
  options.epsIterRef = 1e-6;
  sqp.setOptions(options);

  // qpOASES uses row-major storing
	qpOASES::real_t H_qp[(NB_JOINTS3+NB_SLACKS3) * (NB_JOINTS3+NB_SLACKS3)];
	qpOASES::real_t A_qp[NB_CONSTRAINTS3 * (NB_JOINTS3+NB_SLACKS3)];
	qpOASES::real_t g_qp[(NB_JOINTS3+NB_SLACKS3)];
	qpOASES::real_t lb_qp[(NB_JOINTS3+NB_SLACKS3)];
	qpOASES::real_t ub_qp[(NB_JOINTS3+NB_SLACKS3)];
	qpOASES::real_t lbA_qp[NB_CONSTRAINTS3];
	qpOASES::real_t ubA_qp[NB_CONSTRAINTS3];
  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;
  
	Eigen::Matrix<float,NB_TASKS3,1> error;
	Eigen::Matrix<float,NB_JOINTS3,1> joints;
	joints = joints0;
	float T = dt;
  while(count<500)
  {
  	count++;

	  Eigen::Matrix4f Hfk;
	  Hfk = Utils<float>::getForwardKinematics(joints);

	  Eigen::Vector3f xEE, xRCM, xTool;
	  Eigen::Matrix3f wRb;

	  xEE = Hfk.block(0,3,3,1);
	  wRb = Hfk.block(0,0,3,3);

	  xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
	  xTool = xEE+toolOffset*wRb.col(2);

	  error << xTrocar-xRCM,xdTool-xTool,phid-joints(NB_JOINTS3-1);
	  // error /= dt;
	  // std::cerr <<"error:" << error.transpose() << std::endl;
	  if(error.segment(0,3).norm()< 1e-3f && error.segment(3,3).norm()< 1e-3f && std::fabs(error(6)) <1e-3f)
	  {
	  	break;
	  }

	  Eigen::Matrix<float,3,NB_JOINTS3> Jrcm, Jtool;
	  Jrcm = jacobianRCM(joints,xTrocar);
	  Jtool = jacobianTool(joints,toolOffset);

	  Eigen::Matrix<float,NB_TASKS3,NB_JOINTS3> J;
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = Jrcm;
	  J.block(3,0,3,7) = Jtool;
	  J(NB_TASKS3-1,NB_JOINTS3-1) = 1.0f;

	  Eigen::Matrix<float, NB_JOINTS3+NB_SLACKS3, NB_JOINTS3+NB_SLACKS3> H;
	  H.setConstant(0.0f);
	  H.block(0,0,NB_JOINTS3,NB_JOINTS3) = Eigen::Matrix<float,NB_JOINTS3,NB_JOINTS3>::Identity();
	  // H(NB_JOINTS3+NB_SLACKS3-1,NB_JOINTS3+NB_SLACKS3-1) = 100.0f;

	  Eigen::Matrix<float,NB_JOINTS3+NB_SLACKS3,1> g;
	  g.setConstant(0.0f);

	  Eigen::Matrix<float,NB_CONSTRAINTS3,NB_JOINTS3+NB_SLACKS3> A;
	  A.setConstant(0.0f);
	  A.block(0,0,NB_TASKS3,NB_JOINTS3) = J;
	  A.block(NB_TASKS3,0,NB_JOINTS3,NB_JOINTS3) = Eigen::Matrix<float,NB_JOINTS3,NB_JOINTS3>::Identity();


	 	Eigen::Matrix<float,NB_JOINTS3,1> jointLimits;

	  jointLimits(0) = 170.0f*M_PI/180.0f;
	  jointLimits(1) = 120.0f*M_PI/180.0f;
	  jointLimits(2) = 170.0f*M_PI/180.0f;
	  jointLimits(3) = 120.0f*M_PI/180.0f;
	  jointLimits(4) = 170.0f*M_PI/180.0f;
	  jointLimits(5) = 120.0f*M_PI/180.0f;
	  jointLimits(6) = 170.0f*M_PI/180.0f;

	  Eigen::Matrix<float,NB_CONSTRAINTS3,1> lbA, ubA;
	  lbA.segment(0,NB_TASKS3) = error;
	  ubA.segment(0,NB_TASKS3) = error;
	  lbA.segment(NB_TASKS3,NB_JOINTS3) = -jointLimits-joints;
	  ubA.segment(NB_TASKS3,NB_JOINTS3) = jointLimits-joints;

	  Eigen::Matrix<float,NB_JOINTS3+NB_SLACKS3,1> lb, ub;
	  ub(0) = 110.0f*M_PI/180.0f;
	  ub(1) = 110.0f*M_PI/180.0f;
	  ub(2) = 128.0f*M_PI/180.0f;
	  ub(3) = 128.0f*M_PI/180.0f;
	  ub(4) = 204.0f*M_PI/180.0f;
	  ub(5) = 184.0f*M_PI/180.0f;
	  ub(6) = 184.0f*M_PI/180.0f;	 
	  lb = -ub;
	  // lb(NB_JOINTS3+NB_SLACKS3-1) = 1e-6f;
	  // ub(NB_JOINTS3+NB_SLACKS3-1) = 1e2f;

	  for (int i = 0; i < H.rows(); i++) 
	  {
	    for (int j = 0; j < H.cols(); j++)
	    {
	      H_qp[i*H.cols()+j] = H(i,j);
	    }
	  }

	  for (int i = 0; i < A.rows(); i++)
	  {
	    for (int j = 0; j < A.cols(); j++)
	    {
	      A_qp[i*A.cols()+j] = A(i,j);
	    }
	  }

	  for (int i = 0; i < g.size(); i++) 
	  {
	    g_qp[i] = g(i);
	    lb_qp[i] = lb(i);
	    ub_qp[i] = ub(i);
	  }

	  for (size_t i = 0; i < NB_CONSTRAINTS3; i++) 
	  {
	    lbA_qp[i] = lbA(i);
	    ubA_qp[i] = ubA(i);
	  }

	  qpOASES::SymSparseMat H_mat(H.rows(), H.cols(), H.cols(), H_qp);
	  H_mat.createDiagInfo();
	  qpOASES::SparseMatrix A_mat(A.rows(), A.cols(), A.cols(), A_qp);
	  qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

	  int max_iters = 1000;
	  double max_time = 0.005f;

	  if(count==1)
	  {
			ret = sqp.init(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);
	  	
	  }
	  else
	  {
	  	ret = sqp.hotstart(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);
	  }

		Eigen::Matrix<float,NB_JOINTS3,1> jointVelocities;

		real_t xOpt[NB_JOINTS3+NB_SLACKS3];
	  sqp.getPrimalSolution(xOpt);

	  for(int k = 0; k < NB_JOINTS3; k++)
	  {
	  	jointVelocities(k) = xOpt[k];
	  }

	  // T = xOpt[NB_JOINTS3];
		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    std::cerr << count << " jointsVelocities: " << jointVelocities.transpose() << std::endl;
	    joints += jointVelocities;
		}
  }


  std::cerr << count << std::endl;
  if(error.segment(0,3).norm()> 1e-3f || error.segment(3,3).norm() > 1e-3f || std::fabs(error(6)) > 1e-3f)
  {
  	return joints0;
  }

  return joints;	
}