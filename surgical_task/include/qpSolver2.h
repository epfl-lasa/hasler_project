#include <qpOASES.hpp>
#include "JacobianDefinitions.h"
#include "Utils.h"


#define NB_JOINTS2 7
#define NB_TASKS2 7
#define NB_SLACKS 7
#define NB_CONSTRAINTS2 14
#define RCM_TOLERANCE2 1e-3f
#define TOOL_TOLERANCE2 1e-3f
#define PHI_TOLERANCE2 1e-3f


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


bool qpSolver2(Eigen::VectorXf &joints, Eigen::Matrix<float,NB_JOINTS2,1> joints0, Eigen::Vector3f xTrocar, 
	           float toolOffset, Eigen::Vector3f xdTool, float phid, float dt, Eigen::Vector3f xRobotBase = Eigen::Vector3f::Zero())
{
	USING_NAMESPACE_QPOASES

	SQProblem sqp(NB_JOINTS2+NB_SLACKS,NB_CONSTRAINTS2);

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
	qpOASES::real_t H_qp[(NB_JOINTS2+NB_SLACKS) * (NB_JOINTS2+NB_SLACKS)];
	qpOASES::real_t A_qp[NB_CONSTRAINTS2 * (NB_JOINTS2+NB_SLACKS)];
	qpOASES::real_t g_qp[(NB_JOINTS2+NB_SLACKS)];
	qpOASES::real_t lb_qp[(NB_JOINTS2+NB_SLACKS)];
	qpOASES::real_t ub_qp[(NB_JOINTS2+NB_SLACKS)];
	qpOASES::real_t lbA_qp[NB_CONSTRAINTS2];
	qpOASES::real_t ubA_qp[NB_CONSTRAINTS2];
  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;
  
	Eigen::Matrix<float,NB_TASKS2,1> error;
	joints = joints0;
  while(count<500)
  {
  	count++;

	  Eigen::Matrix4f Hfk;
	  Hfk = Utils<float>::getForwardKinematics(joints);

	  Eigen::Vector3f xEE, xRCM, xTool;
	  Eigen::Matrix3f wRb;

	  xEE = Hfk.block(0,3,3,1)+xRobotBase;
	  wRb = Hfk.block(0,0,3,3);

	  xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
	  xTool = xEE+toolOffset*wRb.col(2);

	  error << 1.0f*(xTrocar-xRCM),xdTool-xTool,phid-joints(NB_JOINTS2-1);
	  // error /= dt;
	  // std::cerr <<"error:" << error.transpose() << std::endl;
	  if(error.segment(0,3).norm()/1.0f< 1e-3f && error.segment(3,3).norm()< 1e-3f && std::fabs(error(6)) <1e-3f)
	  {
	  	break;
	  }

	  Eigen::Matrix<float,3,NB_JOINTS2> Jrcm, Jtool;
	  Jrcm = jacobianRCM(joints,xTrocar);
	  Jtool = jacobianTool(joints,toolOffset);

	  Eigen::Matrix<float,NB_TASKS2,NB_JOINTS2> J;
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = Jrcm;
	  J.block(3,0,3,7) = Jtool;
	  J(NB_TASKS2-1,NB_JOINTS2-1) = 1.0f;

	  Eigen::Matrix<float, NB_JOINTS2+NB_SLACKS, NB_JOINTS2+NB_SLACKS> H;
	  H.setIdentity();
	  H.setConstant(0.0f);
	  H.block(0,0,NB_JOINTS2,NB_JOINTS2) = Eigen::Matrix<float,NB_JOINTS2,NB_JOINTS2>::Identity();
	  Eigen::Matrix<float,NB_SLACKS,1> slackGains;
	  slackGains << 100.0f,100.0f,100.0f,100.0f,100.0f,100.0f,100.0f;
	  H.block(NB_JOINTS2,NB_JOINTS2,NB_SLACKS,NB_SLACKS) = slackGains.asDiagonal();

	  Eigen::Matrix<float,NB_JOINTS2+NB_SLACKS,1> g;
	  g.setConstant(0.0f);

	  Eigen::Matrix<float,NB_CONSTRAINTS2,NB_JOINTS2+NB_SLACKS> A;
	  A.setConstant(0.0f);
	  A.block(0,0,NB_TASKS2,NB_JOINTS2) = J;
	  A.block(0,NB_JOINTS2,NB_SLACKS,NB_SLACKS) = -Eigen::Matrix<float,NB_SLACKS,NB_SLACKS>::Identity();
	  A.block(NB_TASKS2,0,NB_JOINTS2,NB_JOINTS2) = dt*Eigen::Matrix<float,NB_JOINTS2,NB_JOINTS2>::Identity();

	  Eigen::Matrix<float,NB_CONSTRAINTS2,1> lbA, ubA;
	  Eigen::Matrix<float,NB_JOINTS2+NB_SLACKS,1> lb, ub;
	  Eigen::Matrix<float,NB_JOINTS2,1> temp;

	  // temp(0) = RCM_TOLERANCE2;
	  // temp(1) = RCM_TOLERANCE2;
	  // temp(2) = RCM_TOLERANCE2;
	  // temp(3) = TOOL_TOLERANCE2;
	  // temp(4) = TOOL_TOLERANCE2;
	  // temp(5) = TOOL_TOLERANCE2;
	  // temp(6) = PHI_TOLERANCE2;


	  lbA.segment(0,NB_TASKS2) = error;
	  ubA.segment(0,NB_TASKS2) = error;

	  temp(0) = 170.0f*M_PI/180.0f;
	  temp(1) = 120.0f*M_PI/180.0f;
	  temp(2) = 170.0f*M_PI/180.0f;
	  temp(3) = 120.0f*M_PI/180.0f;
	  temp(4) = 170.0f*M_PI/180.0f;
	  temp(5) = 120.0f*M_PI/180.0f;
	  temp(6) = 170.0f*M_PI/180.0f;

	  lbA.segment(NB_TASKS2,NB_JOINTS2) = -temp-joints;
	  ubA.segment(NB_TASKS2,NB_JOINTS2) = temp-joints;

	  temp(0) = 1.0f;
	  temp(1) = 1.0f;
	  temp(2) = 1.0f;
	  temp(3) = 1.0f;
	  temp(4) = 1.0f;
	  temp(5) = 1.0f;
	  temp(6) = 1.0f;

	  ub(0) = 110.0f*M_PI/180.0f;
	  ub(1) = 110.0f*M_PI/180.0f;
	  ub(2) = 128.0f*M_PI/180.0f;
	  ub(3) = 128.0f*M_PI/180.0f;
	  ub(4) = 204.0f*M_PI/180.0f;
	  ub(5) = 184.0f*M_PI/180.0f;
	  ub(6) = 184.0f*M_PI/180.0f;	 
	  ub.segment(NB_JOINTS2,NB_SLACKS) = temp;
	  lb = -ub;

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

	  for (size_t i = 0; i < NB_CONSTRAINTS2; i++) 
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

		Eigen::Matrix<float,NB_JOINTS2,1> jointVelocities;

		real_t xOpt[NB_JOINTS2+NB_SLACKS];
	  sqp.getPrimalSolution(xOpt);

	  for(int k = 0; k < NB_JOINTS2; k++)
	  {
	  	jointVelocities(k) = xOpt[k];
	  }

		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    	// std::cerr << count << " jointsVelocities: " << jointVelocities.transpose() << std::endl;
	    	joints += dt*jointVelocities;
		}

  }

  float sum = (joints-joints0).array().abs().sum();
  float maxDiff = (joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  // std::cerr << (joints-joints0).transpose() << std::endl;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180.0f/M_PI <<std::endl;

  // if(maxDiff>10)
  // {
  // 	joints = joints0;
  // 	return false;
  // }

  if(error.segment(0,3).norm()/1.0f> 1e-3f || error.segment(3,3).norm() > 1e-3f || std::fabs(error(6)) > 1e-3f)
  {
  	joints = joints0;
  	return false;

  }

  return true;	
}