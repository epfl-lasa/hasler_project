#define __MANY_CONSTRAINTS__
#include <qpOASES.hpp>
#include "JacobianDefinitions.h"
#include "Utils.h"


#define NB_JOINTS1 7
#define NB_TASKS1 7
#define NB_CONSTRAINTS1 7
#define RCM_TOLERANCE1 1e-3f/std::sqrt(3)
#define TOOL_TOLERANCE1 1e-3f/std::sqrt(3)
#define PHI_TOLERANCE1 1e-3f


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


Eigen::Matrix<float,NB_JOINTS1,1> qpSolver(Eigen::Matrix<float,NB_JOINTS1,1> joints0, Eigen::Vector3f xTrocar,
                                           float toolOffset, Eigen::Vector3f xdTool, float phid, float dt,
                                           Eigen::Vector3f xRobotBase = Eigen::Vector3f::Zero())
{
	USING_NAMESPACE_QPOASES

	SQProblem sqp(NB_JOINTS1,NB_CONSTRAINTS1);

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
	qpOASES::real_t H_qp[NB_JOINTS1 * NB_JOINTS1];
	qpOASES::real_t A_qp[NB_CONSTRAINTS1 * NB_JOINTS1];
	qpOASES::real_t g_qp[NB_JOINTS1];
	qpOASES::real_t lb_qp[NB_JOINTS1];
	qpOASES::real_t ub_qp[NB_JOINTS1];
	qpOASES::real_t lbA_qp[NB_CONSTRAINTS1];
	qpOASES::real_t ubA_qp[NB_CONSTRAINTS1];
  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;
  
	Eigen::Matrix<float,NB_TASKS1,1> error;
	Eigen::Matrix<float,NB_JOINTS1,1> joints;
	joints = joints0;

	float rcmGain = 1.0f;
	float toolGain = 1.0f;
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

	  error << xTrocar-xRCM,xdTool-xTool,phid-joints(NB_JOINTS1-1);
	  // error /= dt;
	  // std::cerr <<"error:" << error.transpose() << std::endl;
	  if(rcmGain*error.segment(0,3).norm()< 1e-4f && toolGain*error.segment(3,3).norm()< 1e-3f && std::fabs(error(6)) <1e-3f)
	  {
	  	break;
	  }

	  Eigen::Matrix<float,3,NB_JOINTS1> Jrcm, Jtool;
	  Jrcm = jacobianRCM(joints,xTrocar);
	  Jtool = jacobianTool(joints,toolOffset);

	  Eigen::Matrix<float,NB_TASKS1,NB_JOINTS1> J;
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = rcmGain*Jrcm;
	  J.block(3,0,3,7) = toolGain*Jtool;
	  J(NB_TASKS1-1,NB_JOINTS1-1) = 1.0f;

	  Eigen::Matrix<float, NB_JOINTS1, NB_JOINTS1> H;
	  // H = 1.0f*(J.transpose()*J)+0.01f*Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1>::Identity();
	  H = 1.0f*(J.transpose()*J)+0.0001f*Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1>::Identity();
	  // H = 1.0f*(J.transpose()*J);

	  Eigen::Matrix<float,NB_JOINTS1,1> g;
	  g = -1.0f*(J.transpose()*error);

	  Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1> A;
	  A = dt*Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1>::Identity();

	  Eigen::Matrix<float,NB_JOINTS1,1> lbA, ubA, lb, ub, temp;

	  temp(0) = 170.0f*M_PI/180.0f;
	  temp(1) = 120.0f*M_PI/180.0f;
	  temp(2) = 170.0f*M_PI/180.0f;
	  temp(3) = 120.0f*M_PI/180.0f;
	  temp(4) = 170.0f*M_PI/180.0f;
	  temp(5) = 120.0f*M_PI/180.0f;
	  temp(6) = 170.0f*M_PI/180.0f;

	  lbA = -temp-joints;
	  ubA = temp-joints;

	  ub(0) = 110.0f*M_PI/180.0f;
	  ub(1) = 110.0f*M_PI/180.0f;
	  ub(2) = 128.0f*M_PI/180.0f;
	  ub(3) = 128.0f*M_PI/180.0f;
	  ub(4) = 204.0f*M_PI/180.0f;
	  ub(5) = 184.0f*M_PI/180.0f;
	  ub(6) = 184.0f*M_PI/180.0f;	 
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

	  for (size_t i = 0; i < NB_CONSTRAINTS1; i++) 
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

		Eigen::Matrix<float,NB_JOINTS1,1> jointVelocities;

		real_t xOpt[NB_JOINTS1];
	  sqp.getPrimalSolution(xOpt);

	  for(int k = 0; k < NB_JOINTS1; k++)
	  {
	  	jointVelocities(k) = xOpt[k];
	  }

		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    // std::cerr << count << " jointsVelocities: " << jointVelocities.transpose() << std::endl;
	    joints += dt*jointVelocities;
		}
  }


  std::cerr << count << std::endl;
  if(rcmGain*error.segment(0,3).norm()> 1e-3f || toolGain*error.segment(3,3).norm() > 1e-3f || std::fabs(error(6)) > 1e-3f)
  {
  	std::cerr << 0 << std::endl;
  	return joints0;
  }

	std::cerr << 1 << std::endl;

  return joints;	
}

// #include <qpOASES.hpp>

// #include "jacobianRCM.h"
// #include "jacobianTool.h"

// #define NB_JOINTS1 7
// #define NB_TASKS1 7
// #define NB_CONSTRAINTS1 7
// #define RCM_TOLERANCE1 1e-3f/std::sqrt(3)
// #define TOOL_TOLERANCE1 1e-3f/std::sqrt(3)
// #define PHI_TOLERANCE1 1e-3f

// Eigen::Matrix<float,NB_JOINTS1,1> qpSolver(Eigen::Matrix<float,NB_JOINTS1,1> joints0, Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt)
// {
// 	USING_NAMESPACE_QPOASES
// 	// /* Setup data of first QP. */
// 	// real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
// 	// real_t A[1*2] = { 1.0, 1.0 };
// 	// real_t g[2] = { 1.5, 1.0 };
// 	// real_t lb[2] = { 0.5, -2.0 };
// 	// real_t ub[2] = { 5.0, 2.0 };
// 	// real_t lbA[1] = { -1.0 };
// 	// real_t ubA[1] = { 2.0 };
// 	// /* Setup data of second QP. */
// 	// real_t g_new[2] = { 1.0, 1.5 };
// 	// real_t lb_new[2] = { 0.0, -1.0 };
// 	// real_t ub_new[2] = { 5.0, -0.5 };
// 	// real_t lbA_new[1] = { -2.0 };
// 	// real_t ubA_new[1] = { 1.0 };
// 	// /* Setting up QProblem object. */
// 	// QProblem example( 2,1 );
// 	// /* Solve first QP. */
// 	// int nWSR = 10;
// 	// example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

// 	// /* Solve second QP. */
// 	// nWSR = 10;
// 	// example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );
// 	// /* Get and print solution of second QP. */
// 	// real_t xOpt[2];
// 	// example.getPrimalSolution( xOpt );
// 	// printf( "\n xOpt = [ %e, %e ];  objVal = %e\n\n",
// 	// xOpt[0],xOpt[1],example.getObjVal() );

// 	QProblem qp(NB_JOINTS1,NB_CONSTRAINTS1);

// 	auto options = qp.getOptions();
//   // options.printLevel = qpOASES::PL_NONE;
//   options.printLevel = qpOASES::PL_LOW;
//   // options.enableFarBounds = qpOASES::BT_TRUE;
//   // options.enableFlippingBounds = qpOASES::BT_TRUE;
//   options.enableRamping = qpOASES::BT_FALSE;
//   options.enableNZCTests = qpOASES::BT_FALSE;
//   options.enableDriftCorrection = 0;
//   options.terminationTolerance = 1e-6;
//   options.boundTolerance = 1e-4;
//   options.epsIterRef = 1e-6;
//   qp.setOptions(options);

//   // // we need this, because QPOases changes these values
//   // double max_time = _max_time;
//   // int max_iters = _max_iters;

//   Eigen::Matrix4f Hfk;
//   Hfk = Utils<float>::getForwardKinematics(joints0);

//   Eigen::Vector3f xEE, xRCM0, xTool0;
//   Eigen::Matrix3f wRb;

//   xEE = Hfk.block(0,3,3,1);
//   wRb = Hfk.block(0,0,3,3);

//   xRCM0 = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
//   xTool0 = xEE+toolOffset*wRb.col(2);

//   Eigen::Matrix<float,3,NB_JOINTS1> Jrcm, Jtool;
//   Jrcm = jacobianRCM(joints0,xTrocar);
//   Jtool = jacobianTool(joints0,toolOffset);

//   Eigen::Matrix<float,NB_TASKS1,NB_JOINTS1> J;
//   J.setConstant(0.0f);
//   J.block(0,0,3,7) = Jrcm;
//   J.block(3,0,3,7) = Jtool;
//   J(NB_TASKS1-1,NB_JOINTS1-1) = 1.0f;

//   Eigen::Matrix<float, NB_JOINTS1, NB_JOINTS1> H;
//   H = 2.0f*(J.transpose()*J);

//   Eigen::Matrix<float, NB_TASKS1,1> error;
// 	error << xTrocar-xRCM0,xdTool-xTool0,phid-joints0(NB_JOINTS1-1);

//   Eigen::Matrix<float,NB_JOINTS1,1> g;
//   g = -2.0f*(J.transpose()*error);

//   Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1> A;
//   A = dt*Eigen::Matrix<float,NB_JOINTS1,NB_JOINTS1>::Identity();

//   Eigen::Matrix<float,NB_JOINTS1,1> lbA, ubA, lb, ub, temp;

//   temp(0) = 170.0f*M_PI/180.0f;
//   temp(1) = 120.0f*M_PI/180.0f;
//   temp(2) = 170.0f*M_PI/180.0f;
//   temp(3) = 120.0f*M_PI/180.0f;
//   temp(4) = 170.0f*M_PI/180.0f;
//   temp(5) = 120.0f*M_PI/180.0f;
//   temp(6) = 170.0f*M_PI/180.0f;

//   lbA = -temp-joints0;
//   ubA = temp-joints0;

//   ub(0) = 110.0f*M_PI/180.0f;
//   ub(1) = 110.0f*M_PI/180.0f;
//   ub(2) = 128.0f*M_PI/180.0f;
//   ub(3) = 128.0f*M_PI/180.0f;
//   ub(4) = 204.0f*M_PI/180.0f;
//   ub(5) = 184.0f*M_PI/180.0f;
//   ub(6) = 184.0f*M_PI/180.0f;	 
//   lb = -ub;

//   // qpOASES uses row-major storing
// 	qpOASES::real_t H_qp[NB_JOINTS1 * NB_JOINTS1];
// 	qpOASES::real_t A_qp[NB_CONSTRAINTS1 * NB_JOINTS1];
// 	qpOASES::real_t g_qp[NB_JOINTS1];
// 	qpOASES::real_t lb_qp[NB_JOINTS1];
// 	qpOASES::real_t ub_qp[NB_JOINTS1];
// 	qpOASES::real_t lbA_qp[NB_CONSTRAINTS1];
// 	qpOASES::real_t ubA_qp[NB_CONSTRAINTS1];

//   for (int i = 0; i < H.rows(); i++) 
//   {
//     for (int j = 0; j < H.cols(); j++)
//     {
//       H_qp[i*H.cols()+j] = H(i,j);
//     }
//   }

//   for (int i = 0; i < A.rows(); i++)
//   {
//     for (int j = 0; j < A.cols(); j++)
//     {
//       A_qp[i*A.cols()+j] = A(i,j);
//     }
//   }

//   for (int i = 0; i < g.size(); i++) 
//   {
//     g_qp[i] = g(i);
//     lb_qp[i] = lb(i);
//     ub_qp[i] = ub(i);
//   }

//   for (size_t i = 0; i < NB_CONSTRAINTS1; i++) 
//   {
//     lbA_qp[i] = lbA(i);
//     ubA_qp[i] = ubA(i);
//   }

//   qpOASES::SymSparseMat H_mat(H.rows(), H.cols(), H.cols(), H_qp);
//   H_mat.createDiagInfo();
//   qpOASES::SparseMatrix A_mat(A.rows(), A.cols(), A.cols(), A_qp);
//   qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

//   int max_iters = 500;
// 	ret = qp.init(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);

// 	Eigen::Matrix<float,NB_JOINTS1,1> jointVelocities;

// 	real_t xOpt[NB_JOINTS1];
//   qp.getPrimalSolution(xOpt);

//   for(int k = 0; k < NB_JOINTS1; k++)
//   {
//   	jointVelocities(k) = xOpt[k];
//   }

// 	if (ret == qpOASES::SUCCESSFUL_RETURN)
// 	{

// 		Eigen::Matrix<float,NB_JOINTS1,1> joints;

// 	  joints = joints0+dt*jointVelocities;

// 	  Hfk = Utils<float>::getForwardKinematics(joints);

//     Eigen::Vector3f xRCM, xTool;

//     xEE = Hfk.block(0,3,3,1);
//     wRb = Hfk.block(0,0,3,3);

//     xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
//     xTool = xEE+toolOffset*wRb.col(2);

//     std::cerr << "jointsVelocities: " << jointVelocities.transpose() << std::endl;

//     // std::cerr << "joints: " << joints.transpose() << std::endl;
//     // std::cerr << xTrocar.transpose() << std::endl;
//     // std::cerr << xRCM.transpose() << std::endl;
//     // std::cerr << xdTool.transpose() << std::endl;
//     // std::cerr << xTool.transpose() << std::endl;
//     // std::cerr << phid << std::endl;
//     // std::cerr << joints(NB_JOINTS1-1) << std::endl;
//     error << xTrocar-xRCM,xdTool-xTool,phid-joints(NB_JOINTS1-1);
//     std::cerr << "error:" << error.transpose() << std::endl;

// 	  return joints;	
// 	}
// 	else
// 	{
// 		return joints0;
// 	}
// }