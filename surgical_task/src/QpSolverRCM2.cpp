#include "QpSolverRCM2.h"
#include "Utils.h"


QpSolverRCM2::QpSolverRCM2()
{
	_rcmGain = 1.0f;
	_toolGain = 1.0f;
	_slackGains.setConstant(1000.0f);
	// _slackGains(NB_JOINTS-1) = 0.0f;
	_slackLimits.setConstant(0.1f);
	// _slackLimits << RCM_TOLERANCE,
	// 								RCM_TOLERANCE,
	// 								RCM_TOLERANCE,
	// 								TOOL_TOLERANCE,
	// 								TOOL_TOLERANCE,
	// 								TOOL_TOLERANCE,
	// 								PHI_TOLERANCE;

	_H.setConstant(0.0f);
	_A.setConstant(0.0);
	_g.setConstant(0.0f);
	_lb.setConstant(0.0f);
	_ub.setConstant(0.0f);
	_lbA.setConstant(0.0f);
	_ubA.setConstant(0.0f);

  _jointLimits(0) = 170.0f*M_PI/180.0f;
  _jointLimits(1) = 120.0f*M_PI/180.0f;
  _jointLimits(2) = 170.0f*M_PI/180.0f;
  _jointLimits(3) = 120.0f*M_PI/180.0f;
  _jointLimits(4) = 170.0f*M_PI/180.0f;
  _jointLimits(5) = 120.0f*M_PI/180.0f;
  _jointLimits(6) = 170.0f*M_PI/180.0f;

  _jointVelocitiesLimits(0) = 110.0f*M_PI/180.0f;
  _jointVelocitiesLimits(1) = 110.0f*M_PI/180.0f;
  _jointVelocitiesLimits(2) = 128.0f*M_PI/180.0f;
  _jointVelocitiesLimits(3) = 128.0f*M_PI/180.0f;
  _jointVelocitiesLimits(4) = 204.0f*M_PI/180.0f;
  _jointVelocitiesLimits(5) = 184.0f*M_PI/180.0f;
  _jointVelocitiesLimits(6) = 184.0f*M_PI/180.0f;	

	_H_qp = new qpOASES::real_t[_H.rows()*_H.cols()];
	_A_qp = new qpOASES::real_t[_A.rows()*_A.cols()];
	_g_qp = new qpOASES::real_t[_g.rows()];
	_lb_qp = new qpOASES::real_t[_lb.rows()];
	_ub_qp = new qpOASES::real_t[_ub.rows()];
	_lbA_qp = new qpOASES::real_t[_lbA.rows()];
	_ubA_qp = new qpOASES::real_t[_ubA.rows()];

	_sqp = new SQProblem(NB_VARIABLES_2,NB_CONSTRAINTS_2);

	auto options = _sqp->getOptions();

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
  _sqp->setOptions(options);

}

QpSolverRCM2::~QpSolverRCM2()
{
	delete _H_qp;
	delete _A_qp;
	delete _g_qp;
	delete _lb_qp;
	delete _ub_qp;
	delete _lbA_qp;
	delete _ubA_qp;
	delete _sqp;
}

bool QpSolverRCM2::step(Eigen::VectorXf &joints, Eigen::Matrix<float,NB_JOINTS,1> joints0, 
	                     Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt,
	                     Eigen::Vector3f xRobotBasis)
{


  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;
	Eigen::Matrix<float,NB_TASKS_2,1> error;
	joints = joints0;
	Eigen::Matrix<float,NB_JOINTS,1> jointVelocities;
	jointVelocities.setConstant(0.0f);
	real_t xOpt[NB_JOINTS+NB_SLACKS_2];

	qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

  while(count<500)
  {
  	count++;

	  Eigen::Matrix4f Hfk;
	  Hfk = Utils<float>::getForwardKinematics(joints);

	  Eigen::Vector3f xEE, xRCM, xTool;
	  Eigen::Matrix3f wRb;

	  xEE = Hfk.block(0,3,3,1) + xRobotBasis;
	  wRb = Hfk.block(0,0,3,3);

	  xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
	  xTool = xEE+toolOffset*wRb.col(2);

	  error << _rcmGain*(xTrocar-xRCM),_toolGain*(xdTool-xTool),phid-joints(NB_JOINTS-1);
	  if(checkConvergence(error))
	  {
	  	break;
	  }

	  Eigen::Matrix<float,3,NB_JOINTS> Jrcm, Jtool;
	  Jrcm = jacobianRCM(joints,xTrocar);
	  Jtool = jacobianTool(joints,toolOffset);

	  Eigen::Matrix<float,NB_TASKS_2,NB_JOINTS> J;
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = Jrcm;
	  J.block(3,0,3,7) = Jtool;
	  
	  J(NB_TASKS_2-1,NB_JOINTS-1) = 1.0f;
	  if(count==1)
	  {
	  	std::cerr << error.transpose() << std::endl;
	  }
	  	// std::cerr << Jrcm <<std::endl << std::endl;
	  	// std::cerr << Jtool <<std::endl;

	  _H.setIdentity();
	  _H.setConstant(0.0f);
	  // _H.block(0,0,NB_JOINTS,NB_JOINTS) = dt*dt*0.001*Eigen::Matrix<float,NB_JOINTS,NB_JOINTS>::Identity();
	  _H.block(0,0,NB_JOINTS,NB_JOINTS) = 1.0f*Eigen::Matrix<float,NB_JOINTS,NB_JOINTS>::Identity();
	  _H.block(NB_JOINTS,NB_JOINTS,NB_SLACKS_2,NB_SLACKS_2) = _slackGains.asDiagonal();
	  _H(NB_VARIABLES_2-1,NB_VARIABLES_2-1) = 1.0f;
	  // _g.segment(0,NB_JOINTS) = 0.001*dt*joints0;

	  _A.setConstant(0.0f);
	  _A.block(0,0,NB_TASKS_2,NB_JOINTS) = J;
	  _A.block(0,NB_JOINTS,NB_TASKS_2,NB_SLACKS_2) = -Eigen::Matrix<float,NB_SLACKS_2,NB_SLACKS_2>::Identity();
	  _A.block(NB_TASKS_2,0,NB_JOINTS,NB_JOINTS) = Eigen::Matrix<float,NB_JOINTS,NB_JOINTS>::Identity();
	  _A.block(NB_TASKS_2,NB_VARIABLES_2-1,NB_JOINTS,1) = -_jointVelocitiesLimits;
	  _A.block(NB_TASKS_2+NB_JOINTS,0,NB_JOINTS,NB_JOINTS) = Eigen::Matrix<float,NB_JOINTS,NB_JOINTS>::Identity();
	  _A.block(NB_TASKS_2+NB_JOINTS,NB_VARIABLES_2-1,NB_JOINTS,1) = _jointVelocitiesLimits;
	  // _A.block(NB_TASKS_2,0,NB_JOINTS,NB_JOINTS) = dt*Eigen::Matrix<float,NB_JOINTS,NB_JOINTS>::Identity();
	  
	  _ubA.segment(0,NB_TASKS_2) = error;
	  _lbA.segment(0,NB_TASKS_2) = error;
	  _ubA.segment(NB_TASKS_2,NB_JOINTS).setConstant(0.0f);
	  _lbA.segment(NB_TASKS_2,NB_JOINTS).setConstant(-10.0f);
	  _ubA.segment(NB_TASKS_2+NB_JOINTS,NB_JOINTS).setConstant(10.0f);
	  _lbA.segment(NB_TASKS_2+NB_JOINTS,NB_JOINTS).setConstant(0.0f);

	  // _ubA.segment(NB_TASKS_2,NB_JOINTS) = _jointLimits-joints;
	  // _lbA.segment(NB_TASKS_2,NB_JOINTS) = -_jointLimits-joints;

		_ub.segment(0,NB_JOINTS) = _jointLimits-joints; 
	  _ub.segment(NB_JOINTS,NB_SLACKS_2) = _slackLimits;
	  _ub(NB_VARIABLES_2-1) = 10.0f;
		_lb.segment(0,NB_JOINTS) = -_jointLimits-joints; 
	  _lb.segment(NB_JOINTS,NB_SLACKS_2) = -_slackLimits;
	  _lb(NB_VARIABLES_2-1) =1e-6;

	  copyQpOASESVariables();

	  qpOASES::SymSparseMat H_mat(_H.rows(), _H.cols(), _H.cols(), _H_qp);
	  H_mat.createDiagInfo();
	  qpOASES::SparseMatrix A_mat(_A.rows(), _A.cols(), _A.cols(), _A_qp);

	  int max_iters = 1000;
	  double max_time = 0.005f;

	  if(count==1)
	  {
			ret = _sqp->init(&H_mat, _g_qp, &A_mat, _lb_qp, _ub_qp, _lbA_qp, _ubA_qp, max_iters);
	  }
	  else
	  {
	  	ret = _sqp->hotstart(&H_mat, _g_qp, &A_mat, _lb_qp, _ub_qp, _lbA_qp, _ubA_qp, max_iters);
	  }

		// for(int k = 0; k < NB_JOINTS; k++)
	 //  {
	 //  	xOpt[k] = 0.0f;
	 //  }  
	  _sqp->getPrimalSolution(xOpt);

	  for(int k = 0; k < NB_JOINTS; k++)
	  {
	  	jointVelocities(k) = xOpt[k];
	  	// std::cerr << jointVelocities.transpose() << std::endl;
	  }

		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    	// std::cerr << count << " jointsVelocities: " << jointVelocities.transpose() << std::endl;
	    	joints += jointVelocities;
	    	// std::cerr << xOpt[NB_VARIABLES_2-1] << std::endl;
	    	// std::cerr << jointVelocities/xOpt[NB_VARIABLES_2-1]*180.0f/M_PI << std::endl;
		}

  }

  float sum = (joints-joints0).array().abs().sum();
  float maxDiff = (joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  // std::cerr << (joints-joints0).transpose() << std::endl;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180.0f/M_PI<<std::endl;


  // if(maxDiff>10)
  // {
  // 	joints = joints0;  	
  // 	return false;
  // }

  std::cerr << error.transpose() << std::endl;
  std::cerr << error.segment(0,3).norm() << " " << error.segment(3,3).norm() << std::endl;
  if(!checkConvergence(error))
  {
  	joints = joints0;
  	return false;
  }
  return true;	
}



void	QpSolverRCM2::setParameters(float rcmGain, float toolGain, Eigen::Matrix<float,NB_SLACKS_2,1> slackGains, 
	                       		     Eigen::Matrix<float,NB_SLACKS_2,1> slackLimits)
{
	_rcmGain = rcmGain;
	_toolGain = toolGain;
	_slackGains = slackGains;
	_slackLimits = slackLimits;
} 



bool QpSolverRCM2::checkConvergence(Eigen::Matrix<float,NB_TASKS_2,1> error)
{
	if(error.segment(0,3).norm()/_rcmGain< RCM_TOLERANCE && 
		 error.segment(3,3).norm()/_toolGain< TOOL_TOLERANCE &&
		 std::fabs(error(6)) <PHI_TOLERANCE)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void QpSolverRCM2::copyQpOASESVariables()
{
  for (int i = 0; i < _H.rows(); i++) 
  {
    for (int j = 0; j < _H.cols(); j++)
    {
      _H_qp[i*_H.cols()+j] = _H(i,j);
    }
  }

  for (int i = 0; i < _A.rows(); i++)
  {
    for (int j = 0; j < _A.cols(); j++)
    {
      _A_qp[i*_A.cols()+j] = _A(i,j);
    }
  }

  for (int i = 0; i < _g.size(); i++) 
  {
    _g_qp[i] = _g(i);
    _lb_qp[i] = _lb(i);
    _ub_qp[i] = _ub(i);
  }

  for (size_t i = 0; i < _lbA.size(); i++) 
  {
    _lbA_qp[i] = _lbA(i);
    _ubA_qp[i] = _ubA(i);
  }
}

