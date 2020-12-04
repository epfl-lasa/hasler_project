#include "QpSolverRCM3.h"
#include "Utils.h"


QpSolverRCM3::QpSolverRCM3(): _nbTasks(7), _nbJoints(7), _nbSlacks(7), 
                              _nbVariables(7), _nbConstraints(7),
                              _rcmTolerance(1e-3), _toolTolerance(1e-3), _phiTolerance(1e-2)
{
	_rcmGain = 1.0f;
	_toolGain = 1.0f;

	_slackLimits.resize(_nbJoints);
  _slackGains.resize(_nbJoints);
	_jointLimits.resize(_nbJoints);
  _jointVelocitiesLimits.resize(_nbJoints);
  _H.resize(_nbVariables,_nbVariables);
	_A.resize(_nbConstraints, _nbVariables);
	_g.resize(_nbVariables);
	_lb.resize(_nbVariables);
	_ub.resize(_nbVariables);
	_lbA.resize(_nbConstraints);
	_ubA.resize(_nbConstraints);

	_slackGains.setConstant(1000000.0f);
	_slackLimits.setConstant(0.1f);


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

	_sqp = new SQProblem(_nbVariables,_nbConstraints);
	// _qpb = new QProblemB(_nbVariables);

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

QpSolverRCM3::~QpSolverRCM3()
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

bool QpSolverRCM3::step(Eigen::VectorXf &joints, Eigen::VectorXf joints0, 
	                     Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt,
	                     Eigen::Vector3f xRobotBasis)
{

  int count = 0;
	Eigen::VectorXf error(_nbTasks);
	joints = joints0;
	Eigen::VectorXf deltaJoints(_nbJoints);
	deltaJoints.setConstant(0.0f);
	real_t xOpt[_nbJoints+_nbSlacks];

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

	  error << _rcmGain*(xTrocar-xRCM),_toolGain*(xdTool-xTool),phid-joints(_nbJoints-1);
	  if(checkConvergence(error))
	  {
	  	break;
	  }

	  Eigen::MatrixXf Jrcm(3,_nbJoints), Jtool(3,_nbJoints);
	  // Jrcm = jacobianRCM(joints,xTrocar);
	  // Jtool = jacobianTool(joints,toolOffset);
	  Jrcm  = Utils<float>::getGeometricJacobian(joints,xRCM-xEE).block(0,0,3,7);
	  Jtool  = Utils<float>::getGeometricJacobian(joints,xTool-xEE).block(0,0,3,7);

	  Eigen::MatrixXf J(_nbTasks,_nbJoints);
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = Jrcm;
	  J.block(3,0,3,7) = Jtool;	  
	  J(_nbTasks-1,_nbJoints-1) = 1.0f;

	  if(count==1)
	  {
	  	std::cerr << error.transpose() << std::endl;
	  }


	  _H = J.transpose()*J+0.0001f*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);

	  _g = -J.transpose()*error;

	  _A = dt*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);
	  // _A.setConstant(0.0f);
	  // _A.block(0,0,_nbTasks,_nbJoints) = J;
	  // _A.block(0,_nbJoints,_nbTasks,_nbSlacks) = -Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);
	  
	  _ubA = _jointLimits-joints;
	  _lbA = -_jointLimits-joints;
	  // _lbA.segment(0,_nbTasks) = error;

		_ub.segment(0,_nbJoints) = _jointVelocitiesLimits; 
		_lb.segment(0,_nbJoints) = -_jointVelocitiesLimits;
	  // _ub.segment(_nbJoints,_nbSlacks) = _slackLimits;
	  // _lb.segment(_nbJoints,_nbSlacks) = -_slackLimits;

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

	  _sqp->getPrimalSolution(xOpt);

	  // if(count==1)
	  // {
			// ret = _qpb->init(&H_mat, _g_qp, _lb_qp, _ub_qp, max_iters);
	  // }
	  // else
	  // {
	  // 	ret = _qpb->hotstart(&H_mat, _g_qp, _lb_qp, _ub_qp, max_iters);
	  // }

	  // _qpb->getPrimalSolution(xOpt);


	  for(int k = 0; k < _nbJoints; k++)
	  {
	  	deltaJoints(k) = xOpt[k];
	  }

		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    	joints += dt*deltaJoints;
		}

  }

  float sum = (joints-joints0).array().abs().sum();
  float maxDiff = (joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180.0f/M_PI<<std::endl;

  std::cerr << error.transpose() << std::endl;
  std::cerr << error.segment(0,3).norm() << " " << error.segment(3,3).norm() << std::endl;
  if(!checkConvergence(error))
  {
  	joints = joints0;
  	return false;
  }
  return true;	
}



bool QpSolverRCM3::step3(Eigen::VectorXf &joints, Eigen::VectorXf joints0, 
	                     Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f vdTool, float omegad, float dt,
	                     Eigen::Vector3f xRobotBasis)
{

  int count = 0;
	Eigen::VectorXf error(_nbTasks);
	joints = joints0;
	Eigen::VectorXf jointVelocities(_nbJoints);
	jointVelocities.setConstant(0.0f);
	real_t xOpt[_nbJoints+_nbSlacks];

	qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

  while(count<1)
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

	  // error << _rcmGain*(xTrocar-xRCM),_toolGain*vdTool,phid-joints(_nbJoints-1);
	  error << _rcmGain*(xTrocar-xRCM),_toolGain*vdTool,omegad;

	  // if(checkConvergence(error))
	  // {
	  // 	break;
	  // }

	  Eigen::MatrixXf Jrcm(3,_nbJoints), Jtool(3,_nbJoints);
	  // Jrcm = jacobianRCM(joints,xTrocar);
	  // Jtool = jacobianTool(joints,toolOffset);
	  Jrcm  = Utils<float>::getGeometricJacobian(joints,xRCM-xEE).block(0,0,3,7);
	  Jtool  = Utils<float>::getGeometricJacobian(joints,xTool-xEE).block(0,0,3,7);

	  Eigen::MatrixXf J(_nbTasks,_nbJoints);
	  J.setConstant(0.0f);
	  J.block(0,0,3,7) = Jrcm;
	  J.block(3,0,3,7) = Jtool;	  
	  J(_nbTasks-1,_nbJoints-1) = 1.0f;

	  if(count==1)
	  {
	  	std::cerr << error.transpose() << std::endl;
	    std::cerr << "input error: "<< (error-J*jointVelocities).transpose() << std::endl;	  
	  }


	  _H = J.transpose()*J+0.0001f*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);

	  _g = -J.transpose()*error;

	  _A = dt*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);
	  // _A.setConstant(0.0f);
	  // _A.block(0,0,_nbTasks,_nbJoints) = J;
	  // _A.block(0,_nbJoints,_nbTasks,_nbSlacks) = -Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);
	  
	  _ubA = _jointLimits-joints;
	  _lbA = -_jointLimits-joints;
	  // _lbA.segment(0,_nbTasks) = error;

		_ub.segment(0,_nbJoints) = _jointVelocitiesLimits; 
		_lb.segment(0,_nbJoints) = -_jointVelocitiesLimits;
	  // _ub.segment(_nbJoints,_nbSlacks) = _slackLimits;
	  // _lb.segment(_nbJoints,_nbSlacks) = -_slackLimits;

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

	  _sqp->getPrimalSolution(xOpt);

	  // if(count==1)
	  // {
			// ret = _qpb->init(&H_mat, _g_qp, _lb_qp, _ub_qp, max_iters);
	  // }
	  // else
	  // {
	  // 	ret = _qpb->hotstart(&H_mat, _g_qp, _lb_qp, _ub_qp, max_iters);
	  // }

	  // _qpb->getPrimalSolution(xOpt);


	  for(int k = 0; k < _nbJoints; k++)
	  {
	  	jointVelocities(k) = xOpt[k];
	  }

		if(ret == qpOASES::SUCCESSFUL_RETURN)
		{
	    	joints += dt*jointVelocities;
  			std::cerr << "output error: "<< (error-J*jointVelocities).transpose() << std::endl;
		}
  }

  float sum = (joints-joints0).array().abs().sum();
  float maxDiff = (joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180.0f/M_PI<<std::endl;

  if(maxDiff>10)
  {
  	joints = joints0;  	
  	return false;
  }

  std::cerr << error.transpose() << std::endl;
  std::cerr << error.segment(0,3).norm() << " " << error.segment(3,3).norm() << std::endl;
  // if(!checkConvergence(error))
  // {
  // 	joints = joints0;
  // 	return false;
  // }
  return true;	
}




void	QpSolverRCM3::setParameters(float rcmGain, float toolGain, Eigen::VectorXf slackGains, 
	                       		      Eigen::VectorXf slackLimits)
{
	_rcmGain = rcmGain;
	_toolGain = toolGain;
	_slackGains = slackGains;
	_slackLimits = slackLimits;
} 



bool QpSolverRCM3::checkConvergence(Eigen::VectorXf error)
{
	if(error.segment(0,3).norm()/_rcmGain< _rcmTolerance && 
		 error.segment(3,3).norm()/_toolGain< _toolTolerance &&
		 std::fabs(error(6)) <_phiTolerance)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void QpSolverRCM3::copyQpOASESVariables()
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

