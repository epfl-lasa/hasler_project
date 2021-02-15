#include "QpSolverRCMCollision2.h"


QpSolverRCMCollision2::QpSolverRCMCollision2():_nbTasks(7), _nbJoints(7), _nbSlacks(7), 
                              				   _nbVariables(7), _nbConstraints(8),
                              				   _rcmTolerance(1e-3), _toolTolerance(1e-3), _phiTolerance(1e-2)
{
	_rcmGain = 1.0f;
	_toolGain = 1.0f;

	_slackGains.resize(_nbSlacks);
	_slackGains.setConstant(1000.0f);
	_slackLimits.resize(_nbSlacks);
	_slackLimits.setConstant(1.0f);

	_jointMin.resize(_nbJoints);
	_jointMax.resize(_nbJoints);
	_jointVelocitiesLimits.resize(_nbJoints);

	setRobot(Utils<float>::ROBOT_ID::KUKA_LWR);

	_H.resize(_nbVariables,_nbVariables);
	_A.resize(_nbConstraints, _nbVariables);
	_g.resize(_nbVariables);
	_lb.resize(_nbVariables);
	_ub.resize(_nbVariables);
	_lbA.resize(_nbConstraints);
	_ubA.resize(_nbConstraints);

	_H.setConstant(0.0f);
	_A.setConstant(0.0);
	_g.setConstant(0.0f);
	_lb.setConstant(0.0f);
	_ub.setConstant(0.0f);
	_lbA.setConstant(0.0f);
	_ubA.setConstant(0.0f);


	_H_qp = new qpOASES::real_t[_H.rows()*_H.cols()];
	_A_qp = new qpOASES::real_t[_A.rows()*_A.cols()];
	_g_qp = new qpOASES::real_t[_g.rows()];
	_lb_qp = new qpOASES::real_t[_lb.rows()];
	_ub_qp = new qpOASES::real_t[_ub.rows()];
	_lbA_qp = new qpOASES::real_t[_lbA.rows()];
	_ubA_qp = new qpOASES::real_t[_ubA.rows()];

	_sqp = new SQProblem(_nbVariables,_nbConstraints);

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

QpSolverRCMCollision2::~QpSolverRCMCollision2()
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


void QpSolverRCMCollision2::setRobot(Utils<float>::ROBOT_ID robotID)
{
	_robotID = robotID;

	if(_robotID == Utils<float>::ROBOT_ID::KUKA_LWR)
	{
	  _jointMax(0) = 170.0f*M_PI/180.0f;
	  _jointMax(1) = 120.0f*M_PI/180.0f;
	  _jointMax(2) = 170.0f*M_PI/180.0f;
	  _jointMax(3) = 120.0f*M_PI/180.0f;
	  _jointMax(4) = 170.0f*M_PI/180.0f;
	  _jointMax(5) = 120.0f*M_PI/180.0f;
	  _jointMax(6) = 170.0f*M_PI/180.0f;
	  _jointMin = -_jointMax;

	  _jointVelocitiesLimits(0) = 110.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(1) = 110.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(2) = 128.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(3) = 128.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(4) = 204.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(5) = 184.0f*M_PI/180.0f;
	  _jointVelocitiesLimits(6) = 184.0f*M_PI/180.0f;	
	  _jointVelocitiesLimits *= 0.8f;
	}
	else if(_robotID == Utils<float>::ROBOT_ID::FRANKA_PANDA)
	{
	  _jointMax(0) = 2.8973f;
	  _jointMax(1) = 1.7628f;
	  _jointMax(2) = 2.8973f;
	  _jointMax(3) = -0.0698f;
	  _jointMax(4) = 2.8973f;
	  _jointMax(5) = 3.7525f;
	  _jointMax(6) = 2.8973f;

	  _jointMin(0) = -2.8973f;
	  _jointMin(1) = -1.7628f;
	  _jointMin(2) = -2.8973f;
	  _jointMin(3) = -3.0718f;
	  _jointMin(4) = -2.8973f;
	  _jointMin(5) = -0.0175f;
	  _jointMin(6) = -2.8973f;

	  _jointVelocitiesLimits(0) = 2.1750f;
	  _jointVelocitiesLimits(1) = 2.1750f;
	  _jointVelocitiesLimits(2) = 2.1750f;
	  _jointVelocitiesLimits(3) = 2.1750f;
	  _jointVelocitiesLimits(4) = 2.6100f;
	  _jointVelocitiesLimits(5) = 2.6100f;
	  _jointVelocitiesLimits(6) = 2.6100f;	
	  _jointVelocitiesLimits *= 0.2f;
	}
}


bool QpSolverRCMCollision2::step(Eigen::VectorXf &joints, Eigen::VectorXf joints0, 
	                     Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f vdTool, float omegad, float dt,
	                     Eigen::Vector3f xRobotBasis, Eigen::Matrix3f wRRobotBasis, float depthGain,
	                     Eigen::Vector3f obstacleDir, float obstacleDistance)
{


  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;
	Eigen::VectorXf error(_nbTasks);
	joints.resize(_nbJoints);
	joints0.resize(_nbJoints);
	joints = joints0;
	Eigen::VectorXf jointVelocities(_nbJoints);
	jointVelocities.setConstant(0.0f);
	real_t xOpt[_nbVariables];

	qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

	_slackLimits.setConstant(1.0f);
	_slackGains.setConstant(100000.0f);

	_rcmGain = depthGain;
	_rcmGain = 20.0f;
	// std::cerr << "rcmGain: " << _rcmGain << std::endl;


  Eigen::Matrix4f Hfk;
  Hfk = Utils<float>::getForwardKinematics(joints, _robotID);

  Eigen::Vector3f xEE, xRCM, xTool, zEE;
  Eigen::Matrix3f wRb;

  xEE = wRRobotBasis*Hfk.block(0,3,3,1) + xRobotBasis;
  wRb = wRRobotBasis*Hfk.block(0,0,3,3);
  zEE = wRb.col(2);

  xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
  xTool = xEE+toolOffset*wRb.col(2);

  // error << (xdTool-xTrocar).normalized()-zEE, _toolGain*(xdTool-xTool), phid-joints(_nbJoints-1);

  Eigen::Vector3f bou;
  bou = (xTrocar-xRCM);

  error << _rcmGain*wRRobotBasis.transpose()*bou, _toolGain*wRRobotBasis.transpose()*vdTool, omegad;
  // if(count==1)
  // {
  // 	std::cerr << "bou: "<< error.transpose() << std::endl;
  // }

  // if(checkConvergence(error))
  // {
  // 	break;
  // }

  Eigen::MatrixXf Jee(3, _nbJoints), Jrcm(3,_nbJoints), Jtool(3,_nbJoints);

  Jee = Utils<float>::getGeometricJacobian(joints,Eigen::Vector3f::Zero(),_robotID).block(0,0,3,7);
  Jrcm  = Utils<float>::getGeometricJacobian(joints,wRRobotBasis.transpose()*(xRCM-xEE),_robotID).block(0,0,3,7);
  Jtool  = Utils<float>::getGeometricJacobian(joints,wRRobotBasis.transpose()*(xTool-xEE),_robotID).block(0,0,3,7);

  Eigen::MatrixXf J(_nbTasks,_nbJoints);
  J.setConstant(0.0f);
  J.block(0,0,3,7) = Jrcm;
  J.block(3,0,3,7) = Jtool;

  J(_nbTasks-1,_nbJoints-1) = 1.0f;

  if(count==1)
  {
		// std::cerr << "input error: "<< (error-J*jointVelocities).transpose() << std::endl;	  
	}


	_H = J.transpose()*J+0.0001f*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);

  _g = -J.transpose()*error;

  _A.block(0,0,_nbJoints,_nbJoints) = dt*Eigen::MatrixXf::Identity(_nbJoints,_nbJoints);
  _A.block(_nbJoints,0,1,_nbJoints) = (wRRobotBasis.transpose()*obstacleDir).transpose()*Jee;
	  
	_ubA.segment(0,_nbJoints) = _jointMax-joints;
	_lbA.segment(0,_nbJoints) = _jointMin-joints;
  float ds = 0.05f, di = 0.1f;

  _lbA(_nbJoints) = -0.5*(obstacleDistance-ds)/(di-ds);
  _ubA(_nbJoints) = 1000.0f;

	_ub = _jointVelocitiesLimits; 
	_lb = -_jointVelocitiesLimits;



  copyQpOASESVariables();

  qpOASES::SymSparseMat H_mat(_H.rows(), _H.cols(), _H.cols(), _H_qp);
  H_mat.createDiagInfo();
  qpOASES::SparseMatrix A_mat(_A.rows(), _A.cols(), _A.cols(), _A_qp);

  int max_iters = 1000;
  double max_time = 0.005f;

  // if(count==1)
  {
		ret = _sqp->init(&H_mat, _g_qp, &A_mat, _lb_qp, _ub_qp, _lbA_qp, _ubA_qp, max_iters);
  }
  // else
  // {
  // 	ret = _sqp->hotstart(&H_mat, _g_qp, &A_mat, _lb_qp, _ub_qp, _lbA_qp, _ubA_qp, max_iters);
  // }

  _sqp->getPrimalSolution(xOpt);

  for(int k = 0; k < _nbJoints; k++)
  {
  	jointVelocities(k) = xOpt[k];
  }

	if(ret == qpOASES::SUCCESSFUL_RETURN)
	{
		joints += dt*jointVelocities;
		std::cerr << "jdot: " << jointVelocities.transpose() << std::endl;
		std::cerr << "jdotLim: " << _jointVelocitiesLimits.transpose() << std::endl;
			// std::cerr << "output error: "<< (error-J*jointVelocities).transpose() << std::endl;
	}
	else
	{
		std::cerr << "Error: " <<(int) ret << std::endl;
	}


  float sum = (joints-joints0).array().abs().sum();
  float maxDiff = (joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  std::cerr <<  _A.block(_nbJoints,0,1,_nbJoints)*jointVelocities << " >= " << _lbA(_nbJoints) << std::endl;
  // std::cerr << (joints-joints0).transpose() << std::endl;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180.0f/M_PI<<std::endl;


  if(maxDiff>10)
  {
  	joints = joints0;  	
  	return false;
  }

  std::cerr << error.transpose() << std::endl;
  std::cerr << error.segment(0,3).norm()/_rcmGain << " " << error.segment(3,3).norm()/_toolGain << std::endl;
  // if(!checkConvergence(error))
  // {
  // 	joints = joints0;
  // 	return false;
  // }
  return true;	
}



void	QpSolverRCMCollision2::setParameters(float rcmGain, float toolGain, Eigen::VectorXf slackGains, 
	                       		     Eigen::VectorXf slackLimits)
{
	_rcmGain = rcmGain;
	_toolGain = toolGain;
	_slackGains = slackGains;
	_slackLimits = slackLimits;
} 



bool QpSolverRCMCollision2::checkConvergence(Eigen::VectorXf error)
{
	if(error.segment(0,3).norm()/_rcmGain< _rcmTolerance && 
		 error.segment(3,3).norm()/_toolGain< _toolTolerance &&
		 std::fabs(error(6)) <_phiTolerance)
	{
		// std::cerr << "Great: " << error.transpose() << std::endl;
		return true;
	}
	else
	{
		return false;
	}
}


void QpSolverRCMCollision2::copyQpOASESVariables()
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
