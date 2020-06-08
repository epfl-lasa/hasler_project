#include "CvxgenSolverRCM.h"
#include "Utils.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

CvxgenSolverRCM::CvxgenSolverRCM():_nbTasks(7), _nbJoints(7), _nbSlacks(7), 
                                   _nbVariables(14), _nbConstraints(14),
                                   _rcmTolerance(1e-3), _toolTolerance(1e-3), _phiTolerance(1e-2)
{

	
	_J.resize(_nbTasks,_nbJoints);
	_damping.resize(_nbJoints);
	_slack.resize(_nbTasks);
	_dx.resize(_nbTasks);
	_jointLimits.resize(_nbJoints);
	_jointVelocityLimits.resize(_nbJoints);
	_slackLimits.resize(_nbJoints);
	_qUp.resize(_nbJoints);
	_qLow.resize(_nbJoints);
	_dqUp.resize(_nbJoints);
	_dqLow.resize(_nbJoints);
	_slackLow.resize(_nbSlacks);
	_slackUp.resize(_nbSlacks);
	_joints.resize(_nbJoints);

	_damping.setConstant(1.0f);
	_slack.setConstant(1000.0f);

	_rcmGain = 1.0f;
	_toolGain = 1.0f;

  _jointLimits(0) = 170.0f*M_PI/180.0f;
  _jointLimits(1) = 120.0f*M_PI/180.0f;
  _jointLimits(2) = 170.0f*M_PI/180.0f;
  _jointLimits(3) = 120.0f*M_PI/180.0f;
  _jointLimits(4) = 170.0f*M_PI/180.0f;
  _jointLimits(5) = 120.0f*M_PI/180.0f;
  _jointLimits(6) = 170.0f*M_PI/180.0f;

  _jointVelocityLimits(0) = 110.0f*M_PI/180.0f;
  _jointVelocityLimits(1) = 110.0f*M_PI/180.0f;
  _jointVelocityLimits(2) = 128.0f*M_PI/180.0f;
  _jointVelocityLimits(3) = 128.0f*M_PI/180.0f;
  _jointVelocityLimits(4) = 204.0f*M_PI/180.0f;
  _jointVelocityLimits(5) = 184.0f*M_PI/180.0f;
  _jointVelocityLimits(6) = 184.0f*M_PI/180.0f;	
  
  _slackLimits.setConstant(1.0f);

  _dqUp = _jointVelocityLimits;
  _dqLow = -_jointVelocityLimits;
	memcpy(params.dqlow, _dqLow.data(), _nbJoints*sizeof(double));
	memcpy(params.dqup, _dqUp.data(), _nbJoints*sizeof(double));

  _qUp = _jointLimits;
  _qLow = -_jointLimits;
	
	memcpy(params.qlow, _qLow.data(), _nbJoints*sizeof(double));
	memcpy(params.qup, _qUp.data(), _nbJoints*sizeof(double));

  _slackUp = _slackLimits;
  _slackLow = -_slackLimits;
	
	memcpy(params.slacklow, _slackLow.data(), _nbSlacks*sizeof(double));
	memcpy(params.slackup, _slackUp.data(), _nbSlacks*sizeof(double));



	memcpy(params.damping, _damping.data(), _nbJoints*sizeof(double));
	memcpy(params.slack, _slack.data(), _nbSlacks*sizeof(double));

  set_defaults();
  setup_indexing();
  settings.verbose = 0;
  settings.resid_tol = 1e-6;
  settings.eps = 1e-6;
	settings.max_iters = 1000;

}

CvxgenSolverRCM::~CvxgenSolverRCM()
{

}

bool CvxgenSolverRCM::step(Eigen::VectorXd &joints, Eigen::VectorXd joints0, Eigen::Vector3d xTrocar, 
	                         double toolOffset, Eigen::Vector3d xdTool, double phid, double dt, Eigen::Vector3d xRobotBasis)
{


  // // we need this, because QPOases changes these values
  // double max_time = _max_time;
  // int max_iters = _max_iters;
  int count = 0;

	Eigen::VectorXd jointVelocities(7);
	jointVelocities.setConstant(0.0f);

	// memcpy(params.q0, joints0.data(), _nbTasks*sizeof(double));
	params.dt[0] = dt;

	
	_joints = joints0;


	// std::cerr << joints << std::endl;
	// std::cerr << joints0 << std::endl;
  while(count<100)
  {
  	count++;

	  Eigen::Matrix4d Hfk;
	  Hfk = Utils<double>::getForwardKinematics(_joints);

	  Eigen::Vector3d xEE, xRCM, xTool;
	  Eigen::Matrix3d wRb;

	  xEE = Hfk.block(0,3,3,1)+xRobotBasis;
	  wRb = Hfk.block(0,0,3,3);

	  xRCM = xEE+(xTrocar-xEE).dot(wRb.col(2))*wRb.col(2);
	  xTool = xEE+toolOffset*wRb.col(2);

	  _dx << _rcmGain*(xTrocar-xRCM),_toolGain*(xdTool-xTool),phid-_joints(_nbJoints-1);
	  // _dx << (xdTool-xTrocar).normalized()-wRb.col(2),_toolGain*(xdTool-xTool),phid-_joints(_nbJoints-1);
	  
	  if(count == 1)
	  {
  	  std::cerr << "dx ini: " << _dx.transpose() << std::endl;
	  }
	  if(checkConvergence(_dx))
	  {
	  	break;
	  }

	  Eigen::MatrixXd Jrcm(3,_nbJoints), Jtool(3,_nbJoints);
	  Jrcm = jacobianRCM(_joints.cast<float>(),xTrocar.cast<float>()).cast<double>();
	  // Jrcm = jacobianZ(_joints.cast<float>()).cast<double>();
	  Jtool = jacobianTool(_joints.cast<float>(),toolOffset).cast<double>();

	  _J.setConstant(0.0f);
	  _J.block(0,0,3,7) = Jrcm;
	  _J.block(3,0,3,7) = Jtool;
	  _J(_nbTasks-1,_nbJoints-1) = 1.0f;
	  // std::cerr << J << std::endl;

	  copyCvxgenVariables();

	  solve();



   	for (int k = 0; k < _nbJoints; k++)
   	{
      jointVelocities(k) = vars.dq[k];
		}

  	_joints += dt*jointVelocities;
		// std::cerr << "jv: " << jointVelocities.transpose() << std::endl;
		// std::cerr << "j: " << joints.transpose() << std::endl;
		// std::cerr << "error: " << _dx.norm() << std::endl;

  }

  std::cerr << "dx end: " << _dx.transpose() << std::endl;
  double sum = (_joints-joints0).array().abs().sum();
  double maxDiff = (_joints-joints0).array().abs().maxCoeff()*180.0f/M_PI;
  // std::cerr << (joints-joints0).transpose() << std::endl;
  std::cerr <<"Nb loops: " << count << " Max joints diff: " << maxDiff << " Sum: " << sum*180/M_PI <<std::endl;

  joints = _joints;

  if(!checkConvergence(_dx))
  {
  	joints = joints0;
  	return false;
  }
  return true;	
}



void	CvxgenSolverRCM::setParameters(double rcmGain, double toolGain, Eigen::VectorXd slackGains)
{
	_rcmGain = rcmGain;
	_toolGain = toolGain;
	_slack = slackGains;
} 



bool CvxgenSolverRCM::checkConvergence(Eigen::VectorXd error)
{
	if(error.segment(0,3).norm()/_rcmGain < _rcmTolerance && 
		 error.segment(3,3).norm()/_toolGain < _toolTolerance &&
		 std::fabs(error(6)) < _phiTolerance)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void CvxgenSolverRCM::copyCvxgenVariables()
{

    // set params


  for (int r = 0; r < _nbTasks; r++)
  {
    for (int c = 0; c < _nbJoints; c++)
    {
      params.J[r+1][c] = _J(r,c);

    }
	}

	memcpy(params.q, _joints.data(), _nbJoints*sizeof(double)); // we set qref to zero so that we minimize the dq
	memcpy(params.dx, _dx.data(), _nbTasks*sizeof(double));
}

