#ifndef __CVXGEN_SOLVER_RCM_H__
#define __CVXGEN_SOLVER_RCM_H__
#ifdef __cplusplus
extern "C"
{
#endif
    #include "cvxgen/solver.h"
#ifdef __cplusplus
}
#endif

#include "Eigen/Eigen"
#include "JacobianDefinitions.h"


class CvxgenSolverRCM
{
	private:
	
	//!ros variables

	const int _nbVariables;
	const int _nbJoints;
	const int _nbConstraints;
	const int _nbTasks;
	const int _nbSlacks;

	const float _rcmTolerance;
	const float _toolTolerance;
	const float _phiTolerance;

	Eigen::MatrixXd _J;
	Eigen::VectorXd _damping;
	Eigen::VectorXd _slack;
	Eigen::VectorXd _dx;
	Eigen::VectorXd _jointLimits;
	Eigen::VectorXd _jointVelocityLimits;
	Eigen::VectorXd _slackLimits;
	Eigen::VectorXd _qUp;
	Eigen::VectorXd _qLow;
	Eigen::VectorXd _dqUp;
	Eigen::VectorXd _dqLow;
	Eigen::VectorXd _slackLow;
	Eigen::VectorXd _slackUp;
	Eigen::VectorXd _joints;


	//!subscribers and publishers declaration    
    //!boolean variables
    
    
    bool _first;
    double _rcmGain;
    double _toolGain;

    static CvxgenSolverRCM* me;

	public:
		CvxgenSolverRCM();

		~CvxgenSolverRCM();

		bool step(Eigen::VectorXd &joints, Eigen::VectorXd joints0,
	  	        Eigen::Vector3d xTrocar, double toolOffset, Eigen::Vector3d xdTool, double phid, double dt, Eigen::Vector3d xRobotBasis = Eigen::Vector3d::Zero());

		void setParameters(double rcmGain, double toolGain, Eigen::VectorXd slackGains);	
	
	private:
		bool checkConvergence(Eigen::VectorXd error);

		void copyCvxgenVariables();

};
#endif  // __CVXGEN_SOLVER_RCM_H__
