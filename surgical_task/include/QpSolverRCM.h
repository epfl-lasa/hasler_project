#ifndef __QP_SOLVER_RCM_H__
#define __QP_SOLVER_RCM_H__

#include "Eigen/Eigen"
#include <qpOASES.hpp>
#include "JacobianDefinitions.h"

#define RCM_TOLERANCE 1e-3f
#define TOOL_TOLERANCE 1e-3f
#define PHI_TOLERANCE 1e-2f

USING_NAMESPACE_QPOASES

class QpSolverRCM
{
	private:
	
	//!ros variables

	const int _nbVariables;
	const int _nbJoints;
	const int _nbConstraints;
	const int _nbTasks;
	const int _nbSlacks;

	Eigen::MatrixXf _H;
	Eigen::MatrixXf _A;
	Eigen::VectorXf _g;
	Eigen::VectorXf _lb;
	Eigen::VectorXf _ub;
	Eigen::VectorXf _lbA;
	Eigen::VectorXf _ubA;

	qpOASES::real_t* _H_qp;
	qpOASES::real_t* _A_qp;
	qpOASES::real_t* _g_qp;
	qpOASES::real_t* _lb_qp;
	qpOASES::real_t* _ub_qp;
	qpOASES::real_t* _lbA_qp;
	qpOASES::real_t* _ubA_qp;


	//!subscribers and publishers declaration    
    //!boolean variables
    
    bool _first;
    float _rcmGain;
    float _toolGain;
    Eigen::VectorXf _slackGains, _slackLimits;

    Eigen::VectorXf _jointLimits, _jointVelocitiesLimits;

    SQProblem* _sqp;

    static QpSolverRCM* me;

	public:
		QpSolverRCM();

		~QpSolverRCM();

		bool step(Eigen::VectorXf &joints, Eigen::VectorXf joints0,
	  	          Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt,
	  	          Eigen::Vector3f xRobotBasis = Eigen::Vector3f::Zero());

		void setParameters(float rcmGain, float toolGain, Eigen::VectorXf slackGains,
			                 Eigen::VectorXf slackLimits);	
	
	private:
		bool checkConvergence(Eigen::VectorXf error);

		void copyQpOASESVariables();

};
#endif  // __QP_SOLVER_RCM_H__
