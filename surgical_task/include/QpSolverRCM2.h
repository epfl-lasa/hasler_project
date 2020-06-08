#ifndef __QP_SOLVER_RCM_2_H__
#define __QP_SOLVER_RCM_2_H__

#include "Eigen/Eigen"
#include <qpOASES.hpp>
#include "JacobianDefinitions.h"

#define NB_JOINTS 7
#define NB_TASKS_2 7
#define NB_SLACKS_2 7
#define NB_VARIABLES_2 NB_JOINTS+NB_SLACKS_2+1
#define NB_CONSTRAINTS_2 NB_TASKS_2+2*NB_JOINTS
#define RCM_TOLERANCE 5e-3f
#define TOOL_TOLERANCE 1e-3f
#define PHI_TOLERANCE 1e-2f

USING_NAMESPACE_QPOASES

class QpSolverRCM2
{
	private:
	
	//!ros variables

	Eigen::Matrix<float,NB_VARIABLES_2,NB_VARIABLES_2> _H;
	Eigen::Matrix<float,NB_CONSTRAINTS_2,NB_VARIABLES_2> _A;
	Eigen::Matrix<float,NB_VARIABLES_2,1> _g;
	Eigen::Matrix<float,NB_VARIABLES_2,1> _lb;
	Eigen::Matrix<float,NB_VARIABLES_2,1> _ub;
	Eigen::Matrix<float,NB_CONSTRAINTS_2,1> _lbA;
	Eigen::Matrix<float,NB_CONSTRAINTS_2,1> _ubA;

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
    Eigen::Matrix<float,NB_SLACKS_2,1> _slackGains, _slackLimits;

    Eigen::Matrix<float,NB_JOINTS,1> _jointLimits, _jointVelocitiesLimits;

    SQProblem* _sqp;

    static QpSolverRCM2* me;

	public:
		QpSolverRCM2();

		~QpSolverRCM2();

		bool step(Eigen::VectorXf &joints, Eigen::Matrix<float,NB_JOINTS,1> joints0,
	  	          Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f xdTool, float phid, float dt,
	  	          Eigen::Vector3f xRobotBasis = Eigen::Vector3f::Zero());

		void setParameters(float rcmGain, float toolGain, Eigen::Matrix<float,NB_SLACKS_2,1> slackGains,
			                 Eigen::Matrix<float,NB_SLACKS_2,1> slackLimits);	
	
	private:
		bool checkConvergence(Eigen::Matrix<float,NB_TASKS_2,1> error);

		void copyQpOASESVariables();

};
#endif  // __QP_SOLVER_RCM_2_H__
