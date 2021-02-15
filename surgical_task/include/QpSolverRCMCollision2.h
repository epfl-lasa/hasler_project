#ifndef __QP_SOLVER_RCM_COLLISION_2_H__
#define __QP_SOLVER_RCM_COLLISION_2_H__

#include "Eigen/Eigen"
#include <qpOASES.hpp>
#include "JacobianDefinitions.h"
#include "Utils.h"


USING_NAMESPACE_QPOASES

class QpSolverRCMCollision2
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

    Eigen::VectorXf _jointMax, _jointMin, _jointVelocitiesLimits;

    SQProblem* _sqp;

	Utils<float>::ROBOT_ID _robotID;

    static QpSolverRCMCollision2* me;

	public:
		QpSolverRCMCollision2();

		~QpSolverRCMCollision2();

		void setRobot(Utils<float>::ROBOT_ID robotID);

		bool step(Eigen::VectorXf &joints, Eigen::VectorXf joints0, 
	              Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f vdTool, float phid, float dt,
	              Eigen::Vector3f xRobotBasis = Eigen::Vector3f::Zero(), Eigen::Matrix3f wRRobotBasis = Eigen::Matrix3f::Identity(), float depthGain = 1.0f,
				  Eigen::Vector3f obstacleDir = Eigen::Vector3f::Zero(), float obstacleDistance = 0.0f);


		void setParameters(float rcmGain, float toolGain, Eigen::VectorXf slackGains,
			                 Eigen::VectorXf slackLimits);	
	
	private:
		bool checkConvergence(Eigen::VectorXf error);

		void copyQpOASESVariables();

};
#endif  // __QP_SOLVER_RCM_COLLISION_2_H__/
