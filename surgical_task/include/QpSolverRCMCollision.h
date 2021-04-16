#ifndef __QP_SOLVER_RCM_COLLISION_H__
#define __QP_SOLVER_RCM_COLLISION_H__

#include "Eigen/Eigen"
#include <qpOASES.hpp>
#include "JacobianDefinitions.h"
#include "Utils.h"


USING_NAMESPACE_QPOASES

class QpSolverRCMCollision
{

	public:
		struct Result
		{
			bool res;
			bool eeCollisionConstraintActive;
			bool toolCollisionConstraintActive;
			bool workspaceCollisionConstraintActive;
		};

	private:
	
		//!ros variables

		const int _nbVariables;
		const int _nbJoints;
		int _nbConstraints;
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
  
  	bool _debug;  
    bool _first;
    float _rcmGain;
    float _toolGain;
    Eigen::VectorXf _slackGains, _slackLimits;

    Eigen::VectorXf _jointMax, _jointMin, _jointVelocitiesLimits, _eeVelocityLimits, _jointVelocities;

    SQProblem* _sqp;

		Utils<float>::ROBOT_ID _robotID;
		
		bool _enableEECollisionAvoidance;
		bool _enableToolCollisionAvoidance;
		bool _enableWorkspaceCollisionAvoidance;
		int _idEECollisionConstraint;
		int _idToolCollisionConstraint;
		int _idWorkspaceCollisionConstraint;

		float _eeSafetyCollisionDistance;
		float _toolSafetyCollisionDistance;
    float _eeLinearVelocityLimit;
    float _eeAngularVelocityLimit;


		Eigen::Vector3f _workspaceMinOffset;
		Eigen::Vector3f _workspaceMaxOffset;

    static QpSolverRCMCollision* me;

	public:
		QpSolverRCMCollision(float eeLinearVelocityLimit = 0.25f, float eeAngularVelocityLimit = 1.5f, bool enableEECollisionAvoidance = false, float eeSafetyCollisionDistance = 0.0f, 
			                   bool enableToolCollisionAvoidance = false, float toolSafetyCollisionDistance = 0.0f,
			                   bool enableWorkspaceCollisionAvoidance = false, Eigen::Vector3f workspaceMinOffset = Eigen::Vector3f::Zero(), 
			                   Eigen::Vector3f workspaceMaxOffset = Eigen::Vector3f::Zero());

		~QpSolverRCMCollision();

		void setRobot(Utils<float>::ROBOT_ID robotID);

		Result step(Eigen::VectorXf &joints, Eigen::VectorXf joints0, Eigen::VectorXf currentJoints, Eigen::Vector3f xTrocar, float toolOffset, Eigen::Vector3f vdTool, float phid,
		            float dt, Eigen::Vector3f xRobotBasis = Eigen::Vector3f::Zero(), Eigen::Matrix3f wRRobotBasis = Eigen::Matrix3f::Identity(), 
	                float depthGain = 1.0f, Eigen::Vector3f rEEObstacle = Eigen::Vector3f::Zero(), float dEEObstacle = 0.0f, Eigen::Vector3f eeCollisionOffset = Eigen::Vector3f::Zero(),
	                Eigen::Vector3f rToolObstacle = Eigen::Vector3f::Zero(), float dToolObstacle = 0.0f, Eigen::Vector3f toolCollisionOffset = Eigen::Vector3f::Zero(), 
	                bool useWorkspaceLimits = false, Eigen::Vector3f currentOffset = Eigen::Vector3f::Zero());


		void setParameters(float rcmGain, float toolGain, Eigen::VectorXf slackGains,
			                 Eigen::VectorXf slackLimits);	
	
	private:
		bool checkConvergence(Eigen::VectorXf error);

		void copyQpOASESVariables();

};
#endif  // __QP_SOLVER_RCM_COLLISION_H__/
