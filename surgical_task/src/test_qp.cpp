#include "Eigen/Eigen"
#include <iostream>
#include "Utils.h"
#include "ros/ros.h"
#include "qpSolver.h"
#include "qpSolver2.h"
#include "qpSolver3.h"
#include "rcmKinematics.h"
// #include "ipoptSolver4.h"
#include "QpSolverRCM.h"
#include "QpSolverRCM2.h"
#include <chrono>
// Eigen::Matrix4f getDHMatrix(float a, float alpha, float d, float theta)
// {
// 	Eigen::Matrix4f H;
// 	H(0,0) = std::cos(theta);
// 	H(0,1) = -std::cos(alpha)*std::sin(theta);
// 	H(0,2) = std::sin(alpha)*std::sin(theta);
// 	H(0,3) = a*std::cos(theta);

// 	H(1,0) = std::sin(theta);
// 	H(1,1) = std::cos(alpha)*std::cos(theta);
// 	H(1,2) = -std::sin(alpha)*std::cos(theta);
// 	H(1,3) = a*std::sin(theta);

// 	H(2,0) = 0.0f;
// 	H(2,1) = std::sin(alpha);
// 	H(2,2) = std::cos(alpha);
// 	H(2,3) = d;

// 	H(3,0) = 0.0f;
// 	H(3,1) = 0.0f;
// 	H(3,2) = 0.0f;
// 	H(3,3) = 1.0f;

// 	return H;
// }


int main(int argc, char **argv)
{

	ros::init(argc, argv, "test_qp");
  

	
	Eigen::VectorXf joints0(7), joints(7), joints2(7), joints3(7), joints4(7);
	Eigen::Matrix4f H;
	Eigen::Matrix3f wRb;
	Eigen::Vector3f x, xRCM, x0, xRCM0, xTool, xTool0, xdTool;
	Eigen::Vector3f xTrocar;

	// joints0 << -0.0906f, 0.983f, 0.222f, -1.28f, 0.843f, 1.3301f, 0.423f;
	// xTrocar << -0.526f, -0.373f, 0.0902f;
	// xdTool << -0.51f, -0.427f, 0.043f;
	
	joints0 << 0.0764f, 0.612f, 0.193f, -1.5076, 0.5990f, 1.025f, 0.159f;
	xTrocar << -0.526f, -0.373f, 0.0902f;
	xdTool << -0.5225f, -0.399f, 0.04098f;

	joints0 << 0.363f, 0.46526f, 0.175589f, -1.62057, 0.135874f, 0.747f, 1.05254f;
	xTrocar << -0.526f, -0.373f, 0.0902f;
	xdTool << -0.5479f, -0.392f, 0.022f;


	joints0 << 0.7218f, 0.8203f, 0.1617f, -1.04878f, -0.40745f, 1.37066, 2.28386f;
	xTrocar << -0.526f, -0.373f, 0.0902f;
	xdTool << -0.5387f, -0.362f, 0.004f;

	// joints0 << -0.0487f, 0.7783f, -0.005f, -1.77358f, 1.32802f, 1.20f, 0.52f;
	// xTrocar << -0.526f, -0.373f, 0.0902f;
	// xdTool << -0.525f, -0.41f, 0.06555f;

	
	// joints0 << 0.09808, 0.5485f, 0.1216f, -1.52003f, 0.5608f, 1.0761, -0.09742f;
	// xTrocar << -0.526f, -0.373f, 0.0902f;
	// xdTool << -0.52108f, -0.3572f, 0.0521f;

	Eigen::Vector3f xRobotBase;
	xRobotBase << 0.0f, 0.5f, 0.0f;
	joints0 << 0.269433,  1.26491, 0.416071, -0.22476, 0.474047,  2.09547,  2.01696;
	xTrocar <<  -0.447232, 0.0650098,  0.204723;
	xdTool << -0.421002, 0.0400123, 0.18099;

	joints0 << 0.284272, 1.25149,   0.41614, -0.223621,  0.443943,   2.09506 ,  1.97214;
	xdTool << -0.420252, 0.0403195,  0.178929;

	joints0 << 1.29922,  1.20598,  -1.9352, -1.38653,  1.63506,  2.03178,  2.96838;
	xdTool << -0.446677, 0.0532827,   0.15793+0.02;

	joints0 << 0.625966, 0.921047, 0.1725, -0.911323, 0.0419271, 1.5811,  0.716837;
	xdTool << -0.384029,  0.0730291, 0.00124362;

	// joints0 << 0.538375,   1.10245,  0.158694, -0.234481,  0.234326,   2.0944,  0.205083;
	// xdTool << -0.394503, 0.0613606,  0.153293;


	float phid = joints0(6);

	float toolOffset = 0.40f;

	float dt = 0.1f;


	H = Utils<float>::getForwardKinematics(joints0);
	x0 = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM0 = x0+(xTrocar-x0).dot(wRb.col(2))*wRb.col(2);
	xTool0 = x0+toolOffset*wRb.col(2);

	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;
	std::cerr << "Before: x0: " << x0.transpose() << std::endl;
	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: xdTool: " << xdTool.transpose() << std::endl;

	std::cerr << "WITH QP" << std::endl;
 	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	joints = qpSolver(joints0,xTrocar,toolOffset,xdTool,phid,dt,xRobotBase);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
	// std::cerr << joints.transpose() << std::endl;
	H = Utils<float>::getForwardKinematics(joints);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);

	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;

	std::cerr << "After: joints:  " << joints.transpose() << std::endl;
	std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	std::cerr << "After: joint diff: " << (joints-joints0).norm() << std::endl;

	std::cerr << "WITH ITERATIVE SOLUTION" << std::endl;
  	begin = std::chrono::steady_clock::now();
	joints2 = rcmInverseKinematics(joints0,xTrocar,toolOffset,xdTool,phid, xRobotBase);
	end = std::chrono::steady_clock::now();

  	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;

	H = Utils<float>::getForwardKinematics(joints2);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);

	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;

	std::cerr << "After: joints:  " << joints2.transpose() << std::endl;
	std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	std::cerr << "After: joint diff: " << (joints2-joints0).norm() << std::endl;


	std::cerr << "WITH QP2" << std::endl;
 	bool result;
 	begin = std::chrono::steady_clock::now();
	result = qpSolver2(joints3,joints0,xTrocar,toolOffset,xdTool,phid,dt, xRobotBase);
	end = std::chrono::steady_clock::now();
 	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
	std::cerr <<(int) result << std::endl;
	H = Utils<float>::getForwardKinematics(joints3);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);
	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;
	std::cerr << "After: joints:  " << joints3.transpose() << std::endl;
	std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	std::cerr << "After: joint diff: " << (joints3-joints0).norm() << std::endl;


	std::cerr << "WITH QP2 class" << std::endl;
 	QpSolverRCM qp;
 	begin = std::chrono::steady_clock::now();
	result = qp.step(joints4,joints0,xTrocar,toolOffset,xdTool,phid,dt, xRobotBase);
	end = std::chrono::steady_clock::now();
 	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
	std::cerr <<(int) result << std::endl;
	H = Utils<float>::getForwardKinematics(joints4);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);
	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;
	std::cerr << "After: joints:  " << joints4.transpose() << std::endl;
	std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	std::cerr << "After: joint diff: " << (joints4-joints0).norm() << std::endl;


	std::cerr << "WITH QP2 class 2" << std::endl;
 	QpSolverRCM2 qp2;
 	begin = std::chrono::steady_clock::now();
	result = qp2.step(joints4,joints0,xTrocar,toolOffset,xdTool,phid,dt, xRobotBase);
	end = std::chrono::steady_clock::now();
 	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
	std::cerr <<(int) result << std::endl;
	H = Utils<float>::getForwardKinematics(joints4);
	x = H.block(0,3,3,1)+xRobotBase;
	wRb = H.block(0,0,3,3);
	xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	xTool = x+toolOffset*wRb.col(2);
	std::cerr << "Before: xRCM0: " << xRCM0.transpose() << std::endl;
	std::cerr << "Before: xTool0: " << xTool0.transpose() << std::endl;
	std::cerr << "Before: joints0:  " << joints0.transpose() << std::endl;
	std::cerr << "After: joints:  " << joints4.transpose() << std::endl;
	std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	std::cerr << "After: joint diff: " << (joints4-joints0).norm() << std::endl;


 // 	begin = std::chrono::steady_clock::now();
	// joints4 = qpSolver3(joints0,xTrocar,toolOffset,xdTool,phid,dt);
	// end = std::chrono::steady_clock::now();
 // 	std::cerr << "Elasped time in [micro s]: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << std::endl;
	// std::cerr << joints4.transpose() << std::endl;
	// H = Utils<float>::getForwardKinematics(joints4);
	// x = H.block(0,3,3,1);
	// wRb = H.block(0,0,3,3);
	// xRCM = x+(xTrocar-x).dot(wRb.col(2))*wRb.col(2);
	// xTool = x+toolOffset*wRb.col(2);
	// std::cerr << "WITH QP3" << std::endl;
	// std::cerr << "After: joints:  " << joints4.transpose() << std::endl;
	// std::cerr << "After: x: " << x.transpose() << std::endl;
	// std::cerr << "After: xRCM: " << xRCM.transpose() << std::endl;
	// std::cerr << "After: xTrocar: " << xTrocar.transpose() << std::endl;
	// std::cerr << "After: xTool: " << xTool.transpose() << std::endl;
	// std::cerr << "After: xdTool: " << xdTool.transpose() << std::endl;
	// std::cerr << "After: joint diff: " << (joints-joints0).norm() << std::endl;

	return 0;
}