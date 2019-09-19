#include "PublishSphericalTrocarFrames.h"
#include "Utils.h"

PublishSphericalTrocarFrames* PublishSphericalTrocarFrames::me = NULL;

PublishSphericalTrocarFrames::PublishSphericalTrocarFrames(ros::NodeHandle &n, double frequency): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency)
{
	me=this;
	_stop = false;


	// _offset << -0.5f, 0.0f, 0.0f;
	_offset << -0.33f, -0.316f, -0.015f;

	float sphereRadius = 0.13f;
	float medialDivision = 10.0f;
	float transversalDivision = 12.0f;

	float angle1 = M_PI/medialDivision;
	float angle2  = 2*M_PI/transversalDivision;

	// _trocarPosition.resize((medialDivision/2-1)*4+1-2*4,3);
	// _trocarOrientation.resize((medialDivision/2-1)*4+1-2*4,3);

	// int id = 0;
	// _trocarPosition.row(id) << 0.0f,0.0f,sphereRadius;
	// _trocarPosition.row(id)+=_offset.transpose();
	// _trocarOrientation.row(id) << 0.0f, 0.0f, 1.0f;
	// id++;
	// for(int k = 1; k <medialDivision/2; k+=2)
	// {
	// 	for(int m = 1 ; m < transversalDivision+1; m+=3)
	// 	{
	// 		_trocarPosition.row(id) << sphereRadius*cos(k*angle1)*cos((m-1)*angle2),
	// 		                           sphereRadius*cos(k*angle1)*sin((m-1)*angle2),
	// 		                           sphereRadius*sin(k*angle1);
 //           	_trocarPosition.row(id)+=_offset.transpose();
 //           	_trocarOrientation.row(id) = (_trocarPosition.row(id)-_offset.transpose()).normalized();
 //           	id++;
	// 	}
	// }

	Eigen::MatrixXf positions;
	Eigen::MatrixXf orientations;
	positions.resize((medialDivision/2-1)*transversalDivision+1,3);
	orientations.resize((medialDivision/2-1)*transversalDivision+1,3);

	int id = 0;
	positions.row(id) << 0.0f,0.0f,sphereRadius;
	positions.row(id)+=_offset.transpose();
	orientations.row(id) << 0.0f, 0.0f, 1.0f;
	id++;
	for(int k = 1; k <medialDivision/2; k+=1)
	{
		for(int m = 1 ; m < transversalDivision+1; m+=1)
		{
			positions.row(id) << sphereRadius*cos(k*angle1)*cos((m-1)*angle2),
			                           sphereRadius*cos(k*angle1)*sin((m-1)*angle2),
			                           sphereRadius*sin(k*angle1);
           	positions.row(id)+=_offset.transpose();
           	orientations.row(id) = (positions.row(id)-_offset.transpose()).normalized();
           	id++;
		}
	}
	std::vector<int> idConserved;
	idConserved.push_back(0);
	idConserved.push_back(28);
	idConserved.push_back(4);
	idConserved.push_back(25);

	_trocarPosition.resize(idConserved.size(),3);
	_trocarOrientation.resize(idConserved.size(),3);

	for(int k = 0; k < idConserved.size(); k++)
	{
		_trocarPosition.row(k) = positions.row(idConserved[k]);
		_trocarOrientation.row(k) = orientations.row(idConserved[k]);
	}



}

PublishSphericalTrocarFrames::~PublishSphericalTrocarFrames()
{
	me->_n.shutdown();
}

bool PublishSphericalTrocarFrames::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{
	//Subscriber definitions	
	signal(SIGINT,PublishSphericalTrocarFrames::stopNode);
	
	if (_n.ok()) 
	{   
		ros::spinOnce();
		ROS_INFO("The actor_world broadcast is about to start");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void PublishSphericalTrocarFrames::stopNode(int sig)
{
  me->_stop= true;
}

void PublishSphericalTrocarFrames::run()
{
  while (!_stop) 
  {	

		updateTf();

    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void PublishSphericalTrocarFrames::updateTf()
{

	Eigen::Vector3f temp;
	Eigen::Vector3f z;
	z << 0.0f,0.0f,1.0f;
	Eigen::Vector4f qd;
	for(int k = 0 ; k < _trocarPosition.rows(); k++)
	{
		tf::Vector3 origin(_trocarPosition(k,0),_trocarPosition(k,1),_trocarPosition(k,2));
  	_transform.setOrigin(origin);
  	qd = Utils<float>::rotationMatrixToQuaternion(Utils<float>::rodriguesRotation(z,_trocarOrientation.row(k).transpose()));
  	_transform.setRotation(tf::Quaternion(qd(1),qd(2),qd(3),qd(0)));
  	_br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world","t"+std::to_string(k)));
	}

}