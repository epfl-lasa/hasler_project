#include "SphericalTrocarFrames.h"
#include "Utils.h"

SphericalTrocarFrames* SphericalTrocarFrames::me = NULL;

SphericalTrocarFrames::SphericalTrocarFrames(ros::NodeHandle &n, double frequency, 
	                                         Eigen::Vector3f sphereCenter, int publishTransforms): 
_nh(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_sphereCenter(sphereCenter),
_publishTransforms(publishTransforms)
{

	me=this;
	_stop = false;

	float sphereRadius = 0.132f;
	int medialDivision = 4;
	int transversalDivision = 6;

	float angle1 = 75.0f*M_PI/(180.0f*4.0f);
	float angle1_offset = 15.0f*M_PI/180.0f;
	float angle2  = M_PI/5.0f;

	_trocarPosition.resize(medialDivision*transversalDivision+1,3);
	_trocarOrientation.resize(medialDivision*transversalDivision+1,3);

	int id = 0;
	_trocarPosition.row(id) << 0.0f,0.0f,sphereRadius;
	_trocarPosition.row(id) += _sphereCenter.transpose();
	_trocarOrientation.row(id) << 0.0f, 0.0f, 1.0f;
	id++;

	Eigen::Matrix3f R;
	R.setIdentity();
/*	R << -1.0f, 0.0f ,0.0f,
	     0.0f, -1.0f, 0.0f,
	     0.0f, 0.0f, 1.0f;*/

	for(int k = 1; k <medialDivision+1; k+=1)
	{
		for(int m = 1 ; m < transversalDivision+1; m+=1)
		{
			_trocarPosition.row(id) << -sphereRadius*cos((k-1)*angle1+angle1_offset)*sin((m-1)*angle2),
			                           sphereRadius*cos((k-1)*angle1+angle1_offset)*cos((m-1)*angle2),
			                           sphereRadius*sin((k-1)*angle1+angle1_offset);
           	_trocarPosition.row(id) = (R*_trocarPosition.row(id).transpose()).transpose(); 
			_trocarPosition.row(id) += _sphereCenter.transpose();
			_trocarOrientation.row(id) = (_trocarPosition.row(id)-_sphereCenter.transpose()).normalized();
			id++;
		}
	}

	Eigen::MatrixXf data;
	data.resize(_trocarPosition.rows(),_trocarPosition.cols()+_trocarOrientation.cols());
	data.block(0, 0, _trocarPosition.rows(), _trocarPosition.cols()) = _trocarPosition;
	data.block(0,_trocarPosition.cols(),_trocarOrientation.rows(),_trocarOrientation.cols()) = _trocarOrientation;

	_msgFrames.data.resize(data.size());
  _msgFrames.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _msgFrames.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _msgFrames.layout.dim[0].label = "nb_frames";
  _msgFrames.layout.dim[1].label = "dim";
  _msgFrames.layout.dim[0].size = data.rows();
  _msgFrames.layout.dim[1].size = data.cols();

	for(int k = 0; k < data.rows(); k++)
	{
		for(int m = 0; m < data.cols(); m++)
		{
			_msgFrames.data[k*data.cols()+m] = data(k,m);
		}
	}
}


SphericalTrocarFrames::~SphericalTrocarFrames()
{
	me->_nh.shutdown();
}


bool SphericalTrocarFrames::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{


  _pubFrames = _nh.advertise<std_msgs::Float32MultiArray>("/spherical_trocar_frames", 1);


	//Subscriber definitions	
	signal(SIGINT,SphericalTrocarFrames::stopNode);
	
	if (_nh.ok()) 
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


void SphericalTrocarFrames::stopNode(int sig)
{
  me->_stop= true;
}

void SphericalTrocarFrames::run()
{
  while (!_stop) 
  {	

  	if(_publishTransforms)
  	{
			updateTf();
  	}

  	_pubFrames.publish(_msgFrames);

    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void SphericalTrocarFrames::updateTf()
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
  	_br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world","f" + std::to_string(k)));
	}

}