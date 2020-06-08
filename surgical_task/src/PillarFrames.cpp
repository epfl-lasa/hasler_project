#include "PillarFrames.h"
#include "Utils.h"

PillarFrames* PillarFrames::me = NULL;

PillarFrames::PillarFrames(ros::NodeHandle &n, double frequency, int publishTransforms): 
_nh(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_publishTransforms(publishTransforms)
{

	me=this;
	_stop = false;

	_pillarPosition.resize(16,3);

	_pillarPosition << 55,	55,	44,
										 55,	18.3333333333,	44,
										 55,	-18.3333333333,	44,
										 55,	-55,	44,
										 18.3333333333,	55,	44,
										 18.3333333333,	18.3333333333,	44,
										 18.3333333333,	-18.3333333333,	44,
										 18.3333333333,	-55,	44,
										 -18.3333333333,	55,	44,
										 -18.3333333333,	18.3333333333,	44,
										 -18.3333333333,	-18.3333333333,	44,
										 -18.3333333333,	-55,	44,
										 -55,	55,	44,
										 -55,	18.3333333333,	44,
										 -55,	-18.3333333333,	44,
										 -55,	-55,	44;

	 _pillarPosition *= 1e-3f;


	_msgFrames.data.resize(_pillarPosition.size());
  _msgFrames.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _msgFrames.layout.dim.push_back(std_msgs::MultiArrayDimension());
  _msgFrames.layout.dim[0].label = "nb_frames";
  _msgFrames.layout.dim[1].label = "dim";
  _msgFrames.layout.dim[0].size = _pillarPosition.rows();
  _msgFrames.layout.dim[1].size = _pillarPosition.cols();

  _firstPillarsCenterFrame = false;


}


PillarFrames::~PillarFrames()
{
	me->_nh.shutdown();
}


bool PillarFrames::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

  _pubFrames = _nh.advertise<std_msgs::Float32MultiArray>("/pillar_frames", 1);


	//Subscriber definitions	
	signal(SIGINT,PillarFrames::stopNode);
	
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


void PillarFrames::stopNode(int sig)
{
  me->_stop= true;
}

void PillarFrames::run()
{
  while (!_stop) 
  {	
  	if(_firstPillarsCenterFrame)
  	{
	  	if(_publishTransforms)
	  	{
				updateTf();
	  	}

	  	_pubFrames.publish(_msgFrames);  		
  	}
  	else
  	{
  		receiveFrames();
  	}


    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void PillarFrames::receiveFrames()
{
  try
  { 

    _lr.waitForTransform("/world", "/pillars_base_link", ros::Time(0), ros::Duration(3.0));
    _lr.lookupTransform("/world", "/pillars_base_link", ros::Time(0), _transform); 
    _pillarsCenter << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
    // _qRobotBaseOrigin[k] << _transform.getRotation().w(), _transform.getRotation().x(), _transform.getRotation().y(), _transform.getRotation().z();
    _firstPillarsCenterFrame = true;
    std::cerr << "[PillarFrames]: origin received: " << _pillarsCenter.transpose() << std::endl;

  	for(int k = 0; k < _pillarPosition.rows(); k++)
		{
			for(int m = 0; m < _pillarPosition.cols(); m++)
			{
				_msgFrames.data[k*_pillarPosition.cols()+m] = _pillarPosition(k,m)+_pillarsCenter(m);
			}
		}
  } 
  catch (tf::TransformException ex)
  {
  }
}



void PillarFrames::updateTf()
{

	Eigen::Vector3f temp;

	for(int k = 0 ; k < _pillarPosition.rows(); k++)
	{
		temp = _pillarPosition.row(k).transpose()+_pillarsCenter;
		tf::Vector3 origin(temp(0),temp(1),temp(2));
  	_transform.setOrigin(origin);
  	_transform.setRotation(tf::Quaternion(0,0,0,1));
  	_br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world","p" + std::to_string(k)));
	}

}