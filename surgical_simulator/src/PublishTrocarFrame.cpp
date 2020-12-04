#include "PublishTrocarFrame.h"


PublishTrocarFrame* PublishTrocarFrame::me = NULL;

PublishTrocarFrame::PublishTrocarFrame(ros::NodeHandle &n, double frequency,std::string name, Eigen::Vector3f trocarOffset): 
_n(n),
_loopRate(frequency),
_dt(1.0f/frequency),
_frameName(name),
_trocarOffset0(trocarOffset)
{
	me=this;
	_stop = false;
	_getTorsoFrame = false;	
	_trocarOffset.setConstant(0.0f);
	_torsoFrameOrigin.setConstant(0.0f);
}

PublishTrocarFrame::~PublishTrocarFrame()
{
	me->_n.shutdown();
}

bool PublishTrocarFrame::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

  	_dynRecCallback = boost::bind(&PublishTrocarFrame::dynamicReconfigureCallback, this, _1, _2);
  	_dynRecServer.setCallback(_dynRecCallback);

	//Subscriber definitions	
	signal(SIGINT,PublishTrocarFrame::stopNode);
	
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


void PublishTrocarFrame::stopNode(int sig)
{
    me->_stop= true;
}

void PublishTrocarFrame::run()
{
  while (!_stop) 
  {	
		// if(!_getTorsoFrame)
		// {	
	  // 	try
			// {	
   //    	_lr.lookupTransform("/world", "/torso_upper_base_link",ros::Time::now(), _transform);
   //    	_torsoFrameOrigin << _transform.getOrigin().x(), _transform.getOrigin().y(), _transform.getOrigin().z();
   //    	std::cerr << _transform.getOrigin().x() << " " << _transform.getOrigin().y() << " " << _transform.getOrigin().z() << std::endl;
   //    	_getTorsoFrame = true;
	  //   } 
	 //    catch (tf::TransformException ex)
	 //    {
  //        // ROS_ERROR("%s",ex.what());
  //        // ros::Duration(1.0).sleep();
		// 	}
		// }
		// else
		{
			updateTf();
		}

    ros::spinOnce();
    _loopRate.sleep();
  }

  ros::spinOnce();
  _loopRate.sleep();
  
  ros::shutdown();
}


void PublishTrocarFrame::updateTf()
{

	Eigen::Vector3f temp;
	temp = _torsoFrameOrigin+_trocarOffset0+_trocarOffset;
	tf::Vector3 origin(temp(0),temp(1),temp(2));
  _transform.setOrigin(origin);
  _transform.setRotation(tf::Quaternion(0,0,0,1));
  _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world",_frameName+"_trocar_frame"));

}


void PublishTrocarFrame::dynamicReconfigureCallback(surgical_simulator::publishTrocarFrame_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request. Updatig the parameters ...");
  _trocarOffset << config.xOffset, config.yOffset, config.zOffset;
}
