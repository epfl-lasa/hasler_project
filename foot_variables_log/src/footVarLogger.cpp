#include "footVarLogger.h"
#include "ros/package.h"
#include "../../5_axis_platform/lib/platform/src/definitions.h"
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"
#include "FootPlatformModel.h"
#include "geometry_msgs/WrenchStamped.h"

#define ListofAxes(enumeration, names) names,
char const *Axis_names[]{
	AXES};
#undef ListofAxes


char const *Platform_Names[]{"none", "right", "left"};

footVarLogger* footVarLogger::me = NULL;


footVarLogger::footVarLogger(ros::NodeHandle &n_1, double frequency,footVarLogger::Platform_Name platform_id, std::string filename_): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_dt(1.0f/frequency),
_filename(filename_)
{

	me=this;
	_stop = false;
	_flagOutputMessageReceived=false;
	_flagPlatformOutCommStarted=false;
	_flagPositionOnlyPublished=false;
	_ftSensorWrenchBiasOK=false;

    _ftSensorWrench.setConstant(0.0f);

	_gravity << 0.0f, 0.0f, -9.80665f;
	_footComPositionFromSensor << 0.0f,0.05f,0.02f; //![m]
	_footOffsetFromEE = 0.0f;
	_footMass = 0.375f; //! [kg]
	_ftSensorWrenchCount = 0;

	_filteredForceGain=0.5;
	_ftSensorFilteredWrench.setConstant(0.0f);

    _footTipPosition.setConstant(0.0f);
    _footTipOrientation.setIdentity();
	// _footTipQuaternion.setConstant(0.0f);
    // _xdFootTip.setConstant(0.0f);
	
	_footOffset << 0.0f,0.0f,10.0f,0.0f,0.0f;

  		Eigen::Matrix<float,NB_AXIS,1> temp;
  		temp << WS_LIMITS[X],WS_LIMITS[Y],0.0f,0.0f,0.0f;
  		_H0 = FootPlatformModel::forwardKinematics(temp,true);
    
	for (int k=0; k<NB_AXIS; k++)
	{       
		//Platform

		_platform_position[k]=0.0f;
		_platform_speed[k]=0.0f;
		_platform_effortD[k]=0.0f;
		_platform_effortM[k]=0.0f;
		
	}	
		_platform_controllerType=TORQUE_ONLY;
		_platform_machineState=EMERGENCY;
		_platform_id=0;
		//ROS
		for (int k=0; k<NB_AXIS; k++)
	{

	}

}
footVarLogger::~footVarLogger()
{
	me->_n.shutdown();
}

bool footVarLogger::init() //! Initialization of the node. Its datatype (bool) reflect the success in initialization
{

	if (_platform_name==LEFT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_LEFT, 1, boost::bind(&footVarLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    	_pubFTSensorFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("footVarLogger_left/filteredFTSensorWrench", 1);
		
	}
	if (_platform_name==RIGHT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&footVarLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
    	_pubFTSensorFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("footVarLogger_right/filteredFTSensorWrench", 1);
	}

	_subForceTorqueSensor = _n.subscribe<geometry_msgs::WrenchStamped>("/ft_sensor/netft_data", 1, boost::bind(&footVarLogger::updateFtSensorWrench,this,_1),ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

	//Subscriber definitions	
	signal(SIGINT,footVarLogger::stopNode);


	_outputFile.open(ros::package::getPath(std::string("foot_variables_log"))+"/data/"+_filename+".txt");

	
	if (_n.ok()) 
	{   
		ros::spinOnce();
		ROS_INFO("The variables logging node of the foot interface is about to start ");
		//! Load the default gains for position control

		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void footVarLogger::stopNode(int sig)
{
    me->_stop= true;
}

void footVarLogger::logData()
{
  _outputFile << ros::Time::now() << " "
              << (int)_platform_id << " "
              << _platform_position.transpose() << " "
              << _platform_speed.transpose() << " "
              << _platform_effortD.transpose() << " "
              << (int)_platform_machineState<< std::endl;
}

void footVarLogger::run()
{
  while (!_stop) 
  {	

	if (_flagPlatformOutCommStarted)
	{
	_mutex.lock();
		if ((_platform_id!=(uint8_t) _platform_name)&&(_platform_id!=UNKNOWN))
		{
			ROS_ERROR("This node for variables looging is acting on the wrong platform");
			break;
		}
		else
		{
			if (_platform_id!=0){
				 footDataTransformation();
				 logData();
				 publishData();
			}
		}
	_mutex.unlock();
	}
		ros::spinOnce();
		_loopRate.sleep();
  }

  ROS_INFO("Parameters setting and variables logging stoped");
  
  logData();
  
  ros::spinOnce();
  _loopRate.sleep();
  _outputFile.close();
  ros::shutdown();
}

void footVarLogger::updateFtSensorWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  //! Set the bias... 
  if(!_ftSensorWrenchBiasOK && _flagPlatformOutCommStarted && _platform_id!=0)
  {
     Eigen::Vector3f loadForce = _footTipOrientation.transpose()*_footMass*_gravity;
     _ftSensorWrenchBias.segment(0,3) -= loadForce;
     _ftSensorWrenchBias.segment(3,3) -= _footComPositionFromSensor.cross(loadForce);
    _ftSensorWrenchBias += raw; 
    _ftSensorWrenchCount++;
    if(_ftSensorWrenchCount==NB_SAMPLES)
    {
      _ftSensorWrenchBias /= NB_SAMPLES;
      _ftSensorWrenchBiasOK = true;
      std::cerr << "[footVarLogger]: Bias " << ": " <<_ftSensorWrenchBias.transpose() << std::endl;
    }
  }

  if(_ftSensorWrenchBiasOK && _flagPlatformOutCommStarted && _platform_id!=0)
  {
    _ftSensorWrench = raw-_ftSensorWrenchBias;
	Eigen::Vector3f loadForce = _footTipOrientation.transpose()*_footMass*_gravity;
	_ftSensorWrench.segment(0,3) -= loadForce;
	_ftSensorWrench.segment(3,3) -= _footComPositionFromSensor.cross(loadForce);
    _ftSensorFilteredWrench = _filteredForceGain*_ftSensorFilteredWrench+(1.0f-_filteredForceGain)*_ftSensorWrench;
  }
}


void footVarLogger::publishData()
{
    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _ftSensorFilteredWrench(0);
    _msgFilteredWrench.wrench.force.y = _ftSensorFilteredWrench(1);
    _msgFilteredWrench.wrench.force.z = _ftSensorFilteredWrench(2);
    _msgFilteredWrench.wrench.torque.x = _ftSensorFilteredWrench(3);
    _msgFilteredWrench.wrench.torque.y = _ftSensorFilteredWrench(4);
    _msgFilteredWrench.wrench.torque.z = _ftSensorFilteredWrench(5);
    _pubFTSensorFilteredWrench.publish(_msgFilteredWrench);
}


void footVarLogger::footDataTransformation()
{

    Eigen::Matrix<float,NB_AXIS,1> offset;
    offset << WS_LIMITS[X],WS_LIMITS[Y],0.0f,0.0f,0.0f;
    Eigen::Matrix4f H;
    H = FootPlatformModel::forwardKinematics(_platform_position+offset,true);
    _footTipPosition = H.block(0,3,3,1);
    _footTipOrientation = H.block(0,0,3,3);

    //   std::cerr << "joints: "<<_platform_position+offset << std::endl;
    //   std::cerr <<"Position: " << _footTipPosition.transpose()-_H0.block(0,3,3,1).transpose() << std::endl;
    //   std::cerr << "Orientation: " << std::endl;
    //   std::cerr << _footTipOrientation << std::endl;

  }


void footVarLogger::fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg)
{
	_flagOutputMessageReceived=true;
	_platform_id = msg->platform_id; 
	for (int k=0; k<NB_AXIS; k++)
		{
			_platform_position[k]=msg->platform_position[k];
			_platform_speed[k]=msg->platform_speed[k];
			_platform_effortD[k]=msg->platform_effortD[k];
			_platform_effortM[k]=msg->platform_effortM[k];
		}	
	_platform_machineState= (footVarLogger::State) msg->platform_machineState;
	_platform_controllerType= (footVarLogger::Controller) msg->platform_controllerType;
	if(!_flagPlatformOutCommStarted)
	{_flagPlatformOutCommStarted=true;}
} 

