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
	_ftWrenchBiasOK=false;

	_ftWrenchBias.setConstant(0.0f);
	_ftWrench.setConstant(0.0f);

	_gravity << 0.0f, 0.0f, -9.80665f;
	_footComPositionFromSensor << 0.0f,0.05f,0.02f; //![m]
	_footMass = 0.375f; //! [kg]
	_ftWrenchCount = 0;

	_filteredForceGain=0.5;
	_ftFilteredWorldWrench.setConstant(0.0f);
	_ftFilteredSensorWrench.setConstant(0.0f);

	_footFtSensorPosition.setConstant(0.0f);
    _footFtSensorOrientation.setIdentity();
	_footFtSensorQuaternion.setIdentity();
    // _xdFootFtSensor.setConstant(0.0f);
	
	_footOffset << 0.0f,0.0f,10.0f,0.0f,0.0f;

  		Eigen::Matrix<float,NB_AXIS,1> temp;
  		//temp << WS_LIMITS[X],WS_LIMITS[Y],0.0f,0.0f,0.0f;
		temp << 0.0, 0.0, 0.0f, 0.0f, 0.0f;
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
    	_pubFtSensorFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("FI_Data/Left/sensorFilteredFT", 1);
		_pubFootPose = _n.advertise<geometry_msgs::PoseStamped>("FI_Data/Left/sensorPose", 1);
		_pubMotorsEffortM = _n.advertise<custom_msgs::FootOutputMsg_v2>("FI_Data/Left/motorsEffortFromFTSensor", 1);
		_subForceTorqueSensor = _n.subscribe<geometry_msgs::WrenchStamped>("ft_sensor_left/netft_data", 1, boost::bind(&footVarLogger::updateFtSensorWrench, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	}
	if (_platform_name==RIGHT){
		_subFootOutput = _n.subscribe<custom_msgs::FootOutputMsg_v2>(PLATFORM_PUBLISHER_NAME_RIGHT, 1, boost::bind(&footVarLogger::fetchFootOutput, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
		_pubFtSensorFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("FI_Data/Right/sensorFilteredFT", 1);
		_pubFootPose = _n.advertise<geometry_msgs::PoseStamped>("FI_Data/Right/sensorPose", 1);
		_pubMotorsEffortM = _n.advertise<custom_msgs::FootOutputMsg_v2>("FI_Data/Right/motorsEffortFromFTSensor", 1);
		_subForceTorqueSensor = _n.subscribe<geometry_msgs::WrenchStamped>("ft_sensor_right/netft_data", 1, boost::bind(&footVarLogger::updateFtSensorWrench, this, _1), ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
	}


	//Subscriber definitions	
	signal(SIGINT,footVarLogger::stopNode);

	if(_filename != std::string("no_file")){
		_outputFile.open(ros::package::getPath(std::string("foot_variables_log")) + "/data/" + _filename + ".txt");
	}
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
	if (_filename != std::string("no_file"))
	{
		_outputFile << ros::Time::now() << " "
					<< (int)_platform_id << " "
					<< _platform_position.transpose() << " "
					<< _platform_speed.transpose() << " "
					<< _platform_effortD.transpose() << " "
					<< (int)_platform_machineState << std::endl;
	}
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
				 computeMotorsEffortFtSensor();
				 //computeFtSensorWrenchDesiredMotors();
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
  if (_filename != std::string("no_file"))
  {
	  _outputFile.close();
  }
  ros::shutdown();
}

void footVarLogger::updateFtSensorWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  Eigen::Matrix<float, 6, 6> extendedRotation;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;
	
  extendedRotation.setConstant(0.0f);
  extendedRotation.block(0,0,3,3) = _footFtSensorOrientation;
  extendedRotation.block(3, 3, 3, 3) = _footFtSensorOrientation;

	  //! Set the bias...
	  if (!_ftWrenchBiasOK && _flagPlatformOutCommStarted && _platform_id != 0)
  {
    // Eigen::Vector3f loadForce = _footFtSensorOrientation.transpose()*_footMass*_gravity;
    // _ftWrenchBias.segment(0,3) += loadForce;
    // _ftWrenchBias.segment(3,3) += _footComPositionFromSensor.cross(loadForce);
    _ftWrenchBias += raw; 
    _ftWrenchCount++;
    if(_ftWrenchCount==NB_SAMPLES)
    {
      _ftWrenchBias /= NB_SAMPLES;
      _ftWrenchBiasOK = true;
      std::cerr << "[footVarLogger]: Bias " << ": " <<_ftWrenchBias.transpose() << std::endl;
    }
  }

  if(_ftWrenchBiasOK && _flagPlatformOutCommStarted && _platform_id!=0)
  {
    _ftWrench = raw-_ftWrenchBias;
	Eigen::Vector3f loadForce = _footFtSensorOrientation.transpose()*_footMass*_gravity;
	_ftWrench.segment(0,3) -= loadForce;
	_ftWrench.segment(3,3) -= _footComPositionFromSensor.cross(loadForce);
    _ftFilteredSensorWrench = _filteredForceGain*_ftFilteredSensorWrench+(1.0f-_filteredForceGain)*_ftWrench;

	//! Attention!!! : The axes of the force torque sensor are rotated so as to know the axis of FORCE APPLICATION BY THE PLATFORM TO THE CENTER OF THE FOOT NOT CONVERSELY
	Eigen::Matrix3f R; //! Rotation of the axes of the sensor to the axes of the frame in the center of the pedal
	Eigen::Matrix<float,6,6> R_extended;
	R.setConstant(0.0f);
	R_extended.setConstant(0.0f);
	R << 1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, -1.0f;
	R_extended.block(0, 0, 3, 3) = R;
	R_extended.block(3,3, 3, 3) = R;
	//std::cerr<<R_extended<<std::endl;
	_ftFilteredWorldWrench = extendedRotation * R_extended * _ftFilteredSensorWrench;
  }
}

void footVarLogger::publishData()
{
    _msgFilteredWrench.header.frame_id = "world";
    _msgFilteredWrench.header.stamp = ros::Time::now();
    _msgFilteredWrench.wrench.force.x = _ftFilteredWorldWrench(0);
    _msgFilteredWrench.wrench.force.y = _ftFilteredWorldWrench(1);
    _msgFilteredWrench.wrench.force.z = _ftFilteredWorldWrench(2);
    _msgFilteredWrench.wrench.torque.x = _ftFilteredWorldWrench(3);
    _msgFilteredWrench.wrench.torque.y = _ftFilteredWorldWrench(4);
    _msgFilteredWrench.wrench.torque.z = _ftFilteredWorldWrench(5);
    _pubFtSensorFilteredWrench.publish(_msgFilteredWrench);

	_msgFootPoseStamped.header.frame_id = "world";
	_msgFootPoseStamped.header.stamp = ros::Time::now();
	_msgFootPoseStamped.pose.orientation.x=_footFtSensorQuaternion.x();
	_msgFootPoseStamped.pose.orientation.y = _footFtSensorQuaternion.y();
	_msgFootPoseStamped.pose.orientation.z = _footFtSensorQuaternion.z();
	_msgFootPoseStamped.pose.orientation.w = _footFtSensorQuaternion.w();
	_msgFootPoseStamped.pose.position.x = _footFtSensorPosition(0);
	_msgFootPoseStamped.pose.position.y = _footFtSensorPosition(1);
	_msgFootPoseStamped.pose.position.z = _footFtSensorPosition(2);
	_pubFootPose.publish(_msgFootPoseStamped);

	_msgFootOutput.platform_stamp=ros::Time::now();
	_msgFootOutput.platform_effortM[0]=_desiredMotorsEffort(0);
	_msgFootOutput.platform_effortM[1] = _desiredMotorsEffort(1);
	_msgFootOutput.platform_effortM[2] = _desiredMotorsEffort(2);
	_msgFootOutput.platform_effortM[3] = _desiredMotorsEffort(3);
	_msgFootOutput.platform_effortM[4] = _desiredMotorsEffort(4);
	_pubMotorsEffortM.publish(_msgFootOutput);
}

void footVarLogger::footDataTransformation()
{
	if (_flagPlatformOutCommStarted && (int)_platform_id!=0){
		Eigen::Matrix<float,NB_AXIS,1> offset;
		// offset << -WS_LIMITS[X],-WS_LIMITS[Y],0.0f,0.0f,0.0f;
		offset << 0.0f,0.0f,0.0f,0.0f,0.0f;
		Eigen::Matrix4f H;
		H = FootPlatformModel::forwardKinematics(_platform_position+offset,true);
		_footFtSensorPosition = H.block(0,3,3,1);
		_footFtSensorOrientation = H.block(0,0,3,3);
		_footFtSensorQuaternion = Eigen::Quaternionf(_footFtSensorOrientation);
		//   std::cerr << "joints: "<<_platform_position+offset << std::endl;
		//   std::cerr <<"Position: " << _footFtSensorPosition.transpose()-_H0.block(0,3,3,1).transpose() << std::endl;
		//   std::cerr << "Orientation: " << std::endl;
		//   std::cerr << _footFtSensorOrientation << std::endl;
	}
  }

  void footVarLogger::computeMotorsEffortFtSensor()
  {
	 	Eigen::Matrix<float, 6, 5> J;
		Eigen::Matrix<float, 5, 1> offset, temp;
		offset << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
		J = FootPlatformModel::geometricJacobian(_platform_position + offset);
		
		temp = J.transpose() * _ftFilteredWorldWrench;
		_desiredMotorsEffort(0) = temp(1);
		_desiredMotorsEffort(1) = temp(0);
		_desiredMotorsEffort(2) = -temp(2);
		_desiredMotorsEffort(3) = temp(3);
		_desiredMotorsEffort(4) = temp(4);
	}

  void footVarLogger::fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr &msg)
  {
	  _flagOutputMessageReceived = true;
	  _platform_id = msg->platform_id;
	  for (int k = 0; k < NB_AXIS; k++)
	  {
		  _platform_position[k] = msg->platform_position[k];
		  _platform_speed[k] = msg->platform_speed[k];
		  _platform_effortD[k] = msg->platform_effortD[k];
		  _platform_effortM[k] = msg->platform_effortM[k];
	  }
	  _platform_machineState = (footVarLogger::State)msg->platform_machineState;
	  _platform_controllerType = (footVarLogger::Controller)msg->platform_controllerType;
	  if (!_flagPlatformOutCommStarted)
	  {
		  _flagPlatformOutCommStarted = true;
	  }
} 

