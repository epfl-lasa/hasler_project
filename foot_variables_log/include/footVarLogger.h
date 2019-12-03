#ifndef __footVarLogger_H__
#define __footVarLogger_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <deque>
#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include "nav_msgs/Path.h"
#include "Eigen/Eigen"
#include <signal.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"
#include <custom_msgs/FootOutputMsg_v2.h>
#include <custom_msgs/FootInputMsg_v2.h>

using namespace std;

#define NB_SAMPLES 50 //! Number of samples to filter the force information


class footVarLogger
{

    public:
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2};    
	private:
        enum FootOutput_Category {FO_POS, FO_SPEED, FO_EFFORTD, FO_EFFORTM, FO_M_STATE, FO_C_TYPE};
    //! External Enumerations (from '../../5_axis_platform/lib/platform/src/Platform.h')
        enum JointState {POSITION, SPEED, ACCELERATION}; 
        
        //! TODO: MAKE THIS GLOBAL VARIABLES WITH THEIR CORRESPONDING NAMES FOR LOGGING
        enum EffortComp {NORMAL,CONSTRAINS,COMPENSATION,FEEDFORWARD}; //! Count := 4
        enum State {HOMING,CENTERING,TELEOPERATION,EMERGENCY,STANDBY,RESET,ROBOT_STATE_CONTROL}; 
        enum Controller {TORQUE_ONLY, POSITION_ONLY, SPEED_ONLY, SPEED_POSITION_CASCADE, POSITION_SPEED_CASCADE}; //! F= D(K(x-xd)-x_dot) SPEED_POSITION_CASCADE IS AN IMPEDANCE CTRL MODULATED BY DYNAMICAL SYSTEM
        
    // ros variables  


    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

	//!subscribers and publishers declaration    
    
    // Publishers declarations

    ros::Publisher _pubFtSensorFilteredWrench;					// Filtered measured wrench
    ros::Publisher _pubDesiredWrench;                  // Filtered measured wrench
    ros::Publisher _pubMeasuredWrench;                  // Filtered measured wrench
    ros::Publisher _pubFootPose;
    ros::Publisher _pubMotorsEffortM;
    // Subscribers declarations
    ros::Subscriber _subFootOutput;            // FootOutputMsg_v2
    ros::Subscriber _subFootInput;            // FootInputMsg_v2
    ros::Subscriber _subForceTorqueSensor;			// force torque sensor


    // Subsciber and publisher messages declaration
    custom_msgs::FootOutputMsg_v2 _msgFootOutput;
    geometry_msgs::WrenchStamped _msgFilteredWrench;
    geometry_msgs::WrenchStamped _msgDesiredWrench;
    geometry_msgs::WrenchStamped _msgMeasuredWrench;
    geometry_msgs::PoseStamped _msgFootPoseStamped;
    //foot_variables_sync::FootOutputMsg_v2 _msgFootOutput;

    //!boolean variables
    
    bool _stop;

    std::mutex _mutex;
    static footVarLogger* me;

    Platform_Name _platform_name;

    //Variables from messages
        //! FootOutputMsg_v2 -> Internal for the platform
            int8_t _platform_id;
            
            Eigen::Matrix<float,NB_AXIS,1> _platform_position;
            Eigen::Matrix<float,NB_AXIS,1> _platform_speed;
            Eigen::Matrix<float,NB_AXIS,1> _platform_effortD;
            Eigen::Matrix<float,NB_AXIS,1> _platform_effortM;
            
        Controller _platform_controllerType;
        State _platform_machineState;

    //! Variables for Logging    
        std::string _filename;
        std::ifstream _inputFile;
		std::ofstream _outputFile;    

    //! User variables

        // Foot characteristics
		float _footMass;														// foot mass [kg]
		float _footOffsetFromEE;										// foot offset along z axis of end effector [m]							
		Eigen::Vector3f _footComPositionFromSensor;                     // Offset of the foot [m]	(3x1)
		Eigen::Vector3f _gravity;										// Gravity vector [m/s^2] (3x1)
        
       
        Eigen::Matrix<float,NB_AXIS,1> _desiredMotorsEffort;		// Filtered wrench [N and Nm] (6x1)
        Eigen::Matrix<float, 6, 1> _desiredFootWrench; // Filtered wrench [N and Nm] (6x1)
        Eigen::Matrix<float, 6, 1> _measuredFootWrench; // Filtered wrench [N and Nm] (6x1)

        // Eigen::Vector3f _vdFoot;
        // Eigen::Vector3f _xdFoot;
        // Eigen::Vector3f _FdFoot;
        Eigen::Vector3f _footFtSensorPosition;

        Eigen::Matrix3f _footFtSensorOrientation;
        Eigen::Quaternionf _footFtSensorQuaternion;    
        // Eigen::Vector3f _xdFootTip;
        Eigen::Matrix<float,6,1> _ftWrench;						// Wrench [N and Nm] (6x1)
		Eigen::Matrix<float,6,1> _ftWrenchBias;				// Wrench bias [N and Nm] (6x1)
        Eigen::Matrix<float, 6, 1> _ftFilteredWorldWrench;          // Filtered wrench [N and Nm] (6x1)
        Eigen::Matrix<float, 6, 1> _ftFilteredSensorWrench;          // Filtered wrench [N and Nm] (6x1)

        int _ftWrenchCount;

		float _filteredForceGain;		// Filtering gain for force/torque sensor

		bool _ftWrenchBiasOK;							// Check if computation of force/torque sensor bias is OK



    public:


         bool _flagPlatformOutCommStarted;
         bool _flagOutputMessageReceived;
         bool _flagPositionOnlyPublished;


// METHODS
	public:
	footVarLogger(ros::NodeHandle &n_1, double frequency, footVarLogger::Platform_Name platform_id_, std::string filename_);
	
    ~footVarLogger();
	
    bool init();
	void run();

	private:
	
    void footDataTransformation();
    void computeMotorsEffortFtSensor();

        //! ROS METHODS

        //! Callbacks

        void
        updateFtSensorWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void publishData(); 
    
    //bool allSubscribersOK();
    void fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg); 
    void logData();
    static void stopNode(int sig);
    
};
#endif  // __footVarLogger_H__