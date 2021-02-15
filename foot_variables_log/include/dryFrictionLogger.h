#ifndef __dryFrictionLogger_H__
#define __dryFrictionLogger_H__

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
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../5_axis_platform/lib/platform/src/definitions_security.h"
#include "../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include <custom_msgs/FootOutputMsg.h>
#include <custom_msgs/FootInputMsg_v2.h>
#include <custom_msgs/setControllerSrv.h>
#include <custom_msgs/setStateSrv.h>

using namespace std;

#define NB_SAMPLES 50 //! Number of samples to filter the force information


class dryFrictionLogger
{

    public:
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2};
        enum DecisionState {COMM_BEGIN, PLATFORM_IDENTIFY, REQ_ROBOT_STATE, REQ_POS_CTRL, POS_TOLERANCE, CHANGE_POSITION , CHANGE_SIGN,
        NEW_DATA_POINT,  CHANGE_AXIS,  REQ_TELEOP_CTRL, REQ_TORQUE_CTRL,  COMPUTE_FRICTION, ID_FRICTION , STOP_ALL};
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

    ros::Publisher _pubFriction;                  // Filtered measured wrench
    ros::Publisher _pubFootInput;
    // Subscribers declarations
    ros::Subscriber _subFootOutput;            // FootOutputMsg

    ros::ServiceClient _clientSetController;
    ros::ServiceClient _clientSetState;

    // Subsciber and publisher messages declaration
    custom_msgs::FootOutputMsg _msgFootOutput;
    custom_msgs::FootOutputMsg _msgStaticFriction;
    custom_msgs::FootInputMsg_v2 _msgFootInput;
    custom_msgs::setControllerSrv _srvSetController;
    custom_msgs::setStateSrv _srvSetState;

    //!boolean variables
    
    bool _stop;

    bool _flagFrictionIDed;
    bool _flagNextPositionOk;
    bool _flagFirstTime;
    // bool _flagFirstTimeFriction;

    bool _flagResponseSetController;
    bool _flagResponseSetState;

    bool _flagReqSetControllerForPosition;
    bool _flagReqSetStateForPosition;

    bool _flagReqSetControllerForFrictionID;
    bool _flagReqSetStateForFrictionID;


    // volatile bool _flagLastPositionOk;
    std::mutex _mutex;
    static dryFrictionLogger* me;

    Platform_Name _platform_name;
    Axis _frictionAxis;
    //Variables from messages
        //! FootOutputMsg -> Internal for the platform
            int8_t _platform_id;
            
            Eigen::Matrix<float,NB_AXIS,1> _platform_position;
            Eigen::Matrix<float, NB_AXIS, 1> _platform_position_last;
            Eigen::Matrix<float,NB_AXIS,1> _platform_speed;
            Eigen::Matrix<float,NB_AXIS,1> _platform_effortD;
            Eigen::Matrix<float,NB_AXIS,1> _platform_effortM;
            
        Controller _platform_controllerType;
        State _platform_machineState;
        DecisionState _decisionState;
        DecisionState _lastDecisionState;

        //! Variables for Logging
        std::string _filename;
        std::string _rawFilename;
        std::ofstream _outputFile;    

    //! User variables

        Eigen::Matrix<float,NB_AXIS,1> _frictionMotorsEffort;		// Identified motors static friction [N and Nm] (6x1)
        Eigen::Matrix<float,NB_AXIS,1> _desiredMotorsEffort;		// Variable effort for motors [N and Nm] (6x1)
        Eigen::Matrix<float, NB_AXIS, 1> _desiredMotorsPosition;      // Variable effort for motors [N and Nm] (6x1)

        // Eigen::Matrix<float, NB_AXIS, 1> _desiredMotorsEffort_prev;

        int _motionSign;
        int _nSignChanges;
        int _nAxis;
        int _nPositions;
        int _nDataPoints;

    public:


         bool _flagPlatformOutCommStarted;
         bool _flagOutputMessageReceived;
         bool _flagPositionOnlyPublished;
         bool _flagLoggingOk;


// METHODS
	public:
	dryFrictionLogger(ros::NodeHandle &n_1, double frequency, dryFrictionLogger::Platform_Name platform_id_, Axis axis_, std::string filename_);
	
    ~dryFrictionLogger();
	
    bool init();
	void run();

	private:
	
    void computeStaticFriction();
    void requestSetController(Controller controller_, bool* ControllerRequested_);
    void requestSetState(State state_, bool* StateRequested_);
    void generateNewRefPoints();
    void requestDocumentFriction();
    float clamp(float x, float out_min, float out_max);
    // void requestSetControllerTorque();

    //! ROS METHODS

    //! Callbacks

    void updateFtSensorWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void publishIDFriction();
    void publishFootEffort();
    void publishDesiredPosition();
    

    //bool allSubscribersOK();
    void fetchFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg); 
    void logData();
    static void stopNode(int sig);
    
};
#endif  // __dryFrictionLogger_H__