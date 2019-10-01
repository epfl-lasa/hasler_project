#ifndef __footVarSynchronizer_H__
#define __footVarSynchronizer_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include "nav_msgs/Path.h"
#include "Eigen/Eigen"
#include <signal.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <foot_variables_sync/machineStateParamsConfig.h>
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"
#include <custom_msgs/FootOutputMsg_v2.h>
#include <custom_msgs/FootInputMsg_v2.h>
#include <custom_msgs/setStateSrv.h>
#include <custom_msgs/setControllerSrv.h>

#define NB_PARAMS_CATEGORIES 10
#define NB_FO_CATEGORIES 6


using namespace std;

class footVarSynchronizer
{

    public:
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2};    
	private:
        enum Params_Category {M_STATE, EFF_COMP, C_AXIS, C_TYPE, FLAG_SENDPOS,FLAG_CAPTUREPOS, DES_POS, FLAG_GAINS, PID_POS, PID_SPEED};
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
    
    // Subscribers declarations
    ros::Subscriber _subFootOutput;            // FootOutputMsg_v2
    ros::Subscriber _subFootInput;            // FootInputMsg_v2
    
    // Publisher declaration
    ros::Publisher _pubFootInput;               // FootInputMsg_v2

    ros::ServiceClient _clientSetState;
    ros::ServiceClient _clientSetController;

    // Subsciber and publisher messages declaration
    custom_msgs::FootInputMsg_v2 _msgFootInput;
    custom_msgs::FootOutputMsg_v2 _msgFootOutput;
    custom_msgs::FootOutputMsg_v2 _msgFootOutputPrev;
    custom_msgs::setStateSrv _srvSetState;
    custom_msgs::setControllerSrv _srvSetController;
    
    //foot_variables_sync::FootOutputMsg_v2 _msgFootOutput;

    //!boolean variables
    
    bool _stop;

    std::mutex _mutex;
    static footVarSynchronizer* me;

    //! Dynamic Reconfigures    
	dynamic_reconfigure::Server<foot_variables_sync::machineStateParamsConfig> _dynRecServer;
	dynamic_reconfigure::Server<foot_variables_sync::machineStateParamsConfig>::CallbackType _dynRecCallback;
    foot_variables_sync::machineStateParamsConfig _config;
    foot_variables_sync::machineStateParamsConfig _configPrev;
    //Internal Variables

    Platform_Name _platform_name;

    //Variables from messages
        //! FootOutputMsg_v2 -> Internal for the platform
            int8_t _platform_id;
            
            Eigen::Matrix<double,NB_AXIS,1> _platform_position;
            Eigen::Matrix<double,NB_AXIS,1> _platform_speed;
            Eigen::Matrix<double,NB_AXIS,1> _platform_effortD;
            Eigen::Matrix<double,NB_AXIS,1> _platform_effortM;
            
            Controller _platform_controllerType;
            State _platform_machineState;

    //! FootInputMsg_v2 + setStateSrv + setControllerSrv -> External = within the ros network
        
            Eigen::Matrix<double,NB_AXIS,1> _ros_position;
            Eigen::Matrix<double,NB_AXIS,1> _ros_speed;
            Eigen::Matrix<double,NB_AXIS,1> _ros_effort;

            bool _ros_defaultControl;

            int8_t _ros_controlledAxis;
            // int8_t _ros_controlledAxis_prev;
            
            Eigen::Matrix<uint8_t,NB_EFFORT_COMPONENTS,1> _ros_effortComp;
            // Eigen::Matrix<uint8_t,NB_EFFORT_COMPONENTS,1> _ros_effortComp_prev;

            uint8_t _ros_controllerType; //! To be compared with platform
            int8_t _ros_newState; //! To be compared with platform
            

        //! PID Gains
        // PID variables
        //General Variables

        Eigen::Matrix<double,NB_AXIS,1> _ros_posP;
        Eigen::Matrix<double,NB_AXIS,1> _ros_posI;
        Eigen::Matrix<double,NB_AXIS,1> _ros_posD;
        Eigen::Matrix<double,NB_AXIS,1> _ros_speedP;
        Eigen::Matrix<double,NB_AXIS,1> _ros_speedI;
        Eigen::Matrix<double,NB_AXIS,1> _ros_speedD;

        // Other variables
    public:


        bool _flagWasDynReconfCalled;
        bool _flagInitialConfig;
        bool _flagParamsActionsTaken;
        bool _flagPlatformActionsTaken;

        bool _flagPlatformOutCommStarted;
        bool _flagOutputMessageReceived;
        bool _flagPlatformInCommStarted;
        bool _flagPositionOnlyPublished;
        
        bool _flagControlThisPosition; //! To make sure you don't send a torque and position in the same message
        bool _flagCapturePlatformPosition;

        bool _flagUpdateConfig;

        bool _flagIsParamStillSame[NB_PARAMS_CATEGORIES];
        
        bool _flagIsPlatformStillSame[NB_PARAMS_CATEGORIES];

        bool _flagSetControllerRequested;
        bool _flagSetStateRequested;

        bool _flagResponseSetController;
        bool _flagResponseSetState;

        


// METHODS
	public:
	// footVarSynchronizer(ros::NodeHandle &n_1, ros::NodeHandle &n_2, ros::NodeHandle &n_3, double frequency, footVarSynchronizer::Platform_Name platform_id_);
	footVarSynchronizer(ros::NodeHandle &n_1, double frequency, footVarSynchronizer::Platform_Name platform_id_);
	
    ~footVarSynchronizer();
	
    bool init();
	void run();

	private:
	
    //! ROS METHODS

    //bool allSubscribersOK();
    void fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg); 
    void sniffFootInput(const custom_msgs::FootInputMsg_v2::ConstPtr& msg); 
    
    void dynamicReconfigureCallback(foot_variables_sync::machineStateParamsConfig &config, uint32_t level);
    void changeParamCheck();
    void requestDoActionsParams();
    void changedPlatformCheck();
    void requestDoActionsPlatform();

    void publishPositionOnly();
    void requestSetController();
    void updateConfigAfterParamsChanged();
    void updateConfigAfterPlatformChanged();
    void requestSetState();

    //! OTHER METHODS
    void controlGainsDefault(int axis_);
    void resetDesiredPositionToCurrent(int axis_); 
    static void stopNode(int sig);
};
#endif  // __footVarSynchronizer_H__