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
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../5_axis_platform/lib/platform/src/definitions_security.h"
#include "../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include <foot_variables_sync/machineStateParamsConfig.h>
#include <custom_msgs/FootOutputMsg.h>
#include <custom_msgs/FootInputMsg.h>
#include <custom_msgs/setStateSrv.h>
#include <custom_msgs/setControllerSrv.h>
#include <geometry_msgs/WrenchStamped.h>

#include "Utils_math.h"
#include "geometry_msgs/PointStamped.h"
#include "custom_msgs/TwoFeetOneToolMsg.h"

#define NB_PARAMS_CATEGORIES 10
#define NB_FO_CATEGORIES 6
#define NB_FI_PUBLISHERS 3

using namespace std;


const float effortLims[] = {15.0f, 15.0f, 7.0f, 7.0f, 7.0f};

class footVarSynchronizer
{

    public:
                
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2}; 
        
        enum PID_POS_Categories {S_TELEOP_PID,S_ROBOT_CTRL_PID,TOOL_POS_PID,TOOL_SPEED_PID,MP_TOOL_MIXED_PID,EXT_PID, NB_POS_PID_C};   
        enum Tool_Control {TOOL_POSITION_CTRL,TOOL_SPEED_CTRL};
	private:
        enum Params_Category {M_STATE, EFF_COMP, C_AXIS, C_TYPE, FLAG_SENDPOS,FLAG_CAPTUREPOS, DES_POS, FLAG_GAINS, PID_POS, PID_SPEED};
        enum FootOutput_Category {FO_POS, FO_SPEED, FO_EFFORTD, FO_EFFORTM, FO_M_STATE, FO_C_TYPE};
    //! External Enumerations (from '../../5_axis_platform/lib/platform/src/Platform.h')
        
        
    // ros variables  
    bool _flagPIDGainsByInput;
    bool _mixedPlatformOn;
    bool _controlTools;
    Tool_Control _myToolControl;

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;
    PID_POS_Categories _myPIDCategory;
    int _nbDesiredFootInputPublishers;
    

	//!subscribers and publishers declaration    
    
    // Subscribers declarations
    ros::Subscriber _subFootOutput;            // FootOutputMsg
    ros::Subscriber _subForceSensor;           // geometry_msgs/WrenchStamped.h
    ros::Subscriber _subForceModified;         // geometry_msgs/WrenchStamped.h
    ros::Subscriber _subTorquesModified;       //custom_msgs/FootInputMsg
    ros::Subscriber _subLegGravCompTorques;    //custom_msgs/FootInputMsg
    ros::Subscriber _subInertiaCoriolisCompTorques;    //custom_msgs/FootInputMsg
    
    ros::Subscriber _subLegGravCompWrench;    // geometry_msgs/WrenchStamped.h
    ros::Subscriber _subPlatformControlFromTool; // mixed platform

    std::vector<ros::Subscriber> _subDesiredFootInput;            // FootInputMsg
    std::vector<custom_msgs::FootInputMsg> _msgDesiredFootInput;
    custom_msgs::FootInputMsg _msgTotalDesiredFootInput;
    custom_msgs::TwoFeetOneToolMsg _msgTwoFeetOneTool;
   
    // Publisher declaration
    ros::Publisher _pubFootInput;               // FootInputMsg
    
    ros::ServiceClient _clientSetState;
    ros::ServiceClient _clientSetController;

    // Subsciber and publisher messages declaration
    custom_msgs::FootInputMsg _msgFootInput;
    custom_msgs::FootOutputMsg _msgFootOutput;
    custom_msgs::FootOutputMsg _msgFootOutputPrev;
    custom_msgs::setStateSrv _srvSetState;
    custom_msgs::setControllerSrv _srvSetController;
    
    
    
    //foot_variables_sync::FootOutputMsg _msgFootOutput;

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
    uint8_t _mainPlatform;

    //Variables from messages
        //! FootOutputMsg -> Internal for the platform
            int8_t _platform_id;
            
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _platform_position;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _platform_speed;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _platform_effortD;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _platform_effortM;
            
            Controller _platform_controllerType;
            State _platform_machineState;

    //! FootInputMsg + setStateSrv + setControllerSrv -> External = within the ros network
        
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_position;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_speed;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_effort;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_effortM;
            Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_filterAxisFS;

            Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _leg_grav_comp_effort;
            Eigen::Matrix<float, NB_PLATFORM_AXIS, 1> _inertial_coriolis_comp_effort;

            Eigen::Matrix<float, NB_AXIS_WRENCH, 1> _ros_forceSensor_controlled;

            Eigen::Matrix<float, NB_AXIS_WRENCH, 1> _legWrenchGravityComp;

            Eigen::Matrix<float, NB_AXIS_WRENCH, 1> _ros_forceModified; //! Comes from force modifier node

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

        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_posP;
        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_posI;
        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_posD;
        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_speedP;
        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_speedI;
        Eigen::Matrix<float,NB_PLATFORM_AXIS,1> _ros_speedD;

        float _ros_paramP_A[NB_POS_PID_C][NB_PLATFORM_AXIS];
        float _ros_paramP_B[NB_POS_PID_C][NB_PLATFORM_AXIS];
        float _ros_paramI[NB_POS_PID_C][NB_PLATFORM_AXIS];
        float _ros_paramD[NB_POS_PID_C][NB_PLATFORM_AXIS];

        const float _ros_posP_Max[NB_PLATFORM_AXIS] = {8000.0,8000.0,50000.0,10000.0,10000.0};
        const float _ros_posI_Max[NB_PLATFORM_AXIS] = {8000.0,8000.0,50000.0,10000.0,10000.0};
        const float _ros_posD_Max[NB_PLATFORM_AXIS] = {45.0,45.0,700.0,700.0,700.0};

        const float _ros_speedP_Max[NB_PLATFORM_AXIS] = {1000.0,1000.0,1000.0,1000.0,1000.0};
        const float _ros_speedI_Max[NB_PLATFORM_AXIS] = {1000.0,1000.0,1000.0,1000.0,1000.0};
        const float _ros_speedD_Max[NB_PLATFORM_AXIS] = {1000.0,1000.0,1000.0,1000.0,1000.0};
        // Other variables
    private:


        volatile bool _flagWasDynReconfCalled;
        bool  _flagForceModifiedConnected;
        #define HUMAN_ON_PLATFORM_THRESHOLD 10
        bool _flagHumanOnPlatform, _flagCompensateLeg;
        

        bool _flagLegCompTorquesRead;
        bool _flagInertiaCoriolisRead;
        bool _flagLegCompWrenchRead;

        bool _flagInitialConfig;
        bool _flagParamsActionsTaken;
        bool _flagPlatformActionsTaken;

        bool _flagOutputMessageReceived;
        bool _flagPlatformInCommStarted;
        bool _flagPositionOnlyPublished;
        bool _flagEffortOnlyPublished;

        volatile bool _flagControlThisPosition; //! To make sure you don't send a torque and position in the same message
        volatile bool _flagControlZeroEffort; 
        volatile bool _flagLoadPIDGains;
        volatile bool _flagSendPIDGains; 
        volatile bool _flagCapturePlatformPosition;
        volatile bool _flagUpdateConfig;

        volatile bool _flagTwoFeetOneToolRead;

        bool _flagIsParamStillSame[NB_PARAMS_CATEGORIES];
        
        bool _flagIsPlatformStillSame[NB_PARAMS_CATEGORIES];

        bool _flagSetControllerRequested;
        bool _flagSetStateRequested;

        bool _flagResponseSetController;
        bool _flagResponseSetState;

        std::vector<bool> _flagDesiredFootInputsRead;

        // METHODS
      public:
	// footVarSynchronizer(ros::NodeHandle &n_1, ros::NodeHandle &n_2, ros::NodeHandle &n_3, float frequency, footVarSynchronizer::Platform_Name platform_id_);
	footVarSynchronizer(ros::NodeHandle &n_1, float frequency, footVarSynchronizer::Platform_Name platform_id_);
	
    ~footVarSynchronizer();
	
    bool init();
	void run();

	private:
	
    void checkWhichPIDGainsToUse();
    //bool allSubscribersOK();
    void processFootOutput();
    void readFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg);
    void sniffFootInput(const custom_msgs::FootInputMsg::ConstPtr& msg); 
    
    
    void dynamicReconfigureCallback(foot_variables_sync::machineStateParamsConfig &config, uint32_t level);
    void changeParamCheck();
    void updateInternalVariables();
    void requestDoActionsParams();
    void changedPlatformCheck();
    void requestDoActionsPlatform();

    void publishFootInput(bool* flagVariableOnly_);
    void requestSetController();
    void updateConfigAfterParamsChanged();
    void updateConfigAfterPlatformChanged();
    void requestSetState();

    //! OTHER METHODS
    void controlGainsDefault(int axis_);
    void resetDesiredPositionToCurrent(int axis_);

    
    // void readForceSensor(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    // void readForceBias(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void readForceModified(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void readTorquesModified(const custom_msgs::FootOutputMsg::ConstPtr &msg);
    void readInertiaCoriolisCompFI(const custom_msgs::FootInputMsg::ConstPtr &msg);
    void readLegGravCompFI(const custom_msgs::FootInputMsg::ConstPtr &msg);
    void readLegGravityCompWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void readTwoFeetOneToolMsg(const custom_msgs::TwoFeetOneToolMsg::ConstPtr &msg);
    void correctForceForLegCompensation();
    void processAllPublishers();


    void readDesiredFootInputs(const custom_msgs::FootInputMsg::ConstPtr &msg, unsigned int n_);
    void getPIDParams();

    static void stopNode(int sig);
};
#endif  // __footVarSynchronizer_H__