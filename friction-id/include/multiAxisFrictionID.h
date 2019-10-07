#ifndef __multiAxisFrictionID_H__
#define __multiAxisFrictionID_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include "Eigen/Eigen"
#include <signal.h>
#include "../../5_axis_platform/lib/platform/src/definitions_2.h"
#include <custom_msgs/FootOutputMsg_v2.h>
#include <custom_msgs/FootInputMsg_v2.h>



using namespace std;

class multiAxisFrictionID
{

    public:
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2};    
	private:
        enum JointState {POSITION, SPEED, ACCELERATION}; 
        enum EffortComp {NORMAL,CONSTRAINS,COMPENSATION,FEEDFORWARD}; 
        enum State {HOMING,CENTERING,TELEOPERATION,EMERGENCY,STANDBY,RESET,ROBOT_STATE_CONTROL}; 
        enum Controller {TORQUE_ONLY, POSITION_ONLY, SPEED_ONLY, SPEED_POSITION_CASCADE, POSITION_SPEED_CASCADE}; 

    // ros variables  

    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;
    
    public:
        //Stair case variables
        Eigen::Matrix<float, NB_AXIS, 1> _limitWS;
        float _tau[NB_AXIS];                                 // time constant of the loading of the step
        int8_t _nSteps[NB_AXIS];                            // number of steps
        int8_t _currentStep[NB_AXIS];
        ros::Duration _ros_dt;
        ros::Time _currentTime;
        ros::Time _prevTime[NB_AXIS];
     
    private: 

	//!subscribers and publishers declaration    
    
    // // Subscribers declarations
     ros::Subscriber _subFootOutput;            // FootOutputMsg_v2
    
    // Publisher declaration
    ros::Publisher _pubFootInput;               // FootInputMsg_v2

    // Subsciber and publisher messages declaration
    custom_msgs::FootInputMsg_v2 _msgFootInput;
    
    //!boolean variables
    
    bool _stop;

    std::mutex _mutex;
    static multiAxisFrictionID* me;

    //Internal Variables

    Platform_Name _platform_name;

    //Variables from messages
        //! FootOutputMsg_v2 -> Internal for the platform
    int8_t _platform_id;

    Eigen::Matrix<double, NB_AXIS, 1> _platform_position;
    Eigen::Matrix<double, NB_AXIS, 1> _platform_speed;
    Eigen::Matrix<double, NB_AXIS, 1> _platform_effortD;
    Eigen::Matrix<double, NB_AXIS, 1> _platform_effortM;

    Eigen::Matrix<double, NB_AXIS, 1> _ros_position;
    Eigen::Matrix<double, NB_AXIS, 1> _ros_speed;
    Eigen::Matrix<double, NB_AXIS, 1> _ros_effort;

    Controller _platform_controllerType;
    State _platform_machineState;
    
    //Flags 

public:
    bool _flagOutputMessageReceived;
    bool _flagPlatformInCommStarted;
    bool _flagPlatformOutCommStarted;
    bool _flagPositionOnlyPublished;
    bool _flagInitialConfig;
    bool _flagIntegratorsZeroed[NB_AXIS];

private:
    bool _flagStairCaseStarted[NB_AXIS];
    bool _flagStairCaseFinished[NB_AXIS];
    bool _flagNextStep[NB_AXIS][2];

    // METHODS
public:
	// multiAxisFrictionID(ros::NodeHandle &n_1, ros::NodeHandle &n_2, ros::NodeHandle &n_3, double frequency, multiAxisFrictionID::Platform_Name platform_id_);
	multiAxisFrictionID(ros::NodeHandle &n_1, double frequency, multiAxisFrictionID::Platform_Name platform_id_);
	
    ~multiAxisFrictionID();
	
    bool init();
	void run();

	private:
	
    //! ROS METHODS

    //bool allSubscribersOK();
    void fetchFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg);     
    void publishPositionOnly();
    
    //! OTHER METHODS
    double stepFunGen(int axis_, int dir_);
    void staircaseJointGen(int axis_, int dir_);
    static void stopNode(int sig);
};
#endif  // __multiAxisFrictionID_H__