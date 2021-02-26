#ifndef __multiAxisFrictionID_H__
#define __multiAxisFrictionID_H__

#include "ros/ros.h"
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include "Eigen/Eigen"
#include <signal.h>
#include "../../5_axis_platform/lib/platform/src/definitions_main.h"
#include "../../5_axis_platform/lib/platform/src/definitions_ros.h"
#include "../../5_axis_platform/lib/platform/src/definitions_security.h"
#include "../../5_axis_platform/lib/platform/src/definitions_pid.h"
#include "lp_filter.h"
#include <custom_msgs/FootOutputMsg.h>
#include <custom_msgs/FootInputMsg.h>

using namespace std;

#define NB_DIR 4
#define NB_ERROR_CHECKS 2 
#define NB_STEPS_DEFAULT 20
#define TAU_DEFAULT 11.2f

class multiAxisFrictionID
{

    public:
        enum Direction {FORWARD_1=0, REVERSE_1=1, REVERSE_2 = 2, FORWARD_2 = 3};
        enum Platform_Name {UNKNOWN=0,RIGHT=1, LEFT=2};
        enum Strategy {NONE, STEPS, RAMP};    
        enum Sequence {ONE_AFTER_THE_OTHER, ALL_TOGETHER};
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
        //Strate case variables
        float _tau;                                 // time constant of the loading of the step
        int _nSteps[NB_AXIS];                            // number of steps
        float _stepSize[NB_AXIS];
        int8_t _currentStep[NB_AXIS];
        int8_t _currentOffset[NB_AXIS];
        ros::Duration _ros_dt;
        ros::Time _currentTime;
        ros::Time _prevTime[NB_AXIS];
     
    private: 

	//!subscribers and publishers declaration    
    
    // // Subscribers declarations
     ros::Subscriber _subFootOutput;            // FootOutputMsg
    
    // Publisher declaration
    ros::Publisher _pubFootInput;               // FootInputMsg

    // Subsciber and publisher messages declaration
    custom_msgs::FootInputMsg _msgFootInput;
    
    //!boolean variables
    
    bool _stop;

    std::mutex _mutex;
    static multiAxisFrictionID* me;

    //Internal Variables

    Platform_Name _platform_name;

    //Variables from messages
        //! FootOutputMsg -> Internal for the platform
    int8_t _platform_id;

    Eigen::Matrix<float, NB_AXIS, 1> _platform_position;
    Eigen::Matrix<float, NB_AXIS, 1> _platform_speed;
    Eigen::Matrix<float, NB_AXIS, 1> _platform_effortD;
    Eigen::Matrix<float, NB_AXIS, 1> _platform_effortM;

    Eigen::Matrix<float, NB_AXIS, 1> _ros_position;
    lp_filter* _ros_positionFilter[NB_AXIS];
    Eigen::Matrix<float, NB_AXIS, 1> _ros_speed;
    Eigen::Matrix<float, NB_AXIS, 1> _ros_effort;

    Controller _platform_controllerType;
    State _platform_machineState;
    Strategy _strategy;
    Sequence _sequence;
    int _whichAxis;
    uint8_t _currentAxis;

    //Variables related to the methods
        Direction _direction[NB_AXIS];
        Direction _direction_prev[NB_AXIS];
        float _WS[NB_AXIS];
    //Flags 

public:
    bool _flagOutputMessageReceived;
    bool _flagPlatformInCommStarted;
    bool _flagPlatformOutCommStarted;
    bool _flagPositionOnlyPublished;
    bool _flagIntegratorsZeroed[NB_AXIS][NB_DIR];
    bool _flagROSINFO;

private:
    bool _flagStrategyStarted[NB_AXIS][NB_DIR]; //!Backwards-Forwards
    bool _flagStrategyFinished[NB_AXIS][NB_DIR];
    bool _flagNextStep[NB_AXIS][NB_DIR][NB_ERROR_CHECKS];
    bool _flagNextAxis;
    bool _flagWSDefined[NB_AXIS];

    // METHODS
public:
	// multiAxisFrictionID(ros::NodeHandle &n_1, ros::NodeHandle &n_2, ros::NodeHandle &n_3, double frequency, multiAxisFrictionID::Platform_Name platform_id_);
	multiAxisFrictionID(ros::NodeHandle &n_1, double frequency,multiAxisFrictionID::Platform_Name platform_id, multiAxisFrictionID::Strategy strategy_, float tau_, int whichAxis_); 
	
    ~multiAxisFrictionID();
	
    bool init();
	void run();

	private:
	
    //! ROS METHODS

    //bool allSubscribersOK();
    void fetchFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg);     
    void publishPositionOnly();
    
    //! FUNCTION GENERATOR METHODS
    double stepFunGen(int axis_, float tau_, Strategy strategy_);
    void funcJointGen(int axis_, float tau_);
    void rampJointGen(int axis_, float slope_);
    void axisSequenceControl(int whichAxis_, Sequence whichSequence_);
    void functionControl(int whichAxis_, Sequence whichSequence_);
    static void stopNode(int sig);
};
#endif  // __multiAxisFrictionID_H__