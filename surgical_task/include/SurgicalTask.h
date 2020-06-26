#ifndef __SURGICAL_TASK_H__
#define __SURGICAL_TASK_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <termios.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/Marker.h"
#include "custom_msgs/FootInputMsg_v2.h"
#include "custom_msgs/FootOutputMsg_v2.h"
#include <dynamic_reconfigure/server.h>
#include "Eigen/Eigen"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "QpSolverRCM.h"
#include "CvxgenSolverRCM.h"
#include <pthread.h>


#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define NB_ROBOTS 2
#define FOOT_INTERFACE_X_RANGE 0.195
#define FOOT_INTERFACE_Y_RANGE 0.180
#define FOOT_INTERFACE_PITCH_RANGE 30.0
#define FOOT_INTERFACE_ROLL_RANGE 30.0
#define FOOT_INTERFACE_YAW_RANGE 40.0
#define NB_AXES_JOYSTICK 8
#define MAX_ORIENTATION_ERROR 0.2

enum ROBOT {LEFT = 0, RIGHT = 1};

class SurgicalTask 
{
  public: 

    enum ROBOT {LEFT = 0, RIGHT = 1};

    enum TROCAR_CONSTRAINT_STRATEGY {VIRTUAL_RCM=0, PRIMARY_TASK=1};

    enum RobotMode {TROCAR_SELECTION = 0, TROCAR_INSERTION = 1, TROCAR_SPACE = 2};

    enum Axis {X = 0, Y = 1, PITCH = 2, ROLL = 3, YAW = 4};

    enum Mapping {POSITION_VELOCITY = 0, POSITION_POSITION = 1};

    enum HUMAN_INPUT {JOYSTICK=0, FOOT=1};

  private:
    // ROS variables
    ros::NodeHandle _nh;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers declarations
    ros::Subscriber _subRobotPose[NB_ROBOTS];             // robot pose
    ros::Subscriber _subRobotTwist[NB_ROBOTS];            // robot twist
    ros::Subscriber _subForceTorqueSensor[NB_ROBOTS];     // force torque sensor
    ros::Subscriber _subDampingMatrix[NB_ROBOTS];         // Damping matrix of DS-impedance controller
    ros::Subscriber _subFootInterfacePose[NB_ROBOTS];
    ros::Subscriber _subFootInterfaceWrench[NB_ROBOTS];
    ros::Subscriber _subFootOutput[NB_ROBOTS];
    ros::Subscriber _subJoystick[NB_ROBOTS];
    ros::Subscriber _subCurrentJoints[NB_ROBOTS];

    // Publisher declaration
    ros::Publisher _pubDesiredTwist[NB_ROBOTS];           // Desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredOrientation[NB_ROBOTS];     // Desired orientation to DS-impedance controller
    ros::Publisher _pubFilteredWrench[NB_ROBOTS];         // Filtered measured wrench
    ros::Publisher _pubDesiredFootWrench[NB_ROBOTS];
    ros::Publisher _pubFootInput[NB_ROBOTS];
    ros::Publisher _pubDesiredWrench[NB_ROBOTS];
    ros::Publisher _pubNullspaceCommand[NB_ROBOTS];
    ros::Publisher _pubDesiredJoints[NB_ROBOTS];

    
    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    geometry_msgs::WrenchStamped _msgFilteredWrench;
    geometry_msgs::Wrench _msgDesiredFootWrench;
    custom_msgs::FootInputMsg_v2 _msgFootInput;
    std_msgs::Float32MultiArray _msgNullspaceCommand;   

    // Tool characteristics
    float _toolMass;                            // Tool mass [kg]
    float _toolOffsetFromEE[NB_ROBOTS];                   // Tool offset along z axis of end effector [m]             
    Eigen::Vector3f _toolComPositionFromSensor;   // Offset of the tool [m] (3x1)
    Eigen::Vector3f _gravity;                   // Gravity vector [m/s^2] (3x1)            

    // Tool state variables
    Eigen::Vector3f _x[NB_ROBOTS];                         // Position [m] (3x1)
    Eigen::Vector3f _xEE[NB_ROBOTS];                        // Position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];                         // Current quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];                     // Initial quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Matrix3f _wRb0[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Vector3f _v[NB_ROBOTS];                         // Velocity [m/s] (3x1)
    Eigen::Vector3f _w[NB_ROBOTS];                         // Angular velocity [rad/s] (3x1)
    Eigen::VectorXf _currentJoints[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _wrench[NB_ROBOTS];            // Wrench [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _wrenchBias[NB_ROBOTS];        // Wrench bias [N and Nm] (6x1)
    Eigen::Matrix<float,6,1> _filteredWrench[NB_ROBOTS];    // Filtered wrench [N and Nm] (6x1)
    Eigen::Vector3f _leftRobotOrigin;
    Eigen::Matrix3f _D[NB_ROBOTS];
    float _d1[NB_ROBOTS];

    // Task variables
    Eigen::Vector3f _xd[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector3f _xd0[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector4f _qd[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector4f _qdPrev[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];    // Desired angular velocity [rad/s] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];        // Desired modulated DS [m/s] (3x1)
    Eigen::Vector3f _xRobotBaseOrigin[NB_ROBOTS];
    Eigen::Vector4f _qRobotBaseOrigin[NB_ROBOTS];
    float _selfRotationCommand[NB_ROBOTS];
    Eigen::Matrix<float,NB_AXES_JOYSTICK,1> _joyAxes[NB_ROBOTS];
    uint32_t _joystickSequenceID[NB_ROBOTS];
    int _wrenchCount[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _nullspaceWrench[NB_ROBOTS];
    Eigen::Matrix<float,7,1> _nullspaceCommand[NB_ROBOTS];
    int _sphericalTrocarId[NB_ROBOTS];
    RobotMode _robotMode[NB_ROBOTS];
    Mapping _mapping[NB_ROBOTS];


    Eigen::Vector3f _trocarPosition[NB_ROBOTS];
    Eigen::Vector3f _trocarOrientation[NB_ROBOTS];
    Eigen::Vector3f _rEETrocar[NB_ROBOTS];
    Eigen::Vector3f _rEERCM[NB_ROBOTS];
    Eigen::Vector3f _xRCM[NB_ROBOTS];
    Eigen::Vector3f _xdEE[NB_ROBOTS];
    Eigen::Vector3f _fxk[NB_ROBOTS];
    Eigen::Vector3f _fx[NB_ROBOTS];
    Eigen::VectorXf _beliefs[NB_ROBOTS];
    Eigen::VectorXf _dbeliefs[NB_ROBOTS];
    Eigen::VectorXf _beliefsC;
    Eigen::VectorXf _dbeliefsC;


    // Booleans
    bool _useRobot[NB_ROBOTS];
    bool _useSim;
    bool _useJoystick;
    bool _firstRobotPose[NB_ROBOTS];       // Monitor the first robot pose update
    bool _firstRobotTwist[NB_ROBOTS];      // Monitor the first robot twist update
    bool _stop;                            // Check for CTRL+C
    bool _firstRobotBaseFrame[NB_ROBOTS];
    bool _alignedWithTrocar[NB_ROBOTS];
    bool _firstJoystick[NB_ROBOTS];
    bool _trocarsRegistered[NB_ROBOTS];
    bool _firstWrenchReceived[NB_ROBOTS];       // Monitor first force/torque data update
    bool _wrenchBiasOK[NB_ROBOTS];              // Check if computation of force/torque sensor bias is OK
    bool _firstDampingMatrix[NB_ROBOTS];        // Monitor first damping matrix update
    bool _firstFootOutput[NB_ROBOTS];
    bool _firstJointsUpdate[NB_ROBOTS];   
    bool _inputAlignedWithOrigin[NB_ROBOTS];
    bool _firstSphericalTrocarFrame[NB_ROBOTS];
    bool _firstPillarsFrame[4];
    bool _usePredefinedTrocars;


    // Foot interface variables
    Eigen::Matrix<float,5,1> _footPose[NB_ROBOTS];
    Eigen::Vector3f _footPosition[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footWrench[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footTwist[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _desiredFootWrench[NB_ROBOTS];   // Filtered wrench [N and Nm] (6x1)
    uint32_t _footInterfaceSequenceID[NB_ROBOTS];
    int _footState[NB_ROBOTS];
    float _xyPositionMapping;
    float _zPositionMapping;
    Eigen::Vector3f _vdFoot[NB_ROBOTS];
    Eigen::Vector3f _xdFoot[NB_ROBOTS];
    Eigen::Vector3f _FdFoot[NB_ROBOTS];
    Eigen::Matrix<float,5,1> _footOffset[NB_ROBOTS];
    Eigen::Vector3f _xdTool[NB_ROBOTS];
    Eigen::VectorXf _ikJoints[NB_ROBOTS];
    float _filteredForceGain;   // Filtering gain for force/torque sensor



    float _velocityLimit;       // Velocity limit [m/s]
    Eigen::Vector3f _xdOffset[NB_ROBOTS];
    Eigen::Vector3f _vdOffset[NB_ROBOTS];

    HUMAN_INPUT _humanInput;
    
    // Other variables
    uint32_t _sequenceID[NB_ROBOTS];
    std::string _fileName;
    std::ifstream _inputFile;
    std::ofstream _outputFile;

    tf::TransformListener _lr;
    tf::StampedTransform _transform;

    float _kxy;     
    float _dxy;     
    float _kphi;        
    float _dphi; 

    TROCAR_CONSTRAINT_STRATEGY _strategy;

    QpSolverRCM _qpSolverRCM;
    CvxgenSolverRCM _cvxgenSolverRCM;

    Eigen::VectorXi _pillarsId;
    Eigen::MatrixXf _pillarsPosition;

    std::mutex _mutex;
    static SurgicalTask* me;

    pthread_t _thread;
    bool _startThread;

    Eigen::Vector3f bou;
    Eigen::Vector3f _vdC;

  public:

    // Class constructor
    SurgicalTask(ros::NodeHandle &n, double frequency);

    // Initialize node
    bool init();

    // Run node
    void run();

  private:
    
    // Callback called when CTRL is detected to stop the node
    static void stopNode(int sig);

    bool allSubscribersOK();

    bool allFramesOK();

    void receiveFrames();

    void computeCommand();

    void humanInputTransformation();

    void updateTrocarInformation(int r);

    void selectRobotMode(int r);

    void trocarSelection(int r);

    void trocarInsertion(int r);

    void trocarSpace(int r);

    void computeDesiredFootWrench();
    // Compute desired orientation
    void computeDesiredOrientation();

    void pillarsAdaptation(int r);
    
    // Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);

    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    void updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg, int k); 

    // Callback to update the robot wrench (force/torque sensor data)
    void updateRobotWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg, int k);

    // Callback to update damping matrix form the DS-impedance controller
    void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg, int k); 

    void updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int k);

    void updateFootOutput(const custom_msgs::FootOutputMsg_v2::ConstPtr& msg, int k);

    void footPositionMapping();
    // Callback to update damping matrix form the DS-impedance controller
    // void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

    void pseudo_inverse(Eigen::Matrix3f &M_, Eigen::Matrix3f &M_pinv_);

    static void* startIkLoop(void* ptr);
    void ikLoop();

    void registerTrocars();


    // void footDataTransformation();
    // void positionPositionMapping();
    // void positionVelocityMapping();
    // void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k);
};


#endif
