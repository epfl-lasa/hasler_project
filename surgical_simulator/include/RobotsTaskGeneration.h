#ifndef __ROBOTS_TASK_GENERATION_H__
#define __ROBOTS_TASK_GENERATION_H__

#include <signal.h>
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
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
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include <dynamic_reconfigure/server.h>
#include "surgical_simulator/robotsTaskGeneration_paramsConfig.h"
#include "Eigen/Eigen"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "sensor_msgs/Joy.h"
#include "visualization_msgs/Marker.h"
#include "custom_msgs/FootOutputMsg.h"


#define NB_SAMPLES 50
#define AVERAGE_COUNT 100
#define NB_ROBOTS 2
#define FOOT_INTERFACE_X_RANGE_RIGHT 0.350
#define FOOT_INTERFACE_Y_RANGE_RIGHT 0.293
#define FOOT_INTERFACE_PHI_RANGE_RIGHT 35
#define FOOT_INTERFACE_X_RANGE_LEFT 0.350
#define FOOT_INTERFACE_Y_RANGE_LEFT 0.293
#define FOOT_INTERFACE_PHI_RANGE_LEFT 35
#define FOOT_INTERFACE_PSI_RANGE 45
#define NB_AXES_JOYSTICK 8

enum ROBOT {LEFT = 0, RIGHT = 1};

class RobotsTaskGeneration 
{
  public: 

    enum ROBOT {LEFT = 0, RIGHT = 1};

    enum TROCAR_CONSTRAINT_STRATEGY {VIRTUAL_RCM=0, PRIMARY_TASK=1};

  private:
    // ROS variables
    ros::NodeHandle _nh;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers declarations
    ros::Subscriber _subRobotPose[NB_ROBOTS];            // robot pose
    ros::Subscriber _subRobotTwist[NB_ROBOTS];           // robot twist
    ros::Subscriber _subJoystick[NB_ROBOTS];             // robot twist

    // Publisher declaration
    ros::Publisher _pubDesiredTwist[NB_ROBOTS];         // Desired twist to DS-impdedance controller
    ros::Publisher _pubDesiredOrientation[NB_ROBOTS];   // Desired orientation to DS-impedance controller
    ros::Publisher _pubMarker;                  // Marker (RVIZ) 

    
    // Subsciber and publisher messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    visualization_msgs::Marker _msgMarker;

    // Tool characteristics
    float _toolOffsetFromEE;                    // Tool offset along z axis of end effector [m]             

    // Tool state variables
    Eigen::Vector3f _x[NB_ROBOTS];                         // Position [m] (3x1)
    Eigen::Vector4f _q[NB_ROBOTS];                         // Current quaternion (4x1)
    Eigen::Vector4f _qinit[NB_ROBOTS];                     // Initial quaternion (4x1)
    Eigen::Matrix3f _wRb[NB_ROBOTS];                       // Orientation matrix (3x1)
    Eigen::Vector3f _v[NB_ROBOTS];                         // Velocity [m/s] (3x1)
    Eigen::Vector3f _w[NB_ROBOTS];                         // Angular velocity [rad/s] (3x1)


    // Task variables
    Eigen::Vector3f _xd[NB_ROBOTS];        // Desired position [m] (3x1)
    Eigen::Vector4f _qd[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector4f _qdPrev[NB_ROBOTS];        // Desired quaternion (4x1)
    Eigen::Vector3f _omegad[NB_ROBOTS];    // Desired angular velocity [rad/s] (3x1)
    Eigen::Vector3f _vd[NB_ROBOTS];        // Desired modulated DS [m/s] (3x1)
    Eigen::Vector3f _xTrocar[NB_ROBOTS];
    Eigen::Vector3f _xLeftRobotOrigin;
    Eigen::Vector4f _qLeftRobotOrigin;
    Eigen::Vector4f _qLeftCameraOrigin;
    Eigen::Matrix3f _rRl;
    Eigen::Matrix3f _rRc;
    Eigen::Matrix3f _rRcp;
    Eigen::Matrix3f _rRt[NB_ROBOTS];
    float _selfRotationCommand[NB_ROBOTS];
    Eigen::Matrix<float,NB_AXES_JOYSTICK,1> _joyAxes[NB_ROBOTS];

    // Booleans
    bool _firstRobotPose[NB_ROBOTS];       // Monitor the first robot pose update
    bool _firstRobotTwist[NB_ROBOTS];      // Monitor the first robot twist update
    bool _stop;                            // Check for CTRL+C
    bool _leftRobotOriginReceived;
    bool _leftTrocarFrameReceived;
    bool _rightTrocarFrameReceived;
    bool _leftCameraFrameReceived;
    bool _alignedWithTrocar[NB_ROBOTS];
    bool _firstJoystick;


    ros::Subscriber _subFootOutput[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footPose[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footPose0[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footSensor[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footWrench[NB_ROBOTS];
    Eigen::Matrix<float,6,1> _footTwist[NB_ROBOTS];
    Eigen::Vector3f _footPosition[NB_ROBOTS];
    uint32_t _footInterfaceSequenceID[NB_ROBOTS];
    bool _firstFootOutput[NB_ROBOTS];
    int _footState[NB_ROBOTS];

    // User variables
    float _velocityLimit;       // Velocity limit [m/s]
    Eigen::Vector3f _xdOffset[NB_ROBOTS];
    
    // Other variables
    uint32_t _sequenceID;
    std::string _fileName;
    std::ifstream _inputFile;
    std::ofstream _outputFile;

    tf::TransformListener _lr;
    tf::StampedTransform _transform;

    TROCAR_CONSTRAINT_STRATEGY _strategy;

    std::mutex _mutex;
    static RobotsTaskGeneration* me;

    // Dynamic reconfigure (server+callback)
    dynamic_reconfigure::Server<surgical_simulator::robotsTaskGeneration_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<surgical_simulator::robotsTaskGeneration_paramsConfig>::CallbackType _dynRecCallback;
    surgical_simulator::robotsTaskGeneration_paramsConfig _config;

  public:

    // Class constructor
    RobotsTaskGeneration(ros::NodeHandle &n, double frequency);

    // Initialize node
    bool init();

    // Run node
    void run();

  private:
    
    // Callback called when CTRL is detected to stop the node
    static void stopNode(int sig);

    bool allSubscribersOK();

    bool allFramesReceived();

    void receiveFrames();

    void alignWithTrocar();

    void trackTarget();

    void computeAttractors();

    // Compute modulated DS
    void computeModulatedDS();

    // Compute desired orientation
    void computeDesiredOrientation();
    
    // Log data to text file
    void logData();

    // Publish data to topics
    void publishData();
    
    // Callback to update the robot pose
    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg, int k);


    // Callback to update the robot twist
    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg, int k);

    void updateJoystick(const sensor_msgs::Joy::ConstPtr& msg, int k);

    void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k);

    void footPositionMapping();
    // Callback to update damping matrix form the DS-impedance controller
    // void updateDampingMatrix(const std_msgs::Float32MultiArray::ConstPtr& msg); 

    // Callback for dynamic reconfigure
    void dynamicReconfigureCallback(surgical_simulator::robotsTaskGeneration_paramsConfig &config, uint32_t level);

    void pseudo_inverse(Eigen::Matrix3f &M_, Eigen::Matrix3f &M_pinv_);

    // void footDataTransformation();
    // void positionPositionMapping();
    // void positionVelocityMapping();
    // void updateFootOutput(const custom_msgs::FootOutputMsg::ConstPtr& msg, int k);
};


#endif
