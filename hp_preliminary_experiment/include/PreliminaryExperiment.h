#ifndef __PRELIMINARY_EXPERIMENT_H__
#define __PRELIMINARY_EXPERIMENT_H__


#include <fstream>
#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <dynamic_reconfigure/server.h>
#include "hp_preliminary_experiment/preliminaryExperiment_paramsConfig.h"

#define NB_MARKERS 6
#define CALIBRATION_COUNT 100

class PreliminaryExperiment
{
	private:


    // ROS variables
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers and publishers declaration
    ros::Subscriber _subRealPose;           // Subscribe to robot current pose
    ros::Subscriber _subRealTwist;          // Subscribe to robot current twist
    ros::Subscriber _subForceTorqueSensor;  // Subscribe to robot current pose
    ros::Subscriber _subOptitrackHip;
    ros::Subscriber _subOptitrackThigh;
    ros::Subscriber _subOptitrackKnee;
    ros::Subscriber _subOptitrackTibia;
    ros::Subscriber _subOptitrackHeel;
    ros::Subscriber _subOptitrackAnkle;
    ros::Subscriber _subOptitrackToe;
    ros::Publisher _pubDesiredTwist;        // Publish desired twist
    ros::Publisher _pubDesiredOrientation;  // Publish desired orientation
    ros::Publisher _pubMarker;              // Publish marker objects
    ros::Publisher _pubTaskAttractor;       // Publish task attractor

    // Subsciber and publisher messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Pose _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
    visualization_msgs::Marker _msgSurfaceMarker;
    geometry_msgs::PointStamped _msgTaskAttractor;

    // Tool variables
    float _loadMass = 0.132f;
    Eigen::Vector3f _loadOffset;
    Eigen::Vector3f _gravity;
    float _toolOffset = 0.114f;
    // float _toolOffset = 0.0f;

    // End effector state variables
    Eigen::Vector3f _x;              // Current position [m] (3x1)
    Eigen::Vector4f _q;              // Current end effector quaternion (4x1)
    Eigen::Matrix3f _wRb;            // Current rotation matrix (3x3)
    Eigen::Matrix<float,6,1> _twist; // Current end effector linear/angular velocity [m/s|rad/s](6x1)

    // End effector desired variables
    Eigen::Vector3f _vd;        // Desired velocity [m/s] (3x1)
    Eigen::Vector3f _xd;        // Desired position [m] (3x1)
    Eigen::Vector4f _qd;        // Desired end effector quaternion (4x1)
    Eigen::Vector3f _omegad;    // Desired angular velocity [rad/s] (3x1)

    Eigen::Matrix<float,3,NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,NB_MARKERS,1> _markersTracked;
    enum MarkersID {HIP = 6, THIGH = 5, KNEE = 4, TIBIA = 3, ANKLE = 2, HEEL = 1, TOE = 0};

    // Boolean variables
    bool _firstRobotPoseReceived;  // Monitor the first robot pose update
    bool _allMarkersPositionReceived;
    bool _stop;
    bool _calibrationOK;
    bool _doCalibration;

    uint16_t _markersCount;
    uint16_t _calibrationCount;

    uint32_t _currentSequenceID;

    std::vector<Eigen::Vector3f> _planeData;

    // Other variables
    static PreliminaryExperiment* me;

    std::mutex _mutex;

    std::ofstream _outputFile;    // File used to log data

    // Dynamic reconfigure (server+callback)
    dynamic_reconfigure::Server<hp_preliminary_experiment::preliminaryExperiment_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<hp_preliminary_experiment::preliminaryExperiment_paramsConfig>::CallbackType _dynRecCallback;


  public:
  
    PreliminaryExperiment(ros::NodeHandle &n, double frequency);

    bool init();

	void run();

    void computePlane();
    
    private:
        
    static void stopNode(int sig);

    void calibration();
        
    void computeAngles();

    void publishData();

    void logData();

    void addPlaneFittingData();


    void updateRobotPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateRobotTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void updateToePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateHeelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateAnklePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateTibiaPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateKneePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateThighPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateHipPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    uint16_t checkTrackedMarker(float a, float b);

    Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

    void dynamicReconfigureCallback(hp_preliminary_experiment::preliminaryExperiment_paramsConfig &config, uint32_t level);


};


#endif // __PRELIMINARY_EXPERIMENT_H__
