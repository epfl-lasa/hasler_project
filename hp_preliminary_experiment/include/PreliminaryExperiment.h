#ifndef __PRELIMINARY_EXPERIMENT_H__
#define __PRELIMINARY_EXPERIMENT_H__


#include <fstream>
#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include <Eigen/Eigenvalues> 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
// #include "hp_preliminary_experiment/preliminaryExperiment_paramsConfig.h"

#define TOTAL_NB_MARKERS 6
#define AVERAGE_COUNT 100

class PreliminaryExperiment
{
public:

    enum ExecutionMode {CALIBRATION = 0, GAME = 1};

    enum TrackingMode {TOE_ONLY = 0, ALL_JOINTS = 1}; 
    
    enum FittingMethod {PLANE = 0, SPHERE = 1};


  private:

    enum MarkersID {HIP = 6, THIGH = 5, KNEE = 4, TIBIA = 3, ANKLE = 2, HEEL = 1, TOE = 0};



    struct planeCalibrationResult
    {
      float c;
      Eigen::Vector3f n;
      Eigen::Vector3f u;
      Eigen::Vector3f v;
      Eigen::Vector3f Pcenter;
    };

    struct sphereCalibrationResult
    {
      Eigen::Vector3f center;
      float radius;
      float phiMean;
      float thetaMean;
      float arcLengthX;
      float arcLengthY;
    };

    // ROS variables
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers and publishers declaration
    ros::Subscriber _subOptitrackHip;
    ros::Subscriber _subOptitrackThigh;
    ros::Subscriber _subOptitrackKnee;
    ros::Subscriber _subOptitrackTibia;
    ros::Subscriber _subOptitrackHeel;
    ros::Subscriber _subOptitrackAnkle;
    ros::Subscriber _subOptitrackToe;
    ros::Publisher _pubChaserPose;

    // Subsciber and publisher messages declaration
    geometry_msgs::PoseStamped _msgChaserPose;

    // Optitrack tracking variables
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,TOTAL_NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,TOTAL_NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,TOTAL_NB_MARKERS,1> _markersTracked;
    Eigen::Matrix3f _R;

    // Boolean variables
    bool _allMarkersPositionReceived;
    bool _stop;
    bool _initializationOK;
    bool _calibration;
    bool _facingScreen;

    // Calibration variables
    planeCalibrationResult _pcr;
    sphereCalibrationResult _scr;
    FittingMethod _fittingMethod;
    std::vector<Eigen::Vector3f> _calibrationData;

    // Game variables
    Eigen::Vector3f _chaserPosition;

    // Other variables
    std::string _subjectName;
    ExecutionMode _executionMode;
    TrackingMode _trackingMode;
    uint16_t _nbMarkers;
    uint16_t _markersCount;
    uint16_t _averageCount;
    uint32_t _currentSequenceID;

    static PreliminaryExperiment* me;
    std::mutex _mutex;

    std::ofstream _outputFile;   // File used to write calibration data and results
    std::ifstream _inputFile;    // File used to read calibration results


  public:
  
    PreliminaryExperiment(ros::NodeHandle &n, double frequency, std::string subjectName, ExecutionMode executionMode, TrackingMode trackingMode, FittingMethod fittingMethod);

    bool init();

    void run();

    void planeLeastSquareFitting();

    void planeEigenSolverFitting();

    void sphereLeastSquareFitting();
    
  private:
        
    static void stopNode(int sig);

    void initializeData();
        
    void computeAngles();

    void computeChaserPose();

    void publishData();

    void logCalibrationData();

    void logCalibrationResult();

    void addSurfaceFittingData();

    void updateToePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateHeelPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateAnklePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateTibiaPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateKneePose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateThighPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void updateHipPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    uint16_t checkTrackedMarker(float a, float b);

};


#endif // __PRELIMINARY_EXPERIMENT_H__
