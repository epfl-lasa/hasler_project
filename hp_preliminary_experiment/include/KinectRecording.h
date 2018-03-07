#ifndef __KINECT_RECORDING_H__
#define __KINECT_RECORDING_H__


#include <fstream>
#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "sg_filter.h"

#define NB_MARKERS 4
#define AVERAGE_COUNT 100

class KinectRecording
{
  public:
    enum ExecutionMode {CALIBRATION = 0, GAME = 1};

    enum FittingMethod {PLANE = 0, SPHERE = 1};

	private:

    enum MarkersID {PC = 3, PB = 2, PA = 1, TOE = 0};

    struct planeCalibrationResult
    {
      float c;
      Eigen::Vector3f n;
      Eigen::Vector3f u;
      Eigen::Vector3f v;
      Eigen::Vector3f Pcenter;
      Eigen::Matrix3f R;

    };

    struct sphereCalibrationResult
    {
      Eigen::Vector3f center;
      float radius;
      float phiMean;
      float thetaMean;
      float arcLengthX;
      float arcLengthY;
      Eigen::Matrix3f R;

    };

    // ROS variables
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    float _dt;

    // Subscribers and publishers declaration
    ros::Subscriber _subKinectToe;
    ros::Publisher _pubChaserPose;

    // Subsciber and publisher messages declaration
    geometry_msgs::PoseStamped _msgChaserPose;

    
    Eigen::Matrix<float,3,NB_MARKERS> _markersPosition;
    Eigen::Matrix<float,3,NB_MARKERS> _markersPosition0;
    Eigen::Matrix<uint32_t,NB_MARKERS,1> _markersSequenceID;
    Eigen::Matrix<uint16_t,NB_MARKERS,1> _markersTracked;

    // Boolean variables
    bool _allMarkersReceived;
    bool _stop;
    bool _initializationOK;
    bool _useFiltering;

    // Calibration variables
    planeCalibrationResult _pcr;
    sphereCalibrationResult _scr;
    FittingMethod _fittingMethod;
    std::vector<Eigen::Vector3f> _calibrationData;


    Eigen::Matrix3f _R;

    // Game variables
    Eigen::Vector3f _chaserPosition;

    // Other variables
    std::string _subjectName;
    ExecutionMode _executionMode;
    SGF::SavitzkyGolayFilter _filter;
    uint16_t _markerCount;
    uint16_t _averageCount;
    uint32_t _currentSequenceID;

    static KinectRecording* me;
    std::mutex _mutex;

    std::ofstream _outputFile;   // File used to write calibration data and results
    std::ifstream _inputFile;    // File used to read calibration results

  public:
  
    KinectRecording(ros::NodeHandle &n, double frequency, std::string subjectName, ExecutionMode executionMode, FittingMethod fittingMethod);

    bool init();

    void run();
    
    void planeLeastSquareFitting();

    void planeEigenSolverFitting();

    void sphereLeastSquareFitting();

  private:
        
    static void stopNode(int sig);

    void initializeData();
        
    void computeChaserPose();

    void publishData();

    void logCalibrationData();

    void logCalibrationResult();

    void addSurfaceFittingData();

    void updateMarkersPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    uint16_t checkTrackedMarker(float a, float b);
};


#endif // __KINECT_RECORDING_H__
