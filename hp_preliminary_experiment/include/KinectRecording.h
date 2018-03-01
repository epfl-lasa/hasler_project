#ifndef __KINECT_RECORDING_H__
#define __KINECT_RECORDING_H__


#include <fstream>
#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "sg_filter.h"

#define NB_MARKERS 4
#define AVERAGE_COUNT 100

class KinectRecording
{
	private:

    struct CalibrationResult
    {
      float c;
      Eigen::Vector3f n;
      Eigen::Vector3f u;
      Eigen::Vector3f v;
      Eigen::Vector3f Pcenter;
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
    enum MarkersID {PC = 3, PB = 2, PA = 1, TOE = 0};

    // Boolean variables
    bool _allMarkersReceived;
    bool _stop;
    bool _initializationOK;
    bool _calibration;
    bool _facingScreen;

    uint16_t _markerCount;
    uint16_t _averageCount;
    uint32_t _nbData;

    uint32_t _currentSequenceID;

    std::vector<Eigen::Vector3f> _surfaceData;

    Eigen::Vector3f _chaserPosition;

    // Other variables
    static KinectRecording* me;

    std::mutex _mutex;

    std::ofstream _outputFile;   // File used to write calibration data and results
    std::ifstream _inputFile;    // File used to read calibration results

    CalibrationResult _cr;

    Eigen::Matrix3f _R;

    SGF::SavitzkyGolayFilter _filter;

  public:
  
    KinectRecording(ros::NodeHandle &n, double frequency, bool calibration);

    bool init();

    void run();

    void computePlane();
    
  private:
        
    static void stopNode(int sig);

    void initializeData();
        
    void computeChaserPose();

    void publishData();

    void logCalibrationData();

    void logCalibrationResult();

    void addPlaneFittingData();

    void updateMarkersPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    uint16_t checkTrackedMarker(float a, float b);
};


#endif // __KINECT_RECORDING_H__
