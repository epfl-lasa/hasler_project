#ifndef __endoscopeModifier_H__
#define __endoscopeModifier_H__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LP_Filterd.h"
#include "Eigen/Eigen"
#include <signal.h>
#include <mutex>
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include "Utils_math.h"

static const std::string OPENCV_WINDOW = "Image window";

#include "time.h"

using namespace std;
using namespace Eigen;

class endoscopeModifier {

public:

  enum tool_States {INSERTION_STATE, INSIDE_TROCAR_STATE};
  enum toolH_States {HUMAN_IDLE, HUMAN_ENGAGED, HUMAN_CONFUSED}; 
  enum tool_Axis {Y_TOOL_AXIS,X_TOOL_AXIS,Z_TOOL_AXIS,ROLL_TOOL_AXIS,GRASPING_TOOL_AXIS, NB_AXIS_TOOL};

private:
  ros::NodeHandle _nh;
  ros::Rate _loopRate;
  
  float _dt;
  bool _stop;
  std::mutex _mutex;
  static endoscopeModifier *me;
  
  tool_States _endoscopeState;   
  tool_States _grasperState;   
  
  toolH_States _endoscopeHState;   
  toolH_States _grasperHState;   

  // ros::NodeHandle _nh;
  image_transport::ImageTransport* _it;
  image_transport::Subscriber _myImageSub;
  image_transport::Publisher _myOverlayedImagePub;

    
  // METHODS
public:
  endoscopeModifier(ros::NodeHandle &nh, float frequency);
  ~endoscopeModifier();
  

  bool init();
  void run();
private:

  cv_bridge::CvImagePtr _myCVPtr; // Original
  cv_bridge::CvImagePtr _myCVPtrCopy; // Original

  //Eigen
    Matrix<bool,NB_AXIS_TOOL,1> _endoscopeDOFEnable;
    Matrix<bool,NB_AXIS_TOOL,1> _grasperDOFEnable;
  //Flag

    volatile bool _flagImageReceived;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void changeAndPublishImage();
    static void stopNode(int sig);


  

};
#endif // __endoscopeModifier_H__