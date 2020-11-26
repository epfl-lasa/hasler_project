#ifndef __endoscopeModifier_H__
#define __endoscopeModifier_H__

#include <ros/ros.h>
#include <ros/package.h>
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
#include "endoscope_feedback/SurgicalTaskStateMsg.h"

static const std::string OPENCV_WINDOW = "Image window";

#include "time.h"

using namespace std;
using namespace Eigen;
using namespace cv;

class endoscopeModifier {

public:
  enum tool_Types{ENDOSCOPE_T, GRASPER_T, NB_TOOLS_TYPES};
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
  
  tool_States _toolState[NB_TOOLS_TYPES];     
  
  toolH_States _toolHState[NB_TOOLS_TYPES];   

  // ros::NodeHandle _nh;
  image_transport::ImageTransport* _it;
  image_transport::Subscriber _myImageSub;
  image_transport::Publisher _myOverlayedImagePub;
  ros::Subscriber _surgicalTaskStateSub;
    
  // METHODS
public:
  endoscopeModifier(ros::NodeHandle &nh, float frequency);
  ~endoscopeModifier();
  

  bool init();
  void run();
private:

  cv_bridge::CvImagePtr _myCVPtr; // Original
  cv_bridge::CvImagePtr _myCVPtrCopy; // Original

  //cv
    cv::Mat _warning_icon_resized;

  //Eigen
  
    Array2f _feedIMGDims;
    Array2i _toolTextCoord[NB_TOOLS_TYPES];

    

    Matrix<bool,NB_AXIS_TOOL,1> _toolDOFEnable[NB_TOOLS_TYPES];
    
  //Flag

    bool _flagImageReceivedOnce;
    volatile bool _flagImageReceived;
    volatile bool _flagSurgicalTaskStateReceived;
    void loadIcons();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void changeAndPublishImage();
    void readSurgicalTaskState(const endoscope_feedback::SurgicalTaskStateMsgConstPtr& msg);
    void drawTransparency(cv::Mat frame, cv::Mat transp, int xPos, int yPos);

    void putTextForTool(uint toolN_ ,tool_States toolState_, cv::Point point_, cv::Scalar color_);
    static void stopNode(int sig);


    int _iconHeight;  
  // Utils
    cv::Point2i eigenV2iToCv(Eigen::Vector2i eigenVec2i_);
    
};
#endif // __endoscopeModifier_H__
