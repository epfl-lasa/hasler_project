#ifndef __endoscopeModifier_H__
#define __endoscopeModifier_H__

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen_conversions/eigen_msg.h>
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
#include "custom_msgs_gripper/SharedGraspingMsg.h"
#include "custom_msgs_gripper/GripperOutputMsg.h"

static const std::string OPENCV_WINDOW = "Image window";

#include "time.h"

using namespace std;
using namespace Eigen;
using namespace cv;

class endoscopeModifier {

public:
  enum tool_Types{ENDOSCOPE_ROBOT, gripper_ROBOT, gripper_HAND_R, gripper_HAND_L, NB_TOOLS};
  static const int NB_ROBOT_TOOLS =  NB_TOOLS-2; 
  enum tool_States {INSERTION_STATE, INSIDE_TROCAR_STATE};
  enum gripperA_State {A_POSITIONING_OPEN, A_GRASPING, A_HOLDING_GRASP, A_POSITIONING_CLOSE, A_FETCHING_OLD_GRASP, A_RELEASE_GRASP, NB_ACTIONS_gripper, NO_SHARED_CONTROL};
  enum tool_SuperNum_ID {TOOL_1, TOOL_2, TOOL_3, NB_SUPERNUM_TOOLS}; //Other tools apart from the endoscope
  enum toolH_States {HUMAN_IDLE, HUMAN_ENGAGED, HUMAN_CONFUSED}; 
  enum tool_Axis {Y_TOOL_AXIS,X_TOOL_AXIS,Z_TOOL_AXIS,ROLL_TOOL_AXIS,GRASPING_TOOL_AXIS, NB_AXIS_TOOL};

private:
  ros::NodeHandle _nh;
  ros::Rate _loopRate;
  
  float _dt;
  bool _stop;
  std::mutex _mutex;
  static endoscopeModifier *me;
  
  tool_States _toolState[NB_ROBOT_TOOLS];     
  
  toolH_States _toolHState[NB_ROBOT_TOOLS];   

  // ros::NodeHandle _nh;
  image_transport::ImageTransport* _it;
  image_transport::Subscriber _myImageSub;
  image_transport::Publisher _myOverlayedImagePub;
  ros::Subscriber _surgicalTaskStateSub;
  ros::Subscriber _sharedGraspingSub;
  ros::Subscriber _gripperOutputSub;

  gripperA_State _gripperStateSC;
    
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
    Array2i _toolStateTextCoord[NB_ROBOT_TOOLS];
    Array2i _gripperAStateTextCoord;
    
    Affine3d _allToolsPose[NB_TOOLS]; // Endoscope + Ex. Other 3 grippers

    Matrix<bool,NB_AXIS_TOOL,1> _toolDOFEnable[NB_ROBOT_TOOLS];
    
  //Flag

    bool _flagImageReceivedOnce;
    volatile bool _flagImageReceived;
    volatile bool _flagSurgicalTaskStateReceived;
    volatile bool _flagSharedGraspingMsgReceived;
    volatile bool _flagGripperOutputMsgReceived;

    endoscope_feedback::SurgicalTaskStateMsg _surgicalTaskMsg;
    custom_msgs_gripper::SharedGraspingMsg _sharedGraspingMsg;
    custom_msgs_gripper::GripperOutputMsg _gripperOutputMsg;

    void loadIcons();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void addToolStateFB();
    void addGraspingStateFB();
    void drawTransparency(cv::Mat frame, cv::Mat transp, int xPos, int yPos);

    void putTextForTool(uint toolN_ ,tool_States toolState_, cv::Point point_, cv::Scalar color_);
    void putTextForGripper(uint toolN_ ,gripperA_State gripperState_, cv::Point point_, cv::Scalar color_);
    
    void readSurgicalTaskState(const endoscope_feedback::SurgicalTaskStateMsgConstPtr& msg);
    void readSharedGrasping(const custom_msgs_gripper::SharedGraspingMsgConstPtr& msg);
    void readGripperOutput(const custom_msgs_gripper::GripperOutputMsgConstPtr& msg);
    static void stopNode(int sig);

  // Utils
    cv::Point2i eigenV2iToCv(Eigen::Vector2i eigenVec2i_);
    
};
#endif // __endoscopeModifier_H__
