  #include "endoscopeModifier.h"
  
endoscopeModifier *endoscopeModifier::me = NULL;

char const *toolState_Names[]{"insertion mode", "moving inside"};

endoscopeModifier::endoscopeModifier(ros::NodeHandle &nh, float frequency)
    : _nh(nh), _loopRate(frequency), _dt(1.0f / frequency){
    me = this;
    _it = new image_transport::ImageTransport(_nh);
    _stop = false;    
    _endoscopeDOFEnable.setZero();
    _grasperDOFEnable.setZero();
    _flagImageReceived=false;
    _endoscopeState=INSERTION_STATE;
    _grasperState=INSIDE_TROCAR_STATE;
}

endoscopeModifier::~endoscopeModifier()
{
  cv::destroyWindow(OPENCV_WINDOW);
  // delete(_it);
}


bool endoscopeModifier::init() 
{
  _myImageSub = _it->subscribe("/cv_camera/image_raw", 1,
      &endoscopeModifier::imageCb, this);
  _myOverlayedImagePub = _it->advertise("/image_converter/output_video", 1);
  cv::namedWindow(OPENCV_WINDOW);

  // Subscriber definitions
  signal(SIGINT, endoscopeModifier::stopNode);

  if (_nh.ok()) {
    ros::spinOnce();
    ROS_INFO("The node for overlaying information in the endoscope image is about to start ");
    return true;
  }
  else {
    ROS_ERROR("The ros node has a problem.");
    return false;
  }
}

void endoscopeModifier::stopNode(int sig) { me->_stop = true;  }

void endoscopeModifier::run() {

  while (!_stop) {
      
      if (_flagImageReceived)
      {  
        changeAndPublishImage();
        _flagImageReceived=false;
      }

      
    ros::spinOnce();
    _loopRate.sleep();
  }

  ROS_INFO("The endoscope image modifier overlay node stopped");
  ros::spinOnce();
  _loopRate.sleep();
  ros::shutdown();
}

void endoscopeModifier::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      _myCVPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    _flagImageReceived=true;

}

void endoscopeModifier::changeAndPublishImage()
{
  // Information on the video stream
    if (_myCVPtr->image.rows > 60 && _myCVPtr->image.cols > 60)

    _myCVPtrCopy = _myCVPtr;

    cv::Size sourceImgSize = _myCVPtr->image.size();

    cv::putText(_myCVPtrCopy->image, 
            "e:"+std::string(toolState_Names[_endoscopeState]),
            cv::Point(20,50), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,128,0), // BGR Color
            1); // Line Thickness (Optional)
            // cv::CV_AA); // Anti-alias (Optional)
    

    cv::putText(_myCVPtrCopy->image, 
            "g:"+std::string(toolState_Names[_grasperState]),
            cv::Point(420,50), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            1.0, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,128,255), // BGR Color
            1); // Line Thickness (Optional)
            // cv::CV_AA); // Anti-alias (Optional)
    
    // Load warning icon
    cv::Mat baseImage = _myCVPtr->image;
    cv::Mat warning_icon_resized;
    cv::Mat warning_icon_to_blend;
    
    cv::resize(cv::imread("img/warning.png"),warning_icon_resized,cv::Size(128,128));
    warning_icon_resized.copyTo(baseImage(cv::Rect(50,50,warning_icon_resized.cols, warning_icon_resized.rows)));
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, _myCVPtrCopy->image);
    cv::waitKey(3);

    // Output modified video stream
    _myOverlayedImagePub.publish(_myCVPtrCopy->toImageMsg());
}
