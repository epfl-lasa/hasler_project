  #include "endoscopeModifier.h"



 const Eigen::Array2f R_TOOL_TXT_FB[] = { Eigen::Array2f(20.0f/480.0f,30.0f/640.0f), 
                                          Eigen::Array2f(410.0f/480.0f,30.0f/640.0f)};

 const float R_WARNING_ICON = 50.0f/480.0f;
  
 const cv::Scalar COLOR_TOOL_TXT[] = {cv::Scalar(255,128,0), 
                                      cv::Scalar(0,128,255)};
 
endoscopeModifier *endoscopeModifier::me = NULL;

char const *toolState_Names[]{"insertion mode", "moving inside"};
char const *toolID_Names[]{"e: ", "g: "};

endoscopeModifier::endoscopeModifier(ros::NodeHandle &nh, float frequency)
    : _nh(nh), _loopRate(frequency), _dt(1.0f / frequency){
    me = this;
    _it = new image_transport::ImageTransport(_nh);
    _stop = false; 
    for (size_t i = 0; i < NB_TOOLS_TYPES; i++)
    {
      _toolDOFEnable[i].setZero();  
      _toolTextCoord[i].setZero();
      _toolState[i]=INSERTION_STATE;
    }
       
    _feedIMGDims.setZero();
    _flagImageReceivedOnce=false;
    _flagImageReceived=false;
    _flagSurgicalTaskStateReceived = true;
    _iconHeight=50;
  
}

endoscopeModifier::~endoscopeModifier()
{
  cv::destroyWindow(OPENCV_WINDOW);
  delete(_it);
}


bool endoscopeModifier::init() 
{
  _surgicalTaskStateSub = _nh.subscribe<endoscope_feedback::SurgicalTaskStateMsg>( "/surgicalTaskState"
    , 1, boost::bind(&endoscopeModifier::readSurgicalTaskState, this, _1),
    ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());
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

    if (!_flagImageReceivedOnce)
    {
      if (_myCVPtr->image.rows > 60 && _myCVPtr->image.cols > 60)
      {
         _feedIMGDims<<_myCVPtr->image.rows, _myCVPtr->image.cols;
         for (size_t i = 0; i < NB_TOOLS_TYPES; i++)
         {
            _toolTextCoord[i] = (_feedIMGDims * R_TOOL_TXT_FB[i]).cast<int>();
         }
        loadIcons();
      }
      else
      {
        ROS_ERROR("The image received is corrupted (less than 60x60 pixels)");
      }
      _flagImageReceivedOnce=true;
    }

    _flagImageReceived=true;

}

void endoscopeModifier::changeAndPublishImage()
{
  // Information on the video stream
    if (_myCVPtr->image.rows > 60 && _myCVPtr->image.cols > 60)
    cv::Size sourceImgSize = _myCVPtr->image.size();
    _myCVPtrCopy = _myCVPtr;
    for (size_t i = 0; i < NB_TOOLS_TYPES; i++)
    {
      putTextForTool(i, _toolState[i],eigenV2iToCv(_toolTextCoord[i]),COLOR_TOOL_TXT[i]);
      if(_toolState[i]==INSERTION_STATE)
      {
        drawTransparency(_myCVPtrCopy->image,_warning_icon_resized,_toolTextCoord[i].x() + 100,_toolTextCoord[i].y());
      }
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, _myCVPtrCopy->image);
    cv::waitKey(3);

    // Output modified video stream
    _myOverlayedImagePub.publish(_myCVPtrCopy->toImageMsg());
}
void endoscopeModifier::readSurgicalTaskState(const endoscope_feedback::SurgicalTaskStateMsgConstPtr& msg)
{
    for (size_t i = 0; i < NB_TOOLS_TYPES; i++)
    {
      _toolState[i] = (tool_States) msg->robotMode[i];
    }
    
    if (!_flagSurgicalTaskStateReceived)
    {
      _flagSurgicalTaskStateReceived=true;
    }    
}

void endoscopeModifier::drawTransparency(cv::Mat frame, cv::Mat transp, int xPos, int yPos) {
    cv::Mat mask;
    vector<cv::Mat> layers;

    cv::split(transp, layers); // seperate channels
    cv::Mat rgb[3] = { layers[0],layers[1],layers[2] };
    mask = layers[3]; // png's alpha channel used as mask
    cv::merge(rgb, 3, transp);  // put together the RGB channels, now transp insn't transparent 
    transp.copyTo(frame.rowRange(yPos, yPos + transp.rows).colRange(xPos, xPos + transp.cols), mask);
}


void endoscopeModifier::loadIcons()
{
    // Load warning icon
    cv::Mat warning_icon_raw = cv::imread(ros::package::getPath("endoscope_feedback")+"/src/img/warning.png",cv::IMREAD_UNCHANGED);
    cv::Mat warning_icon_to_blend;  
    Array2i sz_ = (_feedIMGDims * R_WARNING_ICON).cast<int>();
    cv::resize(warning_icon_raw,_warning_icon_resized,cv::Size(sz_.x(),sz_.y()));    
}

void endoscopeModifier::putTextForTool(uint toolN_ ,tool_States toolState_, cv::Point point_, cv::Scalar color_)
{
  cv::putText(_myCVPtrCopy->image, 
         std::string(toolID_Names[toolN_])+std::string(toolState_Names[toolState_]),
         point_, // Coordinates
         cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
         1.0, // Scale. 2.0 = 2x bigger
         color_, // BGR Color
         1); // Line Thickness (Optional)
         // cv::CV_AA); // Anti-alias (Optional)
}

cv::Point2i endoscopeModifier::eigenV2iToCv(Eigen::Vector2i eigenVec2i_)
{
  return cv::Point2i(eigenVec2i_.x(),eigenVec2i_.y());
}