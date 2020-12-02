
# SurgicalTaskStateMsg.msg
time surgicalTaskStamp

uint8[2] robotsToolState
geometry_msgs/Pose[4] allToolsPoseWRTImage
geometry_msgs/Twist[4] allToolsTwistWRTImage
geometry_msgs/Wrench[4] allToolsWrench
