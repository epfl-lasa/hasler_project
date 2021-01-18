#!/bin/bash

rostopic pub /right_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[-1.3,-0.1,0.2,-2.5,1.2,2.0,0.0,0.0,0.0]"" -1

rostopic pub /left_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data: "[1.3,-0.1,-0.2,-2.5,-1.2,2.0,0.0,0.0,0.0]"" -1

# exit gracefully by returning a status 
# exit 0

