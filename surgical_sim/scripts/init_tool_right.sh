#!/bin/bash

rostopic pub /right_tool/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[0.0,0.0,0.0,0.0,0.0,0.0]"" -1

# exit gracefully by returning a status 
# exit 0

