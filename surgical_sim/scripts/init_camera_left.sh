#!/bin/bash

rostopic pub /left_tool/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[-0.09072372302531484,0.8903616344174168,0.0,0.13449672820206968]"" -1

# exit gracefully by returning a status 
# exit 0

