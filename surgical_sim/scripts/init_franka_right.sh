#!/bin/bash

rostopic pub /right_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[0.17209899484779,0.568427947966252,-0.18495719070604455,-2.1266516721705067,0.2686955792228609,2.7767888666502287,-0.36829199524356504]"" -1

# exit gracefully by returning a status 
# exit 0

