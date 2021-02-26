#!/bin/bash

rostopic pub /left_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[2.1832588923901586,-1.2877716545307454,-1.9329418736308899,-1.786667766046544,-1.2787036468434687,1.8990075405441846,0.5628668760134472]"" -1

# exit gracefully by returning a status 
# exit 0

