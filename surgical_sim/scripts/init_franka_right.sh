#!/bin/bash

rostopic pub /right_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[0.09470964871310894,0.4541689524406385,-0.008879377716321635,-2.1312375509876818,-0.01602826635779042,2.685868134574921,-0.04145428793537498]"" -1

# exit gracefully by returning a status 
# exit 0

