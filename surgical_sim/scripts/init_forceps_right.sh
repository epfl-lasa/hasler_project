#!/bin/bash

rostopic pub /right_tool/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[0.0,-0.6202494859926144,1.5707963267948974,0.10162325267042625,0.34595200291764794,0.3459524114712389]"" -1

# exit gracefully by returning a status 
# exit 0

