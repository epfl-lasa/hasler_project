#!/bin/bash

rostopic pub /left_panda/joint_position_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
data : "[2.280448433408692,-1.356741723314781,-2.022619694584435,-1.9466445165987398,-1.4273606401112975,2.012965945922268,0.8382446115600093]"" -1

# exit gracefully by returning a status 
# exit 0

