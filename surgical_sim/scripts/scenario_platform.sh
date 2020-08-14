#!/bin/bash
cd ~/catkin_hasler
source devel/setup.bash
roslaunch surgical_sim scenario.launch virtual_tool:=false control_input:=platform
