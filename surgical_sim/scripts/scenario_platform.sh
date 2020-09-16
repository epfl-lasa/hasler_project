#!/bin/bash
cd ~/catkin_hasler
source devel/setup.bash
export GAZEBO_MODEL_PATH=~/catkin_hasler/src/hasler_project/surgical_sim/model:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=~/catkin_hasler/devel/lib:$GAZEBO_PLUGIN_PATH
roslaunch surgical_sim scenario.launch virtual_tool:=false control_input:=platform
