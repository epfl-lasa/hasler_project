#!/bin/bash

if grep -q "ethercat" "${5}"; then
    # setcap does not support symbolic links, so a potential symbolic link has to be resolved first.
    resolved_symlink=$(readlink -f ${8})

    printf ${1}

    # setcap using password
    echo ${9} | sudo -S setcap cap_net_raw+ep ${resolved_symlink}

    # Update the links and cache to the shared catkin libraries.
    # See https://stackoverflow.com/questions/9843178/linux-capabilities-setcap-seems-to-disable-ld-library-path
    sudo ldconfig /opt/ros/$ROS_DISTRO/lib
fi
# launch the node
roslaunch rokubimini_cosmo rokubimini.launch standalone:="${2}" time_step:="${3}" num_spinner:="${4}" rokubimini_setup_file:="${5}" sensor_config_name:="${6}" ros_publishers_file:="${7}"
