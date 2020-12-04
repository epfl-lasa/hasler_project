#! /bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
# remove cmake_code_coverage and cmake_clang_tools

blacklist=("cmake_code_coverage" "cmake_clang_tools")

CATKIN_WS=/root/catkin_ws
cd $CATKIN_WS
for file in $(find -name "package.xml"); do
	for word in ${blacklist[@]}; do
		if grep -q $word $file; then
			sed -e "s/<test_depend>$word<\/test_depend>//g" -i.backup $file
		fi
	done
done

mkdir $CATKIN_WS/packages

packages=(kindr any_measurements message_logger any_worker signal_handler yaml_tools rokubimini rokubimini_bus_manager rokubimini_serial soem rokubimini_ethercat rokubimini_factory rokubimini_manager param_io any_node any_msgs cosmo cosmo_ros rokubimini_msgs kindr_ros any_measurements_ros rokubimini_ros rokubimini_cosmo rokubimini_sdk)

rosdeps_file="/etc/ros/rosdep/sources.list.d/51-my-packages.list"
sudo rm $rosdeps_file
for package_name in ${packages[@]}; do
	roscd $package_name && rm -rf debian/ obj-x86_64-linux-gnu/ && bloom-generate rosdebian --os-name ubuntu --os-version bionic --ros-distro melodic && fakeroot debian/rules binary && dpkg -i "../ros-melodic-${package_name//_/-}"*.deb && cp "../ros-melodic-${package_name//_/-}"*.deb $CATKIN_WS/packages && touch rosdep.yaml && echo "$package_name:" >rosdep.yaml && echo "  ubuntu: [ros-melodic-${package_name//_/-}]" >>rosdep.yaml && sudo echo "yaml file://$(pwd)/rosdep.yaml" >>$rosdeps_file && rosdep update
done

# kindr any_measurements message_logger any_worker signal_handler yaml_tools rokubimini rokubimini_bus_manager rokubimini_serial soem rokubimini_ethercat rokubimini_factory rokubimini_manager param_io any_node any_msgs cosmo cosmo_ros rokubimini_msgs kindr_ros any_measurements_ros rokubimini_ros rokubimini_cosmo rokubimini_sdk
