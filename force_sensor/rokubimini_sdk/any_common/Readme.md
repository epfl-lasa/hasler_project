# ANY Common

## Overview

Robot-independent control framework, providing base classes, message and service definitions.

The packages contained have been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

Contact: leggedrobotics@ethz.ch

## Building

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/any_common/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/any_common/job/master/)

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/any_common/issues).

## Packages

This is only an overview. For more detailed documentation, please check the packages individually.

### any_description

Xacro snippets for Gazebo and urdf file generation.

### any_measurements

Container classes featuring [kindr] integration. Static memory sizes allow them to be used in shared memory.

### any_measurements_ros

Conversion functions between *any_measurements* and ROS msgs.

### any_msgs

Common ROS message and service definitions.

### any_state_estimator

Templated base class for robot state estimators.

### notification

Packages that allow to send notifications for different user interfaces (desktop, joystick,...).

