# Rokubi mini 2.0 - Force-Torque Sensor - README

## Overview

This software package will provide a driver and a ROS interface for the ethercat and serial version of the rokubi mini force-torque sensor.
This is at the moment just a skeleton to connect to rokubi mini devices and support the driver development of its firmware. 

**Authors(s):** Marco Pagnamenta, Markus Staeuble, Mike Karamousadakis

## Building

[![pipeline status](https://gitlab.com/botasys/rokubimini_sdk/badges/master/pipeline.svg)](https://gitlab.com/botasys/rokubimini_sdk/-/commits/master)

## Installation

### Building from Source

In order to use the `rokubimini_sdk` package, you need to download first the following dependencies:

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [any_common](https://bitbucket.org/leggedrobotics/any_common)
- [any_utils](https://bitbucket.org/leggedrobotics/any_utils)
- [cosmo](https://bitbucket.org/leggedrobotics/cosmo)
- [any_node](https://github.com/ANYbotics/any_node)
- [message_logger](https://github.com/ANYbotics/message_logger)
- [kindr](https://github.com/ANYbotics/kindr)
- [kindr_ros](https://github.com/ANYbotics/kindr_ros)
- [openethercat_soem](https://github.com/ANYbotics/SOEM)

The first three, namely `any_common`, `any_utils` and `cosmo` are provided inside the `rokubimini_sdk` package.

#### Building

To build the rokubimini_sdk from source, clone the latest version from this repository into your catkin workspace and compile the package using:

1. The `catkin_tools` package:
   
	```bash
	cd catkin_workspace/src
	git clone https://gitlab.com/botasys/rokubimini_sdk.git
	cd ../
	catkin build rokubimini_sdk
	```

2. If you don't have the `catkin_tools` package, you can still build from source with `catkin_make_isolated`. The only difference is that each package will be processed sequentially:

	```bash
	cd catkin_workspace/src
	git clone https://gitlab.com/botasys/rokubimini_sdk.git
	cd ../
	catkin_make_isolated --pkg rokubimini_sdk
	```

### Unit Tests

No unit tests so far.


## Packages

#### rokubimini

The core C++ library to interface one or multiple rokubimini devices. Contains abstract interfaces to start the communication.

#### rokubimini_ethercat

The ethercat implementation of rokubimini.

#### rokubimini_serial

The serial implementation of rokubimini.

#### rokubimini_manager

The manager of multiple rokubiminis, independently of their implementation (ethercat or serial).

#### rokubimini_bus_manager

An abstract class for managing a bus.

#### rokubimini_factory

The factory that creates each implementation based on the configuration file `config/setup.yaml`.

#### rokubimini_cosmo

Contains a ROS node that communicates with a manager that handles one or multiple rokubiminis (depends on the config file `setup.yaml`). This package contains config files `config/setup.yaml` and `config/rokubimini_sensor.yaml` to configure the communication and the sensor itself.

#### rokubimini_ros

The ROS implementation of rokubimini. Contains conversion traits to convert between ROS messages and the reading and command types used in rokubimini.

#### rokubimini_flashloader

The flashloader performs file over ethercat for firmware flashing.

### rokubimini_msgs

Containts the definitions of the ROS messages and services used for communication over ROS.

## Usage

You need to have sudo rights to the network card. Use the provided setcap launch files to achieve this:

```bash
  roslaunch rokubimini_cosmo  rokubimini_wrapper.launch password:=<your_sudo_password>
```

or you could run it without using your password (and without accessing the network card with sudo credentials though), with:

```bash
  roslaunch rokubimini_cosmo  rokubimini.launch
```

### Configuration

The following parameters can be set in `setup.yaml`:

- name: The name of the rokubimini devices. Needed to handle multiple devices on the master side,
- configuration_file: The relative path to the configuration file of the sensor
- product_code: The product code of each device.
- For the ethercat devices:
  - ethercat_bus: The ethercat bus containing the sensor. Is the ethernet adapter name on which the sensor is connected to, as indicated in the ifconfig command output.
  - ethercat_address: The address of the slave. Ethercat addresss are distributed to each slave incrementally. The slave closest to the master has address 1, the second one 2 etc.
  - ethercat_pdo_type: The PDO type used for communication. Details about different PDO types can be found in the object dictionary. Currently, we only support Z
- For the serial devices:
  - port: The serial port to connect to communicate with the serial sensor.
  - baud_rate: The baud rate used for the serial communication.

The following parameters can be set in `setup.yaml`:

- max_command_age: Commands that are "older" than this age get discarded
- auto_stage_last_command: The last command will be staged automatically with calling updateSendStagedCommands() in the rokubimini manager
- set_reading_to_nan_on_disconnect: Sets the reading to nan when the sensor disconnects
- imu_acceleration_range: 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
- imu_angular_rate_range: 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s
- imu_acceleration_filter: (cut-off Freq) 1 = 460Hz, 2 = 184Hz, 3 = 92Hz, 4 = 41Hz, 5 = 21Hz, 6 = 10Hz, 7 = 5Hz
- imu_angular_rate_filter: (cut-off Freq) 3 = 184Hz, 4 = 92Hz, 5 = 41Hz, 6 = 21Hz, 7 = 10Hz, 8 = 5Hz
- sinc_filter_size: (cut-off Freq high/low@sampling Freq) 51 = 1674/252Hz@1000Hz, 64 = 1255/189Hz@800Hz, 128 = 628/94.5hz@400Hz, 205 = 393/59.5Hz@250Hz 256 = 314/47.5Hz@200Hz, 512 = 157/23.5@100Hz
- fir_disable: false = low cut-off frequency, true = high cut-off frequency from the above result e.g. for sinc filter_size: 48 and fir_disable: 1 you get cut-off freq 1674Hz@1000Hz sample rate
- chop_enable: should be always false
- fast_enable: (only applies if fir_disable is false) True = will result in low cut-off frequency but would be still able to catch step impulses of high cut-off frequncy
- calibration_matrix_active: Use the calibration matrix to compute sensor output
- temperature_compensation_active: Compensate drift due to temperature !not supported yet!
- imu_active: Chooses which IMU type is active: 0 = no imu active, 1 = internal IMU active, 2 = external IMU active (if available), 3 = both IMUs active
- coordinate_system_active: Set a user defined coordinate system
- inertia_compensation_active: Enables compensation due to inertia effect
- orientation_estimation_active: Enables orientation estimation and outputs a quaternion

### Subscribed Topics

None

### Published Topics

* **`/rokubimini/<name>/ft_sensor_readings`** (`rokubimini_msgs/Reading`)
* **`/rokubimini/<name>/ft_sensor_readings/imu`** ([`sensor_msgs/Imu`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
* **`/rokubimini/<name>/ft_sensor_readings/wrench`** ([`geometry_msgs/WrenchStamped`](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/WrenchStamped.html))

Values are calibrated if the parameter to use the calibration is enabled. The **`<name>`** is the configuration parameter `name` that is set from the `setup.yaml`.
	