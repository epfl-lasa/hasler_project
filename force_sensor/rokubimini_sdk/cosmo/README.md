# COSMO - Communication Over Shared Memory On-demand

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/cosmo/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/cosmo/job/master/)


## Overview

The software has been tested under ROS Melodic and Ubuntu 18.04.

The source code is released under a [proprietary license](LICENSE).

**Author(s)**: Philipp Leemann, Christian Gehring


* cosmo: Subscriber and Publisher classes for processes to communicate over shared memory.
* cosmo_example: Example implementation of cosmo.
* cosmo_ros: Extension of cosmo, combining the shared memory communication channel with ROS.
* cosmo_ros_example: Example implementation of cosmo_ros.
* cosmo_node: Interface definition for a generic Node implementation and Wrapper to easily set up ROS and cosmo communication (any_node replacement).
* cosmo_node_example: Example implementation of cosmo_node.

Defining COSMO_DEBUG_LEVEL=1 to cmake (or catkin calls) will print information about memory pools to the console, defining COSMO_DEBUG_LEVEL=2 will also print timing information about sent and received messages.


## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.


## Memory tools

Provided helper tools (can be run with '''rosrun cosmo ...''', add '''--help''' flag to see arguments):

* memory_info: Prints infos about a shared memory pool
* memory_remover: Removes a shared memory pool

## Limitations
Some objects cannot be used in combination with shared memory, because they lead to problems when trying to access them in a process which is not the same as the process which constructed them. This includes the following:

* Any heap memory, so pointers, references and all containers of C++ STL, except std::array, are not allowed. This also includes objects owning one of these types.
* You cannot call virtual functions on objects in shared memory. The pointer to the virtual table is in the address space of the constructing process and cannot be accessed by any other process.
* Be careful with static members, they are allocated in global scope and are not owned by the instance of an object and are thus not copied together with the instance to shared memory.

See http://www.boost.org/doc/libs/1_46_0/doc/html/interprocess/sharedmemorybetweenprocesses.html#interprocess.sharedmemorybetweenprocesses.mapped_region_object_limitations for details

Also, if a process, which has created Subscribers or Publisers, crashes, shared memory is not cleaned appropriately. This means if you start a new process, it may receive to old, invalid messages or, even worse, run into a deadlock if a mutex is still locked by the crashed process. So if one of your proecsses crashs, you should clear the whole shared memory by deleting /dev/shm/COSMO_SHM (or using the memory_remover tool). Note that you should only do this if no other processes using COSMO are still running!

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/cosmo/issues).
