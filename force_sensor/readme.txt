Installation Instructions:
- Unzip the content of this file inside the src/ folder of your of the active catkin workspace
- make sure you have the package "catkin tools" installed otherwise you can install it with the following comand 
	$ sudo apt-get install python-catkin-tools
- navigate inside the catkin workspace and build the software with the following command 
	$ catkin build rokubimini_ros

Operation Instructions:
- Connect the cable of the sensor to your computer and make sure that is appering as serial port under the name 
	"/dev/ttyUSB*" (where * any number)
- Open the file following file and make sure that the serial port is set accordingly
	"/rokubimini_sdk/rokubimini_cosmo/config/setup.yaml"
- To run the driver run the following command 
	$ roslaunch rokubimini_cosmo rokubimini.launch




