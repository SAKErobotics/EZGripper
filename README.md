# EZGripper

## Install the EZGripper ROS Driver

1) Install dependencies:

	$ sudo apt-get install python-serial ros-indigo-joystick-drivers

2) Download code:

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/SAKErobotics/EZGripper.git
	$ cd ..
	$ catkin_make

3) Setup parameters in joy.launch file
  - ~port - serial device to use
  - ~baud - baud rate
  - servo_ids - the servo(s) id controlled as a group of grippers, for example [9,10,11]


4) Launch the node

	$ roslaunch ezgripper_driver joy.launch
	
