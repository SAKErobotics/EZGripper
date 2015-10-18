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
  - grippers - definition of grippers on this serial bus: the gripper name to use for the action interface and the servo id of the gripper (several ids if several grippers are to be used as one group), for example {left:[9], right:[10,11]}

4) Launch the node

	$ roslaunch ezgripper_driver joy.launch
	
## Action API

The driver provides an implementation of the SimpleActionServer, that takes in [control_msgs/GripperCommand](http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html) actions.
A sample client ([nodes/client.py](ezgripper_driver/nodes/client.py)) is included that provides joystick control using the action API.
