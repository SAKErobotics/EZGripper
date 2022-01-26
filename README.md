# EZGripper

A ROS package that serves as a driver to the [EZGripper module](https://sakerobotics.com/) designed by SAKE Robotics. If you are not using ROS, use https://github.com/SAKErobotics/SAKErobotics
## Tutorial 

* Install the python EZGripper library. For kinetic and melodic use [this link](https://github.com/SAKErobotics/libezgripper/tree/master) and for noetic use [the ubuntu 20.04 branch](https://github.com/SAKErobotics/libezgripper/tree/ubuntu-20.04) .

* Install all the remaining dependencies using `rosdep`, at the root of your ROS workspace:

	  rosdep install --from-paths src --ignore-src -r -y

* Clone the ROS Driver at you `src` folder:

	For ROS kinetic and melodic 

   	  git clone --branch=master https://github.com/SAKErobotics/EZGripper.git
	
	For ROS noetic

   	  git clone --branch=noetic-devel https://github.com/SAKErobotics/EZGripper.git
	  

* Build your workspace and source it:

	  catkin_make && source devel/setup.bash

* Connect your USB joystick to the system, and execute:

      roslaunch ezgripper_driver joy.launch


## Additional Configurations

* Setup parameters in joy.launch file
  - **`~port`** - serial device (like `/dev/ttyUSB0`) or tcp endpoint (like `192.168.0.200:5000`) to use
  - **`~baud`** - baud rate of the serial device, not used for tcp
  - **`grippers`** - definition of grippers on this serial bus
  <br/>The gripper name to use for the action interface and the servo id of the gripper (several ids if several grippers are to be used as one group). For example `{left:[9], right:[10,11]}`.
  <br/>By default, SAKE Robotics delivers its grippers with address 1 for Duals and 1 and 2 for Quads and 57kbps.

* Example launch files to support various EZGripper configurations.

	  roslaunch ezgripper_driver joy.launch
	  # joy.launch is configured for a single servo gripper (dual) and the USB interface

	  roslaunch ezgripper_driver joy2.launch
	  # joy2.launch is configured for two independent servos (quad independent) and the USB interface

	  roslaunch ezgripper_driver joy2sync.launch
	  # joy2sync.launch controls two servos as if it were a single servo (quad dependent) and the USB interface

	  roslaunch ezgripper_driver joy_tcp.launch
	  # joy_tcp.launch controls a single servo via TCP instead of USB

## Action API

The driver provides an implementation of the SimpleActionServer, that takes in [control_msgs/GripperCommand](http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html) actions.<br/>
A sample client ([nodes/client.py](ezgripper_driver/nodes/client.py)) is included that provides joystick control using the action API.

## URDF Models

Access the URDF [README](https://github.com/SAKErobotics/EZGripper/tree/master/ezgripper_driver/urdf) for additional information.


## TroubleShooting

### Serial connection issues:

* The following message indicates you have a new version of serial library that causes issues.

	  Error message: 'Serial' object has no attribute 'setParity'  ---

  Do the following command to load an older serial library.

	  sudo apt-get install python-serial==2.0.0

* This indicates the user does not have privileges to use the `/dev/ttyUSBx`:

	  Error message: permission denied (get accurate error message).

	The solution is to add the `<user>` to the `dialout` group.  After executing the following command, reboot.

	  sudo adduser <user> dialout
	  reboot
