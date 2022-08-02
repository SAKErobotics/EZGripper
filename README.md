# EZGripper

A ROS package that serves as a driver to the [EZGripper module](https://sakerobotics.com/) designed by SAKE Robotics. If you are not using ROS, use https://github.com/SAKErobotics/SAKErobotics

## Tutorial

### Installation
---

* Install the python EZGripper library. For kinetic and melodic use [this link](https://github.com/SAKErobotics/libezgripper/tree/master) and for noetic use [the ubuntu 20.04 branch](https://github.com/SAKErobotics/libezgripper/tree/ubuntu-20.04).

* Install the `upatras_gazebo_plugins` by cloning the packge to your catkin workspace in order to enable Gazebo to mimic the EZGripper joints:

	  git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git

* Install all the remaining dependencies using `rosdep`, at the root of your ROS workspace:

	  rosdep install --from-paths src --ignore-src -r -y

* Clone the ROS Driver at you `src` folder:

	For ROS kinetic

   	  git clone --branch=kinetic-devel https://github.com/SAKErobotics/EZGripper.git

	For ROS Melodic

   	  git clone --branch=melodic-devel https://github.com/SAKErobotics/EZGripper.git


	For ROS noetic

   	  git clone --branch=noetic-devel https://github.com/SAKErobotics/EZGripper.git

* Build your workspace and source it:

	  catkin_make && source devel/setup.bash

* For quickly testing hardware connect your USB joystick to the system, and execute:

      roslaunch ezgripper_driver joy.launch

### Simulation testing
---


* Launch the gripper module in RViz :

	  roslaunch ezgripper_description display_single_mount.launch

* Similarly to launch in Gazebo:

	  roslaunch ezgripper_gazebo gazebo_single.launch

* To actuate the gripper into its respective open/close configurations in Gazebo, in a new terminal execute these commands:

	  # Open Gripper
	  rosrun ezgripper_control open_gripper

	  # Close Gripper
	  rosrun ezgripper_control close_gripper

* Result of actuation:

	![ezgripper_gif](https://user-images.githubusercontent.com/45683974/160160044-1a240688-a3f1-4308-a370-0df4f2a84611.gif)

### MoveIt!
---

* Need to install MoveIt!:

          sudo apt install ros-noetic-moveit

* To launch the ezgripper in RViz only:

	  roslaunch ezgripper_single_mount_moveit_config demo.launch

* To launch the ezgripper in Gazebo and RViz for control:

	  roslaunch ezgripper_single_mount_moveit_config demo_gazebo.launch

* To control the ezgripper hardware through MoveIt!:

	  roslaunch ezgripper_single_mount_moveit_config ezgripper_single_mount_moveit_planning_execution.launch

## Additional Configurations

* Setup parameters in [joy.yaml](ezgripper_control/config/joy.yaml) file
  - **`port`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;serial device (like `/dev/ttyUSB0`) or tcp endpoint (like `192.168.0.200:5000`) to use.
  - **`baudrate`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;baud rate of the serial device, not used for tcp.
  - **`no_of_grippers`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;number of grippers to control.
  - Depending upon the number of grippers, gripper profiles can be created as shown:

		gripper_1:
			action_name: gripper_cmd
			servo_ids: [1]
			robot_ns: main

	**`action_name`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;name of the action to be used.<br/>
  **`servo_ids`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;list of servo ids to control. (several ids if several grippers are to be used as one group). For example `[9]` and `[10,11]` for two grippers.<br/>
	By default, SAKE Robotics delivers its grippers with address 1 for Duals and 1 and 2 for Quads and 57kbps.<br/>
 
	**`robot_ns`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;namespace of the robot.

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
---

* The driver provides an implementation of the SimpleActionServer, that takes in [control_msgs/GripperCommand](http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html) actions.<br/>
* A sample client ([scripts/client.py](ezgripper_driver/scripts/client.py)) is included that provides joystick control using the action API.

## URDF Models
---

Access the URDF [models](https://github.com/SAKErobotics/EZGripper/tree/master/ezgripper_driver/urdf) for additional information.


## TroubleShooting
---

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

### ROS Controller Issues:

* Check whether the `joint_state_controller` and the `ezgripper_controller` modules are loaded:

	  rosservice call /ezgripper_single_mount/controller_manager/list_controllers

### ROS Diagnostics

* View the diagnostics of the ezgripper_module:

	  rosrun rqt_runtime_monitor rqt_runtime_monitor
