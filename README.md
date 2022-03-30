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

* Set the following bash variable according to your gripper module version. Execute this command in your terminal  or you can add this line to your ~/.bashrc file

	  export ezgripper_module=<your_gripper_module>

	E.g. for `dual_gen2_single_mount` the command would be 

	  export ezgripper_module=dual_gen2_single_mount

	or for `dual_gen2_triple_mount` the command would be

	  export ezgripper_module=dual_gen2_triple_mount

	Older legacy versions are also maintained and can be setting this bash variable appropriately to one of these: `dual_gen1`, `dual_gen2` or `quad`.


* Launch the gripper module in RViz :

	  roslaunch ezgripper_description display.launch ezgripper_module:=${ezgripper_module}

* Similarly to launch in Gazebo:

	  roslaunch ezgripper_gazebo gazebo.launch ezgripper_module:=${ezgripper_module}

* To actuate the gripper into its respective open/close configurations in Gazebo, in a new terminal execute these commands:

	  # Open Gripper
	  rosrun ezgripper_control open_gripper

	  # Close Gripper
	  rosrun ezgripper_control close_gripper

* Result of actuation:

	![ezgripper_gif](https://user-images.githubusercontent.com/45683974/160160044-1a240688-a3f1-4308-a370-0df4f2a84611.gif)

### MoveIt!
---

* To launch the ezgripper in RViz only:

	  roslaunch ezgripper_${ezgripper_module}_moveit_config demo.launch

* To launch the ezgripper in Gazebo and RViz for control:

	  roslaunch ezgripper_${ezgripper_module}_moveit_config demo_gazebo.launch

* To control the ezgripper hardware through MoveIt!:

	  roslaunch ezgripper_${ezgripper_module}_moveit_config ezgripper_${ezgripper_module}_moveit_planning_execution.launch

## Additional Configurations

* Setup parameters in [joy.yaml](ezgripper_control/config/joy.yaml) file
  - **`port`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;serial device (like `/dev/ttyUSB0`) or tcp endpoint (like `192.168.0.200:5000`) to use.
  - **`baudrate`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;baud rate of the serial device, not used for tcp.
  - **`no_of_grippers`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;number of grippers to control.
  - Depending upon the number of grippers, gripper profiles can be created as shown:

		gripper_1:
			action_name: gripper_cmd
			servo_ids: [1]
			module_type: dual_gen2_single_mount
			robot_ns: main

	**`action_name`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;name of the action to be used.<br/>
  **`servo_ids`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;list of servo ids to control. (several ids if several grippers are to be used as one group). For example `[9]` and `[10,11]` for two grippers.<br/>
	By default, SAKE Robotics delivers its grippers with address 1 for Duals and 1 and 2 for Quads and 57kbps.<br/>
  **`module_type`** - <br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;type of the gripper module. (`dual_gen1`, `dual_gen2`, `dual_gen2_single_mount`, `dual_gen2_triple_mount`, `quad`).<br/>
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

	  rosservice call /ezgripper_${ezgripper_module}/controller_manager/list_controllers

### ROS Diagnostics

* View the diagnostics of the ezgripper_module:

	  rosrun rqt_runtime_monitor rqt_runtime_monitor
