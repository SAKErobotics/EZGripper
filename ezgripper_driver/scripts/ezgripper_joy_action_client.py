#!/usr/bin/python3
"""
EZGripper Joy Action Client Module
"""

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, SAKE Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##

import rospy
from sensor_msgs.msg import Joy
from ezgripper_libs.ezgripper_interface import EZGripper


class EZGripperJoy():
    """
    EZGripper Joy Action Client
    """

    def __init__(self, module_types, gripper_names):
        self.ezgripper_left = EZGripper(module_types[0], gripper_names[0])
        if len(gripper_names) > 1:
            self.ezgripper_right = EZGripper(module_types[1], gripper_names[1])
        else:
            self.ezgripper_right = None
        self.last_command_end_time = rospy.get_rostime()

    def joy_callback(self, joy):
        """
        Joystick Callback
        """

        if not joy.buttons:
            return # Don't break on an empty list

        if joy.buttons[5] == 1 and self.ezgripper_right is not None: # RB
            gripper = self.ezgripper_right
        else:
            gripper = self.ezgripper_left

        if (rospy.get_rostime() - self.last_command_end_time).to_sec() > 0.2:
            # This check should flush all messages accumulated during command execution
            # and avoid executing it again.

            if joy.buttons[0] == 1: # A - hard close
                gripper.hard_close()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[3] == 1: # Y - soft close
                gripper.soft_close()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[1] == 1: # B - open
                gripper.open()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[2] == 1: # X - release
                gripper.release()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[6] == 1: # BACK - Calibrate
                gripper.calibrate()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[13] == 1: # xpad driver mapping
            #if joy.axes[7] == 1.0: # xboxdrv mapping
                gripper.open_step()
                self.last_command_end_time = rospy.get_rostime()

            if joy.buttons[14] == 1:
            #if joy.axes[7] == -1.0:
                gripper.close_step()
                self.last_command_end_time = rospy.get_rostime()

def main():
    """
    Main Function
    """
    rospy.init_node("ezgripper_joy_client")

    no_of_grippers = rospy.get_param("/ezgripper_controller/no_of_grippers")
    gripper_names = []
    module_types = []

    for i in range(1, int(no_of_grippers) + 1):
        action_name = rospy.get_param("/ezgripper_controller/gripper_{}/action_name".format(i))
        module_type = rospy.get_param("/ezgripper_controller/gripper_{}/module_type".format(i))
        robot_ns = rospy.get_param("/ezgripper_controller/gripper_{}/robot_ns".format(i))

        module_types.append(module_type)
        gripper_names.append(robot_ns + '/ezgripper_controller/'+ action_name)


    ezgripper_joy = EZGripperJoy(module_types, gripper_names)

    rospy.Subscriber("/joy", Joy, ezgripper_joy.joy_callback)

    rospy.spin()
    rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
