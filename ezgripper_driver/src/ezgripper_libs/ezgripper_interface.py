#!/usr/bin/python3
"""
EZGripper Interface Module
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
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_srvs.srv import Empty

# http://docs.ros.org/indigo/api/control_msgs/html/msg/GripperCommand.html
# float64 position  # if 0, torque mode, if >0 to 100 correlates to 0-100% rotation range
# float64 max_effort  # if 0, torque released,  if >0 to 100 increasing torque
#
# http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html
# GripperCommand command
# ---
# float64 position  # The current gripper gap size (% rotation of EZGripper fingers) (NOT in meters)
# float64 effort    # The current effort exerted (% available, NOT in Newtons)
# bool stalled      # True iff the gripper is exerting max effort and not moving
# bool reached_goal # True iff the gripper position has reached the commanded setpoint
#


def remap(input_val, in_min, in_max, out_min, out_max):
    """
    Remap Function
    """
    return (input_val - in_min) * (out_max - out_min) / \
            (in_max - in_min) + out_min


class EZGripper():
    """
    EZGripper Class
    """

    MIN_SIMULATED_EFFORT = 0.0
    MAX_SIMULATED_EFFORT = 1.0


    def __init__(self, name):
        self.name = name

        # Open positions
        self._open_position = -1.5707
        self._close_position = 1.94

        self.step_open_pos = self._close_position
        self.step_close_pos = self._open_position

        self._grip_value = self._open_position
        self._connect_to_gripper_action()
        self._connect_to_calibrate_srv()

    def _connect_to_gripper_action(self):
        rospy.loginfo("Waiting for action server %s..."%self.name)
        self._client = actionlib.SimpleActionClient(self.name, GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to action server")

    def _connect_to_calibrate_srv(self):
        service_name = self.name + '/calibrate'
        rospy.loginfo("Waiting for service %s..."%service_name)
        rospy.wait_for_service(service_name)
        self._calibrate_srv = rospy.ServiceProxy(service_name, Empty)
        rospy.loginfo("Connected to service " + service_name)

    def calibrate(self):
        """
        Calibration Function
        """
        rospy.loginfo("ezgripper_interface: calibrate")
        try:
            self._calibrate_srv()
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))
        else:
            self._grip_value = self._open_position
        rospy.loginfo("ezgripper_interface: calibrate done")

    def open_step(self):
        """
        Step Opening the gripper
        """

        # Going from close to open

        step = abs(self._close_position - self._open_position) / 10.0

        if self._close_position < self._open_position:
            self.step_open_pos += step

            if self.step_open_pos > self._open_position:
                self.step_open_pos = self._close_position

        else:
            self.step_open_pos -= step

            if self.step_open_pos < self._open_position:
                self.step_open_pos = self._close_position


        rospy.loginfo("ezgripper_interface: goto position %.3f" % self.step_open_pos)
        goal = GripperCommandGoal()
        goal.command.position = self.step_open_pos
        goal.command.max_effort = 1.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: goto position done")


    def close_step(self):
        """
        Step Closing the Gripper
        """

        # Going from open to close

        step = abs(self._close_position - self._open_position) / 10.0

        if self._open_position < self._close_position:
            self.step_close_pos += step

            if self.step_close_pos > self._close_position:
                self.step_close_pos = self._open_position

        else:
            self.step_close_pos -= step

            if self.step_close_pos < self._close_position:
                self.step_close_pos = self._open_position

        rospy.loginfo("ezgripper_interface: goto position %.3f" % self.step_close_pos)
        goal = GripperCommandGoal()
        goal.command.position = self.step_close_pos
        goal.command.max_effort = 1.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: goto position done")

    def close(self, max_effort):
        """
        Closing the Gripper
        """
        rospy.loginfo("ezgripper_interface: close, effort %.1f" % max_effort)
        goal = GripperCommandGoal()
        goal.command.position = self._close_position
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: close done")
        self._grip_value = self._close_position

    def hard_close(self):
        """
        Hard Closing the Gripper
        """
        rospy.loginfo("ezgripper_interface: hard close")
        goal = GripperCommandGoal()
        goal.command.position = self._close_position
        goal.command.max_effort = 1.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: hard close done")
        self._grip_value = self._close_position

    def soft_close(self):
        """
        Soft Closing the Gripper
        """
        rospy.loginfo("ezgripper_interface: soft close")
        goal = GripperCommandGoal()
        goal.command.position = self._close_position
        goal.command.max_effort = 0.2
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: soft close done")
        self._grip_value = self._close_position

    def open(self):
        """
        Opening the Gripper
        """
        rospy.loginfo("ezgripper_interface: open")
        goal = GripperCommandGoal()
        goal.command.position = self._open_position
        goal.command.max_effort = 1.0
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: open done")
        self._grip_value = self._open_position

    def goto_position(self, current_position, current_effort):
        """
        Go to desired position
        """


        current_position = remap(current_position, \
            100.0, 0.0, self._open_position, self._close_position)


        current_effort = remap(current_effort, \
            0.0, 100.0, self.MIN_SIMULATED_EFFORT, \
                self.MAX_SIMULATED_EFFORT)


        # position in % 0 to 100 (0 is closed), effort in % 0 to 100
        rospy.loginfo("ezgripper_interface: goto position %.3f" % current_position)
        goal = GripperCommandGoal()
        goal.command.position = current_position
        goal.command.max_effort = current_effort
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: goto position done")
        self._grip_value = current_position

    def release(self):
        """
        Release the gripper
        """
        rospy.loginfo("ezgripper_interface: release")
        goal = GripperCommandGoal()
        goal.command.position = 0.0 # not dependent on position
        goal.command.max_effort = 0.0 # max_effort = 0.0 releases all torque on motor
        self._client.send_goal_and_wait(goal)
        rospy.loginfo("ezgripper_interface: release done")
        self._grip_value = self._close_position

if __name__ == "__main__":
    rospy.init_node("ezgripper_interface_node")
    ez = EZGripper('/ezgripper_single_mount/ezgripper_controller/gripper_cmd')
    ez.open()
    ez.calibrate()
    ez.open()
    ez.hard_close()
    ez.open()
    ez.soft_close()
    ez.open()
    rospy.loginfo("Exiting")
