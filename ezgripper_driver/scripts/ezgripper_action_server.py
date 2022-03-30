#!/usr/bin/python3
"""
EZGripper Action Server Module
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

import time
from functools import partial
from math import fabs
import rospy
from std_srvs.srv import Empty, EmptyResponse
from actionlib import SimpleActionServer
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from libezgripper import create_connection, Gripper



class GripperAction:
    """
    GripperCommand Action Server
    """

    _feedback = GripperCommandFeedback()
    _result = GripperCommandResult()

    def __init__(self):

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        self._timeout = 3.0
        self._positional_buffer = 0.05
        self.all_servos = []

        self.port = rospy.get_param("/ezgripper_controller/port")
        self.baudrate = rospy.get_param("/ezgripper_controller/baudrate")
        self.no_of_grippers = rospy.get_param("/ezgripper_controller/no_of_grippers")

        self.grippers = {}
        connection = create_connection(dev_name=self.port, baudrate=self.baudrate)

        for i in range(1, int(self.no_of_grippers) + 1):

            action_name = rospy.get_param("/ezgripper_controller/gripper_{}/action_name".format(i))
            servo_ids = rospy.get_param("/ezgripper_controller/gripper_{}/servo_ids".format(i))
            module_type = rospy.get_param("/ezgripper_controller/gripper_{}/module_type".format(i))

            self.grippers[action_name] = Gripper(connection, action_name, servo_ids)
            self.all_servos += self.grippers[action_name].servos

            self.grippers[action_name].calibrate()
            self.grippers[action_name].open()

            rospy.Service('~/'+ action_name + '/calibrate', \
                Empty, partial(self.calibrate_srv, action_name))

            self._as = \
                SimpleActionServer( \
                    action_name, \
                    GripperCommandAction, \
                    partial(self._execute_callback, action_name, module_type), \
                    False)

            self._as.start()

        # Publishers
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

        # Timers
        rospy.Timer(rospy.Duration(self.time_period), self.joint_state_update)
        rospy.Timer(1.0, self.diagnostics_and_servo_update)

        rospy.loginfo("Gripper server ready")


    def joint_state_update(self, event):
        """
        Publish joint state
        """

        for i in range(1, int(self.no_of_grippers) + 1):

            action_name = rospy.get_param("/ezgripper_controller/gripper_{}/action_name".format(i))
            module_type = rospy.get_param("/ezgripper_controller/gripper_{}/module_type".format(i))

            current_gripper_position = self.grippers[action_name].get_position( \
                use_percentages = False, gripper_module=module_type)

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.position = [current_gripper_position]

            if module_type == 'dual_gen1' or module_type == 'quad':
                finger_joint = 'left_ezgripper_knuckle_1'

            elif module_type == 'dual_gen2' or module_type == 'dual_gen2_single_mount':
                finger_joint = 'left_ezgripper_knuckle_palm_L1_1'

            elif module_type == 'dual_gen2_triple_mount':
                finger_joint = 'left1_ezgripper_knuckle_palm_L1_1'

            msg.name = [finger_joint]

            self.joint_state_pub.publish(msg)


    def diagnostics_and_servo_update(self, event):
        """
        Send Diagnostic Data and monitor servos
        """

        msg = DiagnosticArray()
        msg.status = []
        msg.header.stamp = rospy.Time.now()

        for i in range(1, int(self.no_of_grippers) + 1):
            action_name = rospy.get_param("/ezgripper_controller/gripper_{}/action_name".format(i))

            gripper = self.grippers[action_name]

            temp_msg = KeyValue()
            temp_msg.key = 'Temperature'

            volt_msg = KeyValue()
            volt_msg.key = 'Voltage'

            for servo in gripper.servos:
                status = DiagnosticStatus()
                status.name = "Gripper '%s' servo %d"%(gripper.name, servo.servo_id)
                status.hardware_id = '%s'%servo.servo_id
                temperature = servo.read_temperature()

                temp_msg.value = str(temperature)
                volt_msg.value = str(servo.read_voltage())

                status.values.append(temp_msg)
                status.values.append(volt_msg)

                if temperature >= 70:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'OVERHEATING'
                elif temperature >= 65:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'HOT'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'

                msg.status.append(status)

        self.diagnostics_pub.publish(msg)

        for servo in self.all_servos:
            try:
                servo.check_overload_and_recover()
            except Exception as error:
                rospy.loginfo('Exception while checking overload')
                servo.flushAll()

    def calibrate_srv(self, action_name, request):
        """
        Calibration Service
        """
        rospy.loginfo("Calibrate service: request received")
        self.grippers[action_name].calibrate()
        self.grippers[action_name].open()
        rospy.loginfo("Calibrate service: request completed")
        return EmptyResponse()

    def now_from_start(self, start_time):
        """
        Get time difference from start time
        """
        return rospy.get_time() - start_time

    def _command_gripper(self, action_name, module_type, position, effort):
        """
        Actuate gripper to position and effort
        """

        # Debug string
        rospy.loginfo("Execute goal: position=%.1f, max_effort=%.1f"%
                      (position, effort))

        # Actuate Gripper
        if effort == 0.0:
            rospy.loginfo("Release torque: start")
            self.grippers[action_name].release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            self.grippers[action_name].goto_position(position, effort, \
                use_percentages = False, gripper_module = module_type)
            rospy.loginfo("Go to position: done")

    def _check_state(self, action_name, module_type, position):
        """
        Check if gripper has reached desired position
        """
        return fabs(self.grippers[action_name].get_position( \
            use_percentages = False, \
                gripper_module = module_type) - position) < self._positional_buffer

    def _publish_feedback_and_update_result(self, action_name, module_type, position, effort):
        """
        Publish Gripper Feedback and Update Result
        """
        self._feedback.position = self.grippers[action_name].get_position( \
            use_percentages = False, gripper_module = module_type)
        self._feedback.effort = effort
        self._feedback.reached_goal = self._check_state(action_name, module_type, position)
        self._result = self._feedback
        self._as.publish_feedback(self._feedback)

    def _execute_callback(self, action_name, module_type, goal_handle):
        """
        Execute callback for action server
        """

        position = goal_handle.command.position
        effort = goal_handle.command.max_effort

        start_time = rospy.get_time()

        # Iterate until goal is reached or timeout
        while self.now_from_start(start_time) < self._timeout:

            # Publish Feedback and Update Result
            self._publish_feedback_and_update_result( \
                action_name, module_type, position, effort)

            # Command gripper
            self._command_gripper(action_name, module_type, position, effort)

            # Check if goal is reached
            if self._check_state(action_name, module_type, position):
                self._as.set_succeeded(self._result)
                rospy.loginfo("Gripper has reached desired position")
                return

            # Preemption Check
            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted')
                self._as.set_preempted()
                self._as.set_aborted(self._result)
                return

            time.sleep(0.01)

        rospy.loginfo("Gripper has grasped an object")

def main():
    """
    Main Function
    """
    rospy.init_node('ezgripper_controller')
    GripperAction()
    rospy.spin()
    rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
