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

#
#  If you want to modify this program, you can find the "Main program" and
#  "Main loop" by searching for these terms.  They exist near the end of this file.
#

import sys
from functools import partial
from math import fabs
import rospy
from std_srvs.srv import Empty, EmptyResponse
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from libezgripper import create_connection, Gripper


def calibrate_srv(gripper, msg):
    """
    Calibration Service
    """
    rospy.loginfo("Calibrate service: request received")
    gripper.calibrate()
    gripper.open()
    rospy.loginfo("Calibrate service: request completed")
    return EmptyResponse()

def now_from_start(start):
    return rospy.get_time() - start


class GripperAction:
    """
    GripperCommand Action Server to send Feedback to MoveIt! and
        to recieve goals from MoveIt!
    """

    _feedback = GripperCommandFeedback()
    _result = GripperCommandResult()

    def __init__(self, name, gripper, gripper_module='dual_gen1'):

        self.gripper = gripper
        self.gripper_module = gripper_module
        self._timeout = 3.0
        self._positional_buffer = 0.05

        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer( \
            self._action_name, GripperCommandAction, self.execute_cb, False)
        self._as.start()

    def _command_gripper(self, position, effort):
        """
        Actuate gripper to position and effort
        """

        # Debug string
        rospy.loginfo("Execute goal: position=%.1f, max_effort=%.1f"%
                      (position, effort))

        # Actuate Gripper
        if effort == 0.0:
            rospy.loginfo("Release torque: start")
            self.gripper.release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            self.gripper.goto_position(position, effort, \
                use_percentages = False, gripper_module=self.gripper_module)
            rospy.loginfo("Go to position: done")

    def _check_state(self, position):
        """
        Check if gripper has reached desired position
        """
        return fabs(self.gripper.get_position( \
            use_percentages = False, \
                gripper_module=self.gripper_module) - position) < self._positional_buffer

    def _publish_feedback_and_update_result(self, position, effort):
        """
        Publish Gripper Feedback and Update Result
        """
        self._feedback.position = self.gripper.get_position( \
            use_percentages = False, gripper_module=self.gripper_module)
        self._feedback.effort = effort
        self._feedback.reached_goal = self._check_state(position)
        self._result = self._feedback
        self._as.publish_feedback(self._feedback)


    def execute_cb(self, goal):
        """
        Action server callback to execute a goal
        """

        position = goal.command.position
        effort = goal.command.max_effort
        global max_effort
        max_effort = effort
        control_rate = rospy.Rate(20)
        start_time = rospy.get_time()

        # Iterate until goal is reached or timeout
        while not rospy.is_shutdown() \
            and now_from_start(start_time) < self._timeout:

            # Publish Feedback and Update Result
            self._publish_feedback_and_update_result(position, effort)

            # Command gripper
            self._command_gripper(position, effort)

            # Check if goal is reached
            if self._check_state(position):
                self._as.set_succeeded(self._result)
                rospy.loginfo("Gripper has reached desired position")
                return

            # Preemption Check
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._as.set_aborted(self._result)
                return

            control_rate.sleep()

        rospy.loginfo("Gripper has grasped an object")

def send_diags():
    """
    Send Diagnostic Data
    """

    msg = DiagnosticArray()
    msg.status = []
    msg.header.stamp = rospy.Time.now()

    for gripper in grippers:
        for servo in gripper.servos:
            status = DiagnosticStatus()
            status.name = "Gripper '%s' servo %d"%(gripper.name, servo.servo_id)
            status.hardware_id = '%s'%servo.servo_id
            temperature = servo.read_temperature()
            status.values.append(KeyValue('Temperature', str(temperature)))
            status.values.append(KeyValue('Voltage', str(servo.read_voltage())))

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

    diagnostics_pub.publish(msg)


# Main Program

all_servos = []
references = []
grippers = []
gripper = None
current_gripper_position = 0.0
finger_joint = ''
gripper_module = sys.argv[1]

if gripper_module == 'dual_gen1' or gripper_module == 'quad':
    finger_joint = 'left_ezgripper_knuckle_1'

elif gripper_module == 'dual_gen2':
    finger_joint = 'left_ezgripper_knuckle_palm_L1_1'

diags_last_sent = 0
MAX_VELOCITY = 3.67
max_effort = 0.0

rospy.init_node('ezgripper_controller')
rospy.loginfo("Started")
rate = rospy.Rate(20) # hz

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
gripper_params = rospy.get_param('~grippers', {'gripper_cmd': [1]})

diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

connection = create_connection(port_name, baud)


for gripper_name, servo_ids in gripper_params.items():

    gripper = Gripper(connection, gripper_name, servo_ids)
    all_servos += gripper.servos

    gripper.calibrate()
    gripper.open()

    references.append( rospy.Service('~'+gripper_name+'/calibrate', \
        Empty, partial(calibrate_srv, gripper)))
    references.append( GripperAction('~'+gripper_name, gripper, gripper_module) )

    grippers.append(gripper)


# Main Loop

while not rospy.is_shutdown():

    current_gripper_position = gripper.get_position( \
        use_percentages = False, gripper_module=gripper_module)

    # Publish Joint States
    jointState = JointState()
    jointState.header = Header()
    jointState.header.stamp = rospy.Time.now()
    jointState.name = [finger_joint]
    jointState.position = [current_gripper_position]
    jointState.velocity = [MAX_VELOCITY]
    jointState.effort = [max_effort]
    joint_pub.publish(jointState)

    now = rospy.get_time()
    if now - diags_last_sent > 1.0:
        try:
            send_diags()
            diags_last_sent = now
        except Exception as error:
            rospy.logerr("Exception while reading diagnostics: %s"%error)

    for servo in all_servos:
        try:
            servo.check_overload_and_recover()
        except Exception as error:
            rospy.logerr("Exception while checking overload: %s"%error)
            servo.flushAll()

    rate.sleep()

rospy.loginfo("Exiting")
