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

from functools import partial
import rospy
from std_srvs.srv import Empty, EmptyResponse
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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

class GripperAction:
    """
    GripperCommand Action Server to send Feedback to MoveIt! and
        to recieve goals from MoveIt!
    """

    _feedback = GripperCommandFeedback()
    _result = GripperCommandResult()

    def __init__(self, name, gripper):

        self.gripper = gripper

        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer( \
            self._action_name, GripperCommandAction, self.execute_cb, False)
        self._as.start()

    def execute_cb(self, goal):
        """
        Action server callback to execute a goal
        """

        # helper variables
        success = True

        # Debug string
        rospy.loginfo("Execute goal: position=%.1f, max_effort=%.1f"%
                      (goal.command.position, goal.command.max_effort))

        # Actuate Gripper
        if goal.command.max_effort == 0.0:
            rospy.loginfo("Release torque: start")
            self.gripper.release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            self.gripper.goto_position(goal.command.position, \
                goal.command.max_effort, use_percentages = False)
            rospy.loginfo("Go to position: done")

        # Feedback
        self._feedback.position = current_gripper_position
        self._as.publish_feedback(self._feedback)

        # Preemption Check
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        # Publish result
        if success:
            self._result.position = goal.command.position
            self._result.effort = goal.command.max_effort
            self._result.stalled = False
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)

def send_diags():
    """
    Send Diagnostic Data
    """
    # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
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

rospy.init_node('ezgripper_controller')
rospy.loginfo("Started")

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
gripper_params = rospy.get_param('~grippers', {'gripper_cmd': [1]})

diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

connection = create_connection(port_name, baud)

all_servos = []
references = []
grippers = []
gripper = None
current_gripper_position = 0.0

for gripper_name, servo_ids in gripper_params.items():

    gripper = Gripper(connection, gripper_name, servo_ids)
    all_servos += gripper.servos

    gripper.calibrate()
    gripper.open()

    references.append( rospy.Service('~'+gripper_name+'/calibrate', \
        Empty, partial(calibrate_srv, gripper)))
    references.append( GripperAction('~'+gripper_name, gripper) )

    grippers.append(gripper)


# Main Loop

r = rospy.Rate(20) # hz
diags_last_sent = 0

while not rospy.is_shutdown():

    current_gripper_position = gripper.get_position(use_percentages = False)
    print("Feedback = {}".format(current_gripper_position))

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

    r.sleep()

rospy.loginfo("Exiting")
