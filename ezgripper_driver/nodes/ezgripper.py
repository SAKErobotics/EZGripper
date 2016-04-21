#!/usr/bin/python

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

import rospy
from std_srvs.srv import Empty, EmptyResponse
from ezgripper_libs.lib_robotis import create_connection, Robotis_Servo
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from math import acos, radians
from functools import partial


def set_torque_mode(servo, val):
    if val:
        print "torque on"
        servo.write_address(70, [1])
    else:
        print "torque off"
        servo.write_address(70, [0])

def wait_for_stop(servo):
    wait_start = rospy.get_rostime()
    last_position = 1000000 # read_encoder() cannot return more than 65536
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        current_position = servo.read_encoder()
        if current_position == last_position:
            break
        last_position = current_position
        
        if (rospy.get_rostime() - wait_start).to_sec() > 5.0:
            break
        
        rospy.sleep(0.05)

def calibrate_srv(gripper, msg):
    rospy.loginfo("Calibrate service: request received")
    gripper.calibrate()
    gripper.open()    
    rospy.loginfo("Calibrate service: request completed")
    return EmptyResponse()
    
def servo_position_from_gap(gap):
    angle = acos((gap - distance_between_fingers)/2.0/finger_link_length)
    angle = grip_angle_max - angle # now it's: 0 - closed, 102 degrees - open
    position = int(angle / grip_angle_max * grip_max)
    if position < 0: position = 0
    if position > grip_max: position = grip_max
    return position
    
class Gripper:
    def __init__(self, name, servo_ids):
        self.name = name
        self.servos = [Robotis_Servo( dyn, servo_id ) for servo_id in servo_ids]
    
    def calibrate(self):
        rospy.loginfo("Calibrating: " + self.name)
        
        for servo in self.servos:
            servo.write_address(6, [255,15,255,15] )   # 1) "Multi-Turn" - ON
            servo.write_word(20, 0)                    # 4) set "Multi turn offset" to 0 -- this command affects torque settings, so it was moved before initial torque is applied. 
            servo.write_word(34, 500)                  # 2) "Torque Limit" to 500 (or so)
            servo.write_address(24, [0])               # 3) "Torque Enable" to OFF
            servo.write_address(70, [1])               # 1) Set "Goal Torque Mode" to ON
            servo.write_word(71, 1024 + 100)           # 2) Set "Goal Torque" Direction to CW and Value 100
        
        rospy.sleep(2.0)                               # give it time to stop
        
        for servo in self.servos:
            position = servo.read_word(36)
            servo.write_address(70, [0])               # Stopping torque here improves makes writing "multi-word offset" consistent
            servo.write_word(20,-position)
            
            # The reported position sometimes is off by a rotation (4096)
            # after the first multiturn offset write. Recalculating the offset
            # once more seems to help.
            position = servo.read_word(36)
            multiturnoffset = servo.read_word(20)
            servo.write_word(20, multiturnoffset - position)
            
        rospy.loginfo("Calibration completed")
    
    def open(self):
        for servo in self.servos:
            set_torque_mode(servo, False)
            servo.write_word(30, grip_max)
        rospy.sleep(1.0)
    
    def close(self, closing_torque):
        for servo in self.servos:
            set_torque_mode(servo, True)
            servo.write_word(71, 1024 + closing_torque)  # Set "Goal Torque" Direction to CW and Value
            
        wait_for_stop(self.servos[0])
        
        for servo in self.servos:
            servo.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
    
    def goto_position(self, position):
        for servo in self.servos:
            set_torque_mode(servo, False)            
        for servo in self.servos:
            servo.write_word(30, position)
        wait_for_stop(self.servos[0])

class GripperActionServer:
    def __init__(self, action_name, gripper):
        self.gripper = gripper
        self.action_server = actionlib.SimpleActionServer(action_name, GripperCommandAction, self.gripper_action_execute, False)
        self.action_server.start()
        
    def gripper_action_execute(self, goal):
        rospy.loginfo("Execute goal: position=%.3f, max_effort=%.3f"%
                      (goal.command.position, goal.command.max_effort))
        
        if goal.command.max_effort == 0.0:
            rospy.loginfo("Release torque: start")
            for servo in self.gripper.servos:
                set_torque_mode(servo, False)
            result = GripperCommandResult()
            result.position = goal.command.position
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = True
            self.action_server.set_succeeded(result)
            rospy.loginfo("Release torque: done")
            return
        
        if goal.command.position > max_gap - 0.005:
            rospy.loginfo("Open: start")
            self.gripper.open()
            result = GripperCommandResult()
            result.position = goal.command.position
            result.effort = 12.0
            result.stalled = False
            result.reached_goal = True
            self.action_server.set_succeeded(result)
            rospy.loginfo("Open: done")
            return
        
        if goal.command.position == 0.0:
            rospy.loginfo("Close: start")
            closing_torque = goal.command.max_effort*16.0 - 96.0 # TODO: need a more accurate calculation
            if closing_torque < torque_hold: closing_torque = torque_hold
            if closing_torque > torque_max: closing_torque = torque_max
            rospy.loginfo("Using closing torque %.2f"%closing_torque)
            self.gripper.close(closing_torque)
            result = GripperCommandResult()
            result.position = goal.command.position
            result.effort = 12.0
            result.stalled = False
            result.reached_goal = True
            self.action_server.set_succeeded(result)
            rospy.loginfo("Close: done")
            return
        
        rospy.loginfo("Go to position: start")
        # TODO: use the effort value as well
        servo_position = servo_position_from_gap(goal.command.position)
        rospy.loginfo("Target position: %.3f (%d)"%(goal.command.position, servo_position))
        self.gripper.goto_position(servo_position)
        result = GripperCommandResult()
        result.position = goal.command.position
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = True
        self.action_server.set_succeeded(result)    
        rospy.loginfo("Go to position: done")
    
def send_diags():
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

rospy.init_node('ezgripper')
rospy.loginfo("Started")

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
gripper_params = rospy.get_param('~grippers')

diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

distance_between_fingers = 0.05 # meters
finger_link_length = 0.065
max_gap = distance_between_fingers + finger_link_length * 2

grip_max = 2800 #maximum open position for grippers
grip_angle_max = radians(102) # 102 degrees, in radians
grip_min = 0

torque_max = 350 # maximum torque - MX-64=500, MX-106=350
torque_hold = 100 # holding torque - MX-64=100, MX-106=80

dyn = create_connection(port_name, baud)

all_servos = []
references = []
grippers = []

for gripper_name, servo_ids in gripper_params.iteritems():
    gripper = Gripper(gripper_name, servo_ids)
    all_servos += gripper.servos
    for servo in gripper.servos:
        servo.ensure_byte_set(22, 1) # Make sure 'Resolution divider' is set to 1

    gripper.calibrate()
    gripper.open()
    
    references.append( rospy.Service('~'+gripper_name+'/calibrate', Empty, partial(calibrate_srv, gripper)) )
    references.append( GripperActionServer('~'+gripper_name, gripper) )
    
    grippers.append(gripper)

# Main Loop

r = rospy.Rate(20) # hz
diags_last_sent = 0
while not rospy.is_shutdown():
    now = rospy.get_time()
    if now - diags_last_sent > 1.0:
        try:
            send_diags()
            diags_last_sent = now
        except Exception, e:
            rospy.logerr("Exception while reading diagnostics: %s"%e)
            
    for servo in all_servos:
        try:
            servo.check_overload_and_recover()
        except Exception, e:
            rospy.logerr("Exception while checking overload: %s"%e)
            servo.flushAll()

    r.sleep()
    
rospy.loginfo("Exiting")
