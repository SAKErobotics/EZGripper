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
from ezgripper_libs.lib_robotis import USB2Dynamixel_Device, Robotis_Servo, CommunicationError
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from math import acos, radians


def calibrate(servos):
    rospy.loginfo("Calibrating")
    
    for servo in servos:
        servo.write_address(6, [255,15,255,15] )   # 1) "Multi-Turn" - ON
        servo.write_word(34, 500)                  # 2) "Torque Limit" to 500 (or so)
        servo.write_address(24, [0])               # 3) "Torque Enable" to OFF
        servo.write_address(70, [1])               # 1) Set "Goal Torque Mode" to ON
        servo.write_word(71, 1024 + 100)           # 2) Set "Goal Torque" Direction to CW and Value 100
    
    rospy.sleep(2.0)                               # give it time to stop
    
    for servo in servos:
        position = servo.read_word(36)
        multiturnoffset = servo.read_word(20)      # 4) Set "Multi turn offset" to 0
        servo.write_address(70, [0])
        servo.write_word(20, multiturnoffset - position)
        servo.write_word(71, 1024 + 0)             # 3) Quickly turn "Goal Torque Mode" to OFF (to reduce load on motor)
        
    rospy.loginfo("Calibration completed")
    
def set_torque_mode(servo, val):
    if val:
        print "torque on"
        servo.write_address(70, [1])
    else:
        print "torque off"
        servo.write_address(70, [0])

def gripper_open():
    for servo in servos:
        set_torque_mode(servo, False)
        servo.write_word(30, grip_max)
    rospy.sleep(1.0)

def wait_for_stop():
    wait_start = rospy.get_rostime()
    last_position = 1000000 # read_encoder() cannot return more than 65536
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            current_position = servos[0].read_encoder()
            if current_position == last_position:
                break
            last_position = current_position
            
            if (rospy.get_rostime() - wait_start).to_sec() > 5.0:
                break
        except CommunicationError as e:
            rospy.logwarn("wait_for_stop loop CommunicationError: %s"%e)
            servo.flushAll()
        
        rospy.sleep(0.05)

def gripper_close(closing_torque):
    for servo in servos:
        set_torque_mode(servo, True)
        servo.write_word(71, 1024 + closing_torque)  # Set "Goal Torque" Direction to CW and Value
        
    wait_for_stop()
    
    for servo in servos:
        servo.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value

def gripper_goto_position(position):
    for servo in servos:
        set_torque_mode(servo, False)            
    for servo in servos:
        servo.write_word(30, position)
    wait_for_stop()
    

def calibrate_srv(msg):
    rospy.loginfo("Calibrate service: request received")
    calibrate(servos)
    gripper_open()    
    rospy.loginfo("Calibrate service: request completed")
    return EmptyResponse()
    
def gripper_action_execute(goal):
    rospy.loginfo("Execute goal: position=%.3f, max_effort=%.3f"%
                  (goal.command.position, goal.command.max_effort))
    
    if goal.command.max_effort == 0.0:
        rospy.loginfo("Release torque: start")
        for servo in servos:
            set_torque_mode(servo, False)
        result = GripperCommandResult()
        result.position = goal.command.position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        action_server.set_succeeded(result)
        rospy.loginfo("Release torque: done")
        return
    
    if goal.command.position > max_gap - 0.005:
        rospy.loginfo("Open: start")
        gripper_open()
        result = GripperCommandResult()
        result.position = goal.command.position
        result.effort = 12.0
        result.stalled = False
        result.reached_goal = True
        action_server.set_succeeded(result)
        rospy.loginfo("Open: done")
        return
    
    if goal.command.position == 0.0:
        rospy.loginfo("Close: start")
        closing_torque = goal.command.max_effort*16.0 - 96.0 # TODO: need a more accurate calculation
        if closing_torque < torque_hold: closing_torque = torque_hold
        if closing_torque > torque_max: closing_torque = torque_max
        rospy.loginfo("Using closing torque %.2f"%closing_torque)
        gripper_close(closing_torque)
        result = GripperCommandResult()
        result.position = goal.command.position
        result.effort = 12.0
        result.stalled = False
        result.reached_goal = True
        action_server.set_succeeded(result)
        rospy.loginfo("Close: done")
        return
    
    rospy.loginfo("Go to position: start")
    # TODO: use the effort value as well
    servo_position = servo_position_from_gap(goal.command.position)
    rospy.loginfo("Target position: %.3f (%d)"%(goal.command.position, servo_position))
    gripper_goto_position(servo_position)
    result = GripperCommandResult()
    result.position = goal.command.position
    result.effort = goal.command.max_effort
    result.stalled = False
    result.reached_goal = True
    action_server.set_succeeded(result)    
    rospy.loginfo("Go to position: done")
        
    
def servo_position_from_gap(gap):
    angle = acos((gap - distance_between_fingers)/2.0/finger_link_length)
    angle = grip_angle_max - angle # now it's: 0 - closed, 102 degrees - open
    position = int(angle / grip_angle_max * grip_max)
    if position < 0: position = 0
    if position > grip_max: position = grip_max
    return position
    

# Main Program

rospy.init_node('ezgripper')
rospy.loginfo("Started")

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
servo_ids = rospy.get_param('~servo_ids')

distance_between_fingers = 0.05 # meters
finger_link_length = 0.065
max_gap = distance_between_fingers + finger_link_length * 2

grip_max = 2800 #maximum open position for grippers
grip_angle_max = radians(102) # 102 degrees, in radians
grip_min = 0

torque_max = 350 # maximum torque - MX-64=500, MX-106=350
torque_hold = 100 # holding torque - MX-64=100, MX-106=80

dyn = USB2Dynamixel_Device(port_name, baud)
servos = [Robotis_Servo( dyn, servo_id ) for servo_id in servo_ids]

calibrate(servos)
gripper_open()

last_command_end_time = rospy.get_rostime()
rospy.Service('calibrate', Empty, calibrate_srv)
action_server = actionlib.SimpleActionServer('gripper', GripperCommandAction, gripper_action_execute, False)
action_server.start()

# Main Loop

r = rospy.Rate(20) # hz
while not rospy.is_shutdown():
    for servo in servos:
        try:
            servo.check_overload_and_recover()
        except CommunicationError, e:
            rospy.logwarn("loop CommunicationError: %s"%e)
            servo.flushAll()
        except Exception, e:
            rospy.logerr("Exception: %s"%e)
            servo.flushAll()

    r.sleep()
    
rospy.loginfo("Exiting")
