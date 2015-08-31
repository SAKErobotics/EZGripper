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
#  If you want to modify the actions of the Joystick, search for "joy_callback"
#
#

import rospy
from sensor_msgs.msg import Joy
from ezgripper_libs.lib_robotis import USB2Dynamixel_Device, Robotis_Servo, CommunicationError


def calibrate(servos):
    rospy.loginfo("Calibrating")
    
    for servo in servos:
        servo.write_address(6, [255,15,255,15] )   # 1) "Multi-Turn" - ON
        servo.write_word(34, 500)                  # 2) "Torque Limit" to 500 (or so)
        servo.write_address(24, [0])               # 3) "Torque Enable" to OFF
        servo.write_address(70, [1])               # 1) Set "Goal Torque Mode" to ON
        servo.write_word(71, 1024 + torque_hold)   # 2) Set "Goal Torque" Direction to CW and Value 50
    
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

def gripper_close():
    for servo in servos:
        set_torque_mode(servo, True)
        servo.write_word(71, 1024 + torque_max)  # Set "Goal Torque" Direction to CW and Value
        
    rospy.sleep(0.5)
    
    for servo in servos:
        servo.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value

def gripper_open_step():
    global grip_value
    
    for servo in servos:
        set_torque_mode(servo, False)
    grip_value = grip_value + grip_step
    if grip_value > grip_max:
        grip_value = grip_max
    print grip_value
    for servo in servos:
        servo.write_word(30, grip_value)

def grip_close_step():
    global grip_value
    
    for servo in servos:
        set_torque_mode(servo, False)            
    grip_value = grip_value - grip_step
    if grip_value < grip_min:
        grip_value = grip_min
    print grip_value
    for servo in servos:
        servo.write_word(30, grip_value)

def joy_callback(joy):
    global grip_value, last_command_end_time
    
    # gripper position
    if joy.buttons[13] == 1: # xpad driver mapping
    #if joy.axes[7] == 1.0: # xboxdrv mapping
        gripper_open_step()
            
    if joy.buttons[14] == 1:
    #if joy.axes[7] == -1.0:
        grip_close_step()
        
    if (rospy.get_rostime() - last_command_end_time).to_sec() > 0.2:
        # This check should flush all messages accumulated during command execution
        # and avoid executing it again.
        
        if joy.buttons[3] == 1: # Y
            for servo in servos:
                set_torque_mode(servo, True)
                servo.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
            rospy.sleep(1.0)
            last_command_end_time = rospy.get_rostime()
                
        if joy.buttons[2] == 1: # X
            for servo in servos:
                set_torque_mode(servo, False)
            last_command_end_time = rospy.get_rostime()
    
        if joy.buttons[1] == 1: # B
            gripper_open()
            last_command_end_time = rospy.get_rostime()
    
        if joy.buttons[0] == 1: # A
            gripper_close()
            last_command_end_time = rospy.get_rostime()
            
        if joy.buttons[6] == 1: # BACK
            calibrate(servos)
            gripper_open()
            last_command_end_time = rospy.get_rostime()
    

# Main Program

rospy.init_node('ezgripper')
rospy.loginfo("Started")

port_name = rospy.get_param('~port', '/dev/ttyUSB0')
baud = int(rospy.get_param('~baud', '57600'))
servo_ids = rospy.get_param('~servo_ids')

grip_max = 2800 #maximum open position for grippers
grip_value = grip_max
grip_min = 0
grip_step = 45 # gripper step Cross Up and Cross Down

torque_max = 350 # maximum torque - MX-64=500, MX-106=350
torque_hold = 100 # holding torque - MX-64=100, MX-106=80

dyn = USB2Dynamixel_Device(port_name, baud)
servos = [Robotis_Servo( dyn, servo_id ) for servo_id in servo_ids]

calibrate(servos)
for servo in servos: servo.write_word(30, grip_max)    # Open the gripper to maximum after calibration
rospy.sleep(2.0)

last_command_end_time = rospy.get_rostime()
rospy.Subscriber("/joy", Joy, joy_callback)

# Main Loop

r = rospy.Rate(20) # hz
while not rospy.is_shutdown():
    for servo in servos:
        try:
            servo.check_overload_and_recover()
        except CommunicationError, e:
            rospy.logerr("loop CommunicationError: %s"%e)
            servo.flushAll()
        except Exception, e:
            rospy.logerr("Exception: %s"%e)
            servo.flushAll()

    r.sleep()
    
rospy.loginfo("Exiting")
