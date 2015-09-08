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

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

# http://docs.ros.org/indigo/api/control_msgs/html/msg/GripperCommand.html
# float64 position
# float64 max_effort
#
# http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html
# GripperCommand command
# ---
# float64 position  # The current gripper gap size (in meters)
# float64 effort    # The current effort exerted (in Newtons)
# bool stalled      # True iff the gripper is exerting max effort and not moving
# bool reached_goal # True iff the gripper position has reached the commanded setpoint
# ---
# float64 position  # The current gripper gap size (in meters)
# float64 effort    # The current effort exerted (in Newtons)
# bool stalled      # True iff the gripper is exerting max effort and not moving
# bool reached_goal # True iff the gripper position has reached the commanded setpoint

def gripper_open_step():
    global grip_value
    
    grip_value = grip_value + grip_step
    if grip_value > grip_max:
        grip_value = grip_max
        
    rospy.loginfo("client: goto position %.3f"%grip_value)
    goal = GripperCommandGoal()
    goal.command.position = grip_value
    goal.command.max_effort = 28.0
    client.send_goal_and_wait(goal)
    rospy.loginfo("client: goto position done")

def gripper_close_step():
    global grip_value
    
    grip_value = grip_value - grip_step
    if grip_value < grip_min:
        grip_value = grip_min
        
    rospy.loginfo("client: goto position %.3f"%grip_value)
    goal = GripperCommandGoal()
    goal.command.position = grip_value
    goal.command.max_effort = 28.0
    client.send_goal_and_wait(goal)
    rospy.loginfo("client: goto position done")

def joy_callback(joy):
    global last_command_end_time, grip_value

    if not joy.buttons:
        return # Don't break on an empty list

    if (rospy.get_rostime() - last_command_end_time).to_sec() > 0.2:
        # This check should flush all messages accumulated during command execution
        # and avoid executing it again.
        
        if joy.buttons[0] == 1: # A - hard close
            rospy.loginfo("client: hard close")
            goal = GripperCommandGoal()
            goal.command.position = 0.0
            goal.command.max_effort = 28.0
            client.send_goal_and_wait(goal)
            rospy.loginfo("client: hard close done")
            last_command_end_time = rospy.get_rostime()
            grip_value = grip_min
            
        if joy.buttons[3] == 1: # Y - soft close
            rospy.loginfo("client: soft close")
            goal = GripperCommandGoal()
            goal.command.position = 0.0
            goal.command.max_effort = 12.0
            client.send_goal_and_wait(goal)
            rospy.loginfo("client: soft close done")
            last_command_end_time = rospy.get_rostime()
            grip_value = grip_min
                
        if joy.buttons[1] == 1: # B - open
            rospy.loginfo("client: open")
            goal = GripperCommandGoal()
            goal.command.position = 0.20   # more than max
            goal.command.max_effort = 12.0 # any non 0.0
            client.send_goal_and_wait(goal)
            rospy.loginfo("client: open done")
            last_command_end_time = rospy.get_rostime()
            grip_value = grip_max
    
        if joy.buttons[2] == 1: # X - release
            rospy.loginfo("client: release")
            goal = GripperCommandGoal()
            goal.command.position = 0.0 # any
            goal.command.max_effort = 0.0
            client.send_goal_and_wait(goal)
            rospy.loginfo("client: release done")
            last_command_end_time = rospy.get_rostime()
    
        if joy.buttons[6] == 1: # BACK - Calibrate
            rospy.loginfo("client: calibrate")
            calibrate()
            rospy.loginfo("client: calibrate done")
            last_command_end_time = rospy.get_rostime()
            grip_value = grip_max

        if joy.buttons[13] == 1: # xpad driver mapping
        #if joy.axes[7] == 1.0: # xboxdrv mapping
            gripper_open_step()
            last_command_end_time = rospy.get_rostime()
                 
        if joy.buttons[14] == 1:
        #if joy.axes[7] == -1.0:
            gripper_close_step()
            last_command_end_time = rospy.get_rostime()

rospy.init_node("ezgripper_client")

rospy.loginfo("Waiting for gripper action server...")
client = actionlib.SimpleActionClient("gripper", GripperCommandAction)
client.wait_for_server(rospy.Duration(60))
rospy.loginfo("Connected to server")

rospy.loginfo("Waiting for service calibrate...")
rospy.wait_for_service('calibrate')
calibrate = rospy.ServiceProxy('calibrate', Empty)
rospy.loginfo("Connected to service calibrate")

grip_max = 0.17 #maximum open position for grippers
grip_value = grip_max
grip_min = 0.0
grip_step = grip_max/5 # gripper step Cross Up and Cross Down

last_command_end_time = rospy.get_rostime()
rospy.Subscriber("/joy", Joy, joy_callback)

rospy.spin()

rospy.loginfo("Exiting")
