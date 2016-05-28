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
    while not rospy.is_shutdown():
        current_position = servo.read_encoder()
        if current_position == last_position:
            break
        last_position = current_position
        rospy.sleep(0.1)                         
        
        if (rospy.get_rostime() - wait_start).to_sec() > .2:
            break

def calibrate_srv(gripper, msg):
    rospy.loginfo("Calibrate service: request received")
    gripper.calibrate()
    gripper.open()    
    rospy.loginfo("Calibrate service: request completed")
    return EmptyResponse()
    
def servo_position_from_gap(gap):
    # servo position from gap is a 0-100% range (0.0-1.0).  Because the EZGripper utilizes a rotation grasp instead of a parallel grasp, the definition of gap is dependent on where the gap is measured.  Using the 0-100% range allows the user to define the definition of where the gap is measured.
    position = gap
    if position < 0: position = 0  #Full close
    if position > 1: position = 1  #Full open
    position = position * grip_max #this translates percent to servo range
    return int(position)
    
class Gripper:
    def __init__(self, name, servo_ids):
        self.name = name
        self.servos = [Robotis_Servo( dyn, servo_id ) for servo_id in servo_ids]
    
    def calibrate(self):
        rospy.loginfo("Calibrating: " + self.name)
        
        for servo in self.servos:
            servo.write_address(6, [255,15,255,15] )   # 1) "Multi-Turn" - ON
            servo.write_word(34, 500)                  # 2) "Torque Limit" to 500 (or so)
            servo.write_address(24, [0])               # 3) "Torque Enable" to OFF
            servo.write_address(70, [1])               # 4) Set "Goal Torque Mode" to ON
            servo.write_word(71, 1024 + 100)           # 5) Set "Goal Torque" Direction to CW and Value 100
        
        rospy.sleep(4.0)                               # 6) give it time to stop
        
        for servo in self.servos:
            servo.write_word(71, 1024 + 10)            # 7) Set "Goal Torque" Direction to CW and Value 10 - reduce load on servo
            servo.write_word(20, 0)                    # 8) set "Multi turn offset" to 0   
            position = servo.read_word(36)             # 9) read current position of servo
            servo.write_word(20, -position)
            servo.write_address(70, [0])               # Stopping torque here improves makes writing "multi-word offset" consistent
            
        rospy.loginfo("Calibration completed")
    
    def open(self):
        for servo in self.servos:
            set_torque_mode(servo, False)
            servo.write_word(30, grip_max)
        rospy.sleep(1.0)
    
    def close(self, closing_torque):
        for servo in self.servos:
            self.set_max_effort(closing_torque)
            set_torque_mode(servo, True)
            
        wait_for_stop(self.servos[0])
        
        for servo in self.servos:
            holding_torque = min(torque_hold/10.23, closing_torque) # torque_hold is defined in MX-64 1024 range (so divide by 10.23 to move to 0-100 range)
            self.set_max_effort(holding_torque) 
    
    def set_max_effort(self, max_effort):
             # sets torque for moving to position (moving_torque) and for torque only mode (torque_mode_max_effort)
             # range 0-100% (0-100) - this range is in 0-100 whole numbers so that it can be used where Newton force is expected
        if max_effort > 100: 
            max_effort = 100
            rospy.loginfo("max_effort cannot be > 100, overriding to 100")
        if max_effort < 0: 
            max_effort = 0
            rospy.loginfo("max_effort cannot be < 0, overriding to 0")
        for servo in self.servos:
            moving_torque = int(max_effort*10.23) # max dynamixel torque is 0-1023(unitless) so 100 * 10.23 is maximum value of dynamixel force
            servo.write_word(34, moving_torque) # torque for moving to position,and due to Dynamixel architecture, this also limits max value for register 71 below

            rospy.loginfo("EZGripper moving torque: %d" %moving_torque) 

            torque_mode_max_effort = int(max_effort * 10.23)
            if torque_mode_max_effort > torque_max: torque_mode_max_effort = int(torque_max) # torque limited for torque mode because it is generally not needed and reduces system stress
            servo.write_word(71, 1024+torque_mode_max_effort) # torque mode of closing gripper
            rospy.loginfo("EZGripper goal torque: %d" %torque_mode_max_effort) 

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
        rospy.loginfo("Execute goal: position=%.3f, grip_max=%d max_effort=%.3f"%
                      (goal.command.position, grip_max, goal.command.max_effort))
        
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
        
        if goal.command.position == 0.0:
            rospy.loginfo("Close: start")
            closing_torque = goal.command.max_effort
            rospy.loginfo("Using closing torque %.2f"%closing_torque)
            self.gripper.close(closing_torque)
            result = GripperCommandResult()
            result.position = goal.command.position
            result.effort = 13.0
            result.stalled = False
            result.reached_goal = True
            self.action_server.set_succeeded(result)
            rospy.loginfo("Close: done")
            return
        
        rospy.loginfo("Go to position: start")
                 #servo_position = servo_position_from_gap(goal.command.position)
        servo_position = servo_position_from_gap(goal.command.position)
        rospy.loginfo("Target position: %.3f (%d)"%(goal.command.position, servo_position))
        closing_torque = int(goal.command.max_effort) # TODO: need a more accurate calculation
        self.gripper.set_max_effort(closing_torque)  # essentially sets velocity of movement, but also sets max_effort for initial half second of grasp.
        self.gripper.goto_position(int(servo_position))
        # sets torque to keep gripper in position, but does not apply torque if there is no load.  This does not provide continuous grasping torque.
        holding_torque = min(torque_hold, closing_torque)
        self.gripper.set_max_effort(holding_torque) #
        result = GripperCommandResult()
        result.position = goal.command.position #not necessarily the current position of the gripper if the gripper did not reach its goal position.
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

diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

#distance_between_fingers = 0.05 # meters
#finger_link_length = 0.065
#max_gap = distance_between_fingers + finger_link_length * 2

grip_max = 2500 #maximum open position for grippers
#grip_angle_max = radians(102) # 102 degrees, in radians
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
        #servo.write_address(28,255) # Set PID P=255
        #servo.write_address(27,[0]) # Set PID I=0
        #servo.write_address(26,[0,0,255]) # Set PID D=0

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
