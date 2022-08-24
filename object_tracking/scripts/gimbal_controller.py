#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gimbal Controller Node

Controls two servos to attempt to center the target within the D415 camera tracking the target

Subscribes To:
    /status: the status of the PS4 controller
    /raw_target_loc : the location of the person relative to the camera
Publishes:
    /target_loc_rotation_corrected : the location of the person relative to the robot
        coordiante space:
            x: left-right
            y: up-down
            z: forwards-backwards
    /camera_rot: the yaw and pitch rotation of the camera in radians
"""
import numpy as np
import math

from adafruit_servokit import ServoKit
import time
import rospy
from object_tracking.msg import target_loc, camera_rot
from ds4_driver.msg import Status

running = False

def handle_controller(msg):
    '''
    Callback for reading the PS4 controller 
    Changes global of whether or not the program is "running"
    
    Parameters:
    msg (ds4_driver.msg 'Status'): msg from the controlller
    '''
    global running
    if (msg.button_triangle == 1):
        running = True
    if (msg.button_circle == 1):
        running = False

#grab the target localtion relative to the camera
def handle_target_loc(msg, container):
    '''
    Callback to handle the current target location relative to the camera
    
    Parameters:
    msg (object_tracking.msg 'target_loc'): msg of the target loc 
    container (Dictionary): dictionary to store the data in 

    '''
    container["time_last_found"] = time.time()
    container["target_x"] = msg.target_x
    container["target_y"] = msg.target_y
    container["target_z"] = msg.target_z


def run():
    '''
    Main function for controlling the node
    '''
    global running
    process_rate = rospy.get_param("/process_rate")
    desired_distance=rospy.get_param("/desired_distance")
    running = rospy.get_param("/start_running")
    rospy.init_node('gimbal_controller', anonymous=False)
    
    #setup container
    container = {
        "target_x": 0,
        "target_y": 0,
        "time_last_found": 0,
        "target_z": 1,
    }
    #setup subscriber, publisher and rate
    rospy.Subscriber("/raw_target_loc", target_loc, handle_target_loc, container)
    pub = rospy.Publisher("/camera_rot", camera_rot, queue_size = 10)
    target_pub = rospy.Publisher("/target_loc_rotation_corrected", target_loc, queue_size=process_rate)
    rate = rospy.Rate(process_rate)

    #initialize servokit
    servokit = ServoKit(channels=16)

    #setup default yaw and pitch and scaling for their roation speed
    yaw = 90
    yaw_radians = 0
    pitch = 100
    pitch_radians =0
    yaw_scale = 5
    pitch_scale = 5
    pastXangle = 0
    pastYangle = 0

    
    while not rospy.is_shutdown():
        #if more than a second has passed reset camera pos
        if (time.time() - container["time_last_found"] > .5):
            container["target_x"] = 0
            container["target_y"] = 0
            container["target_z"] = desired_distance
            yaw = 90
            pitch = 100
            
        #rotate target by current rotation
        pitch_corrected_z = (
                container['target_z']*math.cos(pitch_radians)+
                container['target_y']*math.sin(pitch_radians))
        pitch_corrected_y = (
                container['target_y']*math.cos(pitch_radians)-
                container['target_z']*math.sin(pitch_radians))
        yaw_corrected_x = (
                container["target_x"] * math.cos(yaw_radians) +
                pitch_corrected_z         * math.sin(yaw_radians))
        yaw_corrected_z = (
                pitch_corrected_z * math.cos(yaw_radians) -
                container["target_x"] * math.sin(yaw_radians))
        rotated_target_pos = target_loc()
        rotated_target_pos.target_x = yaw_corrected_x
        rotated_target_pos.target_y = pitch_corrected_y
        rotated_target_pos.target_z = yaw_corrected_z
        
        #check if camera is outside deadband
        #if so rotate it
        xangle=math.atan(container['target_x']/container['target_z'])
        yangle=math.atan(container['target_y']/container['target_z'])
        if (abs(xangle+pastXangle) > .15):
            yaw += yaw_scale * -(xangle)
        if (abs(yangle+pastYangle) > .15):
            pitch += pitch_scale * -(yangle)
        #clamp values
        yaw = min(179, max(1, yaw))
        pitch = min(160, max(80, pitch))
        
        pastXangle= xangle
        pastYangle= yangle

        #create msg object to send rotation in radians out
        rot = camera_rot()
        yaw_radians = -((yaw-90)* math.pi/ 180)
        rot.yaw = yaw_radians
        pitch_radians = - ((pitch-100) * math.pi / 180)
        rot.pitch = pitch_radians

        pub.publish(rot)
        target_pub.publish(rotated_target_pos)

        #update servos
        servokit.servo[0].angle = yaw
        servokit.servo[1].angle = pitch
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
