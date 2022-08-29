#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
Copy Target Node

This node is used to copy the heading of a target. The Speed is copied in the navigator_v2.py node

Subscribes To:
    /status : PS4 controller status
    /heading: heading of the target
    /rover_heading: heading of the rover/robot

Publishes:
    /trajectory : a msg describing
                a relative angle the robot should turn towards
                and a how far to move linearly
"""
#combines the obstacle detection (nonexistent right now) and target finder nodes to produce a trajectory for the robot
import numpy as np
import cv2
import math
import time
import rospy
import jetson.utils
import sys
import datetime
import os
from modules import local_planner_apf, local_planner_vhf

from object_tracking.msg import target_loc, trajectory, camera_rot, obstacle_collection
from ds4_driver.msg import Status
time_last_recieved = time.time()
from std_msgs.msg import Float32

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

def handle_target_heading(data,container):
    '''
    Callback for reading the target's heading.
    Saves value in the container.
    
    Parameters:
    data (std_msgs.msg 'Float32'): data from gps_publisher
    container (dictionary): contains the heading for the rover and the target
    ''' 
    container['target_heading'] = data.data


def handle_rover_heading(data,container):
    '''
    Callback for reading the rovers heading.
    Saves value in the container.
    
    Parameters:
    data (std_msgs.msg 'Float32'): data from gps_publisher
    container (dictionary): contains the heading for the rover and the target
    ''' 
    container['robot_heading'] = data.data

def run():
    '''
    Main function controlling the node
    '''
    global running
    #grabs parameters
    process_rate = rospy.get_param("/process_rate")
    running = rospy.get_param("/start_running")



    rospy.init_node('copy_target', anonymous=False)
    #creates pub
    pub = rospy.Publisher("/trajectory", trajectory, queue_size = process_rate)

    #container is used so that callbacks can update information
    container = {
        "robot_heading": 0,
        "target_heading": 0

    }
    #subscribes to the target_finder with the container as an argument
    rospy.Subscriber("/status", Status, handle_controller)
    rospy.Subscriber("/heading",Float32, handle_target_heading,container)
    rospy.Subscriber("/rover_heading", Float32, handle_rover_heading,container)
    
    #creates the rate
    rate = rospy.Rate(process_rate)
    traj = trajectory()
    while not rospy.is_shutdown():

            #prevent wrapping issues
            error = container['robot_heading']-container['target_heading']
            if error < -180:
                container['robot_heading'] += 360
                traj.angular_error = container['robot_heading']-container['target_heading']
            elif error > 180:
                container['robot_heading'] -= 360
                traj.angular_error = container['robot_heading']-container['target_heading']
            else:
                traj.angular_error = error
            
            #convert to radians
            traj.angular_error = traj.angular_error*math.pi/180
            pub.publish(traj)
            rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
