#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
Local Planner Node

Combines the target location data and obstacle location data
produces a trajectory for the robot to head towards

Subscribes To:
    /status : PS4 controller status
    /target_loc_rotation_corrected : the position of the target relative to the robot
    /obstacles : the collection of obstacles in front of the robot

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

def handle_target(msg, container):
    '''
    Callback for reading the target position

    Parameters:
    msg (object_tracking.target_loc): the rotation corrected target location
    container (Dictionary): the dictionary to store the data in
    '''
    global time_last_recieved
    curr_time = time.time()
    container["target_x"] = msg.target_x
    container["target_z"] =msg.target_z
    time_last_recieved = time.time()

def handle_obstacles(msg, container):
    '''
    Callback for reading the obstacles

    Parameters:
    msg (object_tracking.obstacle_collection): the list of obstacles in front of the robot
    container (Dictionary): the dictionary to store the data in
    '''
    container['obstacles'] =msg.obstacles


def run():
    '''
    Main function controlling the node
    '''
    global running
    #grabs parameters
    process_rate = rospy.get_param("/process_rate")
    desired_distance = rospy.get_param("/desired_distance")
    running = rospy.get_param("/start_running")
    max_depth = rospy.get_param("/max_depth")
    robot_radius = rospy.get_param("/robot_radius")
    folder = rospy.get_param("/foldername")

    influence_radius = rospy.get_param("/influence_radius")
    max_attr = rospy.get_param("/max_attr")
    k_attr = rospy.get_param("/k_attr")
    k_repl = rospy.get_param("/obstacle_k_repl")
    gamma = rospy.get_param("/gamma")
    persistence_percent = rospy.get_param("/persistence_percent")
    path_planner = rospy.get_param("/path_planner")

    if (folder == ""):
       folder = datetime.datetime.now().strftime("%m-%d-%H_%M_%S")

    foldername = '/home/rover/logs/'+folder+'/'
    #makes sure another node didn't already create the folder
    try:
        os.mkdir(foldername)
    except FileExistsError:
        pass
    video_output = jetson.utils.videoOutput(foldername+"top_down.mp4", argv=sys.argv)
    #creates node
    rospy.init_node('local_planner', anonymous=False)
    #creates pub
    pub = rospy.Publisher("/trajectory", trajectory, queue_size = process_rate)

    #container is used so that callbacks can update information
    container = {
        "target_x": 0,
        "target_z": desired_distance,
        "obstacles" : [],

    }
    #subscribes to the target_finder with the container as an argument
    rospy.Subscriber("/target_loc_rotation_corrected", target_loc, handle_target, container)
    rospy.Subscriber("/obstacles", obstacle_collection, handle_obstacles, container)
    rospy.Subscriber("/status", Status, handle_controller)
    #creates the rate
    rate = rospy.Rate(process_rate)


    
    
    
    while not rospy.is_shutdown():
        #
        dt = time.time() - time_last_recieved
        #if more than a second has passed than send a trajectory of zero
        if (dt > 1):
            container["target_x"] = 0
            container["target_z"] = desired_distance
        
        robot_rel_x_pos = container['target_x']
        robot_rel_z_pos = container['target_z']
        current_obstacles = container['obstacles']

        target_angle = math.atan(robot_rel_x_pos/ robot_rel_z_pos)
        goal_vec = (robot_rel_x_pos - math.sin(target_angle) * desired_distance, 
                    robot_rel_z_pos - math.cos(target_angle) * desired_distance)
        

        
        if (running):
            traj = None
            if (path_planner=="apf"):
                traj = local_planner_apf.compute(goal_vec, target_angle, current_obstacles, robot_radius, max_depth, video_output, influence_radius, k_attr, k_repl, max_attr, max_attr, gamma, persistence_percent)

            elif (path_planner=="vhf"):
                traj = local_planner_vhf.compute(goal_vec, target_angle, current_obstacles, robot_radius, max_depth, video_output)


            pub.publish(traj)
            rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
