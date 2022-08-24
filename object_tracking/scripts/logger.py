#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logger Node

Collects data from a lot of places and logs it to a csv

Subscribes To:
    /status: PS4 controler status
    /raw_target_loc: target location relative to camera
    /trajectory: error to desired distance and angle
    /cmd_vel: robot commanded velocity
    /odom/ekf/enk_imu: kalmin-filtered robot odometry
    /camera_rot: rotation of the d415 camera
"""

import rospy
import csv
import datetime
import time
import os

from object_tracking.msg import target_loc, trajectory, camera_rot
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ds4_driver.msg import Status

recieved_update = False
running = False
start_time = 0
last_recieved_detection = time.time()

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
def handle_target_loc(msg, container):
    '''
    Callback to handle the current target location relative to the camera
    
    Parameters:
    msg (object_tracking.msg 'target_loc'): msg of the target loc 
    container (Dictionary): dictionary to store the data in 

    '''
    global recieved_update, last_recieved_detection
    recieved_update = True
    t = time.time()
    last_recieved_detection = t
    container["target_x"] = msg.target_x
    container["target_y"] = msg.target_y
    container["target_z"] = msg.target_z
    container["time"] = t -start_time

def handle_trajectory(msg, container):
    '''
    Callback for reading trajectory info

    Parameters:
    trajectory (object_tracking.msg trajectory): current desired relative location of robot
    container (Dictionary): container to store info in

    '''
    global recieved_update
    recieved_update = True
    container["angular_error"] = msg.angular_error
    container["linear_error"] = msg.linear_error
    container["time"] = time.time() -start_time


def handle_cmd_vel(msg, container):
    '''
    Callback for reading current commanded velocity

    Parameters:
    msg (geometry_msgs.msg Twist): current commanded velocity of the object
    container (Dictionary): container to store info in

    '''
    global recieved_update
    recieved_update = True
    container["linear_commanded"] = msg.linear.x
    container["angular_commanded"] = msg.angular.z
    container["time"] = time.time() -start_time

def handle_odometry(msg, container):
    '''
    Callback for handlng odometry info

    Parameters:
    msg (sensor_msgs.msg Odometry): odemetry msg from the robot
    container (dictionary): container to store data in

    '''
    global recieved_update
    recieved_update = True
    container["robot_linear_vel"] = msg.twist.twist.linear.x

def handle_rotation(msg, container):
    '''
    Callback for reading camera rot

    Parameters:
    msg (object_tracking.msgs camera_rot): current roation of target tracking camera
    container (Dictionary): container to store info in

    '''
    global recieved_update
    recieved_update = True
    container['cam_yaw'] = msg.yaw
    container['cam_pitch'] = msg.pitch
    

def run():
    '''
    Main function controlling logger
    '''
    global recieved_update, start_time, running

    process_rate = rospy.get_param("/process_rate")
    folder = rospy.get_param("/foldername")
    running = rospy.get_param("/start_running")
    
    linear_p_gain = rospy.get_param("/linear_p_gain")
    linear_d_gain = rospy.get_param("/linear_d_gain")
    linear_deadband = rospy.get_param("/linear_deadband")
    linear_max = rospy.get_param("/max_linear")

    angular_p_gain = rospy.get_param("/angular_p_gain")
    angular_d_gain = rospy.get_param("/angular_d_gain")
    angular_deadband = rospy.get_param("/angular_deadband")
    angular_max = rospy.get_param("/max_angular")
    const_vel = rospy.get_param("/const_vel")

    max_depth = rospy.get_param("/max_depth")
    robot_radius = rospy.get_param("/robot_radius")
    
    height_change_radius = rospy.get_param("/height_change_radius")
    height_change_threshold = rospy.get_param("/height_change_threshold")
    max_depth = rospy.get_param("/max_depth")
    first_filter_radius = rospy.get_param("/first_filter_radius")
    first_filter_cutoff = rospy.get_param("/first_filter_cutoff")
    small_obstacle_pixels_needed = rospy.get_param("/small_obstacle_pixels_needed")
    max_obstacle_height = rospy.get_param("/max_obstacle_height")
    max_depth_change = rospy.get_param("/max_depth_change")
    top_min_dist = rospy.get_param("/top_min_dist")
    allowed_slope = rospy.get_param("/allowed_slope")
    second_filter_radius = rospy.get_param("/second_filter_radius")
    second_filter_cutoff = rospy.get_param("/second_filter_cutoff")
    max_obstacle_size = rospy.get_param("/max_obstacle_size")
    min_pixel_amount = rospy.get_param("/min_pixel_amount")




    path_planner = rospy.get_param("/path_planner")
    k_attr = rospy.get_param("/k_attr")
    max_attr = rospy.get_param("/max_attr")
    obstacle_k_repl = rospy.get_param("/obstacle_k_repl")
    gamma = rospy.get_param("/gamma")
    influence_radius = rospy.get_param("/influence_radius")
    
    persistence_percent = rospy.get_param("/persistence_percent")
    

    if (folder == ""):
        folder = datetime.datetime.now().strftime("%m-%d-%H_%M_%S")
    foldername = '/home/rover/logs/'+folder + '/'
    try:
        os.mkdir(foldername)
    except FileExistsError:
        pass
    #container storing all time series data
    container = {
            "time" : 0,
            "target_x" : 0,
            "target_y" : 0,
            "target_z"    : 0,
            "angular_error"     : 0,
            "linear_error"     : 0,
            "linear_commanded"    : 0,
            "angular_commanded"   : 0,
            "time_since_last_detection" : 0,
            "robot_linear_vel" : 0,
            "cam_yaw" : 0,
            "cam_pitch" : 0,
    }
    #list containing the same names as in the container
    #must contain exact same names
    fieldnames = ['time', 'time_since_last_detection', 'target_x', 'target_y','target_z','angular_error','linear_error','linear_commanded','angular_commanded','robot_linear_vel','cam_yaw','cam_pitch']

    start_time = time.time()
    rospy.init_node('logger', anonymous=False)
    rospy.Subscriber("/raw_target_loc", target_loc, handle_target_loc, container)
    rospy.Subscriber("/trajectory", trajectory, handle_trajectory, container)
    rospy.Subscriber("/cmd_vel", Twist, handle_cmd_vel, container)
    rospy.Subscriber("/status", Status, handle_controller)
    rospy.Subscriber("/odom/ekf/enc_imu", Odometry, handle_odometry, container)
    rospy.Subscriber("/camera_rot", camera_rot, handle_rotation, container)
    rate = rospy.Rate(process_rate)

    #write constants
    with open (foldername+'constants.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        #write header for constants
        #make sure the two following writerows write variables in the exact same order
        writer.writerow(
                [
                    "process_rate",
                    "linear_p_gain",
                    "linear_d_gain",
                    "linear_deadband",
                    "linear_max",
                    "angular_p_gain",
                    "angular_d_gain",
                    "angular_deadband",
                    "angular_max",
                    "const_vel",
                    'max_depth',
                    'robot_radius',
                    'height_change_radius',
                    'height_change_threshold',
                    'max_depth',
                    'first_filter_radius',
                    'first_filter_cutoff',
                    'small_obstacle_pixels_needed',
                    'max_obstacle_height',
                    'max_depth_change',
                    'top_min_dist',
                    'allowed_slope',
                    'second_filter_radius',
                    'second_filter_cutoff',
                    'max_obstacle_size',
                    'min_pixel_amount',
                    'path_planner',
                    'k_attr',
                    'max_attr/max_force',
                    'obstacle_k_repl',
                    'gamma',
                    'persistence_percent',
                    'obstacle_influence_radius',
                ]
            )
        #write actual constant values
        writer.writerow(
                [
                    process_rate,
                    linear_p_gain,
                    linear_d_gain,
                    linear_deadband,
                    linear_max,
                    angular_p_gain,
                    angular_d_gain,
                    angular_deadband,
                    angular_max,
                    const_vel,
                    max_depth,
                    robot_radius,
                    height_change_radius,
                    height_change_threshold,
                    max_depth,
                    first_filter_radius,
                    first_filter_cutoff,
                    small_obstacle_pixels_needed,
                    max_obstacle_height,
                    max_depth_change,
                    top_min_dist,
                    allowed_slope,
                    second_filter_radius,
                    second_filter_cutoff,
                    max_obstacle_size,
                    min_pixel_amount,
                    path_planner,
                    k_attr,
                    max_attr,
                    obstacle_k_repl,
                    gamma,
                    persistence_percent,
                    influence_radius,
                ]
            )
    #open file for time series writing
    with open (foldername+'log.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames)
        writer.writeheader()
        while not rospy.is_shutdown():
            if recieved_update and running:
                container["time_since_last_detection"] = time.time()-last_recieved_detection

                writer.writerow(container)
                recieved_update = False
            rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
