#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Target Finder Node

Reads data from the D415 camera and processes it with jetson inference
to find the location of a person within each frame.

Subscribes To:
    /status : PS4 controller status
Publishes:
    /raw_target_loc : the location of the person relative to the camera
        coordinate space:
            x : -left +right
            y : -up +down
            z : +forwards -backwards

"""
import cv2
import numpy as np
import pyrealsense2
import os
import datetime
from modules.realsense_depth import *

import jetson.inference
import jetson.utils

import rospy
from object_tracking.msg import target_loc
from ds4_driver.msg import Status

import time
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

def ros_setup(rate_speed):
    '''
    Handles basic ros setup
    
    Parameters:
    rate_speed (int): the rate which the node should return

    Returns:
    rate (rospy.Rate): the rospy rate object
    target_pub (rospy.Publisher): the rospy publisher for raw target loc
    '''
    rospy.init_node('target_finder', anonymous=False)
    
    target_pub = rospy.Publisher("/raw_target_loc", target_loc, queue_size=rate_speed)
    
    
    #not sure what a good rate is, will need to play with this
    rate = rospy.Rate(rate_speed)
    return rate, target_pub

def posenet_generate_detections(frame, network, previous_pose):
    '''
    Process the image to find the target within a frame

    Parameters:
    frame (OpenCV image): the image to detect the person in
    network (jetson.inference.PoseNet): the posenet used to detect with
    previos_pose: the pose found by the last time this function was called

    Returns:
    pose (jetson.inference.PoseNet.Pose): the pose of the target
    cuda_mem (jetson.utils.CudaImage): the frame in CUDA format
    '''
    #Get the RGB version of the frame
    rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    

    #copy to cuda
    cuda_mem = jetson.utils.cudaFromNumpy(rgb)

    #processes it by cuda
    poses = network.Process(cuda_mem)
    if (len(poses) == 0):
        return None, cuda_mem
    
    
    #if there are no poses just just use the one with the most points
    if len(previous_pose) == 0:
        sorted_poses = sorted(poses,key=lambda x: len(x.Keypoints),reverse=True)
        return sorted_poses[0], cuda_mem

    else:
        #Intialize list that will hold the score of each pose
        pose_score = np.zeros(len(poses))


        pose_num = 0 #keeps track of the current pose count
        for pose in poses:
            sum_dist = 0 # total distance between current and previous pose
          
            # Loops through all the pose's keypoints
            for i in range(len(pose.Keypoints)):
                idx_1 = pose.Keypoints[i].ID # Gets the id for the keypoint
                
                #loops through all the previous pose's keypoints
                for j in range(len(previous_pose[0].Keypoints)):
                    idx_2 = previous_pose[0].Keypoints[j].ID # Get the id for the keypoint

                    #If the id for the previous pose and current pose match up, find the distance between the points
                    if idx_1==idx_2:
                        #print('Found a match')
                        #print('-----------')
                        dist_x = np.abs(pose.Keypoints[i].x - previous_pose[0].Keypoints[j].x)
                        dist_y = np.abs(pose.Keypoints[i].y - previous_pose[0].Keypoints[j].y)
                        total_dist = np.sqrt(dist_x**2 + dist_y**2)
                        sum_dist = sum_dist + total_dist #Sum up all the distances for all visible points
                        break

            pose_score[pose_num] = sum_dist #put the sum distance for the current pose in list
            pose_num = pose_num + 1 #Interate to next pose
        min_score = min(pose_score) #Get the max score (sum_dist)
        min_index = np.argmin(pose_score) #Get the index of the min score
        
        return poses[min_index], cuda_mem

def process_detection(detection, depth_frame, distance_average_size):
    '''
    Find the relative position of the target from a detection

    Parameters:
    detection (jetson.inference.PoseNet.Pose): the pose of the person
    depth_frame (OpenCV image): the depth array of the current image
    distance_average_size: the square radius used to calculate the average depth around each point

    Returns:
    centerx (float): the number of pixels from the left of the image 
    centery (float): the number of pixels from the top of the image
    distance (float): the depth in mm from the camera
    '''
    totalX = 0
    totalY = 0
    dist_avgs = []
    #loop through each keypoint
    for keypoint in detection.Keypoints:
        #add x and y to totals
        totalX += keypoint.x
        totalY += keypoint.y
        #grab a box around the center in the depth frame
        distance_box = depth_frame[
            int(max(0,keypoint.y - distance_average_size)):int(min(720, keypoint.y + distance_average_size)),
            int(max(0,keypoint.x - distance_average_size)):int(min(1280, keypoint.x + distance_average_size))
        ]
        #append the average depth of this box to list of averages
        dist_avgs.append(np.mean(np.maximum(distance_box,400)))
    #take median
    distance = np.median(dist_avgs)
    #compute the x and y coords 
    centerx = totalX/len(detection.Keypoints)
    centery = totalY/len(detection.Keypoints)
    #return this ifno
    return centerx, centery, distance

def run():
    '''
    Main function controlling the target_finder node
    '''
    global running
    #get parameters
    process_rate = rospy.get_param("/process_rate")
    running = rospy.get_param("/start_running")
    camera_width = rospy.get_param("/camera_width")
    camera_height = rospy.get_param("/camera_height")

    #create folder for logging
    folder = rospy.get_param("/foldername")
    if (folder == ""):
       folder = datetime.datetime.now().strftime("%m-%d-%H_%M_%S")

    foldername = '/home/rover/logs/'+folder+'/'
    #prevent errors from existing if the folder already exists
    try:
        os.mkdir(foldername)
    except FileExistsError:
        pass
    distance_average_size = rospy.get_param("/distance_average_size")
    #initialize ros stuffs
    rate, target_pub = ros_setup(process_rate)
    rospy.Subscriber("/status", Status, handle_controller)
    #create camera, specifying which one
    depth_cam = DepthCamera(camera_width, camera_height, 'd415')
    intrinsics = depth_cam.get_intrinsics()
    #list of network parameters, none right now but might want to add ones
    network_parameters = []
    #creates network
    net = jetson.inference.poseNet("resnet18-body", network_parameters)
    #creates the video output
    video = jetson.utils.videoOutput(foldername+"video.mp4")
    
    #Initializes the previous_pose variable
    previous_pose = []
    font = jetson.utils.cudaFont()

    while not rospy.is_shutdown():
        #get the enxt frames
        ret, depth_frame, col_frame, trash, more_trash = depth_cam.get_frame()
        #convert to cuda image and get pose detections
        detection, cuda_img = posenet_generate_detections(col_frame, net, previous_pose)

        
        if (detection != None):
            time_last_detected = time.time()
            #process the pose
            centerx,centery, distance = process_detection(detection, depth_frame, distance_average_size)
            #setup the msg to publish
            loc = target_loc()
            loc.target_x = (centerx-intrinsics.ppx) /intrinsics.fx * distance
            loc.target_y = (centery-intrinsics.ppy) /intrinsics.fy * distance
            loc.target_z = distance
            target_pub.publish(loc)
            #Storing previous pose
            previous_pose = [detection]
            jetson.utils.cudaDrawCircle(cuda_img, (centerx,centery), 15, (0,255,127,200))
        #if running process the videoOutput
        if running == True:
            font.OverlayText(cuda_img,1280, 720, datetime.datetime.now().strftime("Time: %H_%M_%S"), 25, 25, (255,255,255,255), (0,0,0,0))
            video.Render(cuda_img)
            
        rate.sleep()
#Good Practice for python
#Something Something prevents unexpected code calling
if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
