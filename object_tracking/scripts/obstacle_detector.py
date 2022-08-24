#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Detector Node

Uses the D435 camera to find obstacles in front of the robot
there is definitely something wierd in this script. 
Also broadcasts the imu data from the d435i camera

It sometimes crashes with errors that don't really make sense
it seems like there might be a cuda function called by pyrealsense2 that fails, 
I dont know, maybe my cuda usage is somehow modifies some cuda state 
that pyrealsense2 doesn't expect to be modified?
hasnt happened in a while though...

Subscribes To:
    /status: the status of the PS4 controller
Publishes:
    /obstacles: a list of circlular obstacles relative to the rover
        coordinate space:
            x: -left +right
            z: +forwards -backwards
    /d435_imu: imu data from the d435i
"""

#camera related imports
import cv2
import numpy as np
import pyrealsense2
from modules.realsense_depth import *

#jetson imports
import jetson.inference
import jetson.utils

#ROS imports
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from object_tracking.msg import obstacle, obstacle_collection
from ds4_driver.msg import Status

#default python package imports
import sys
import time
import os
import datetime

#running global flag
running = False

#checks controller to change from paused to running
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

#handles ros setup stuff, returns the publishers and rate
def ros_setup(rate_speed):
    
    '''
    Handles basic ros setup
    
    Parameters:
    rate_speed (int): the rate which the node should return

    Returns:
    rate (rospy.Rate): the rospy rate object
    imu_pub (rospy.Publisher): the rospy publisehr for the imu
    obs_pub (rospy.Publisher): the rospy publisher for the obstacles
    '''
    #Create ROS node
    rospy.init_node('obstacle_detector', anonymous=False)
    
    #creates publishers
    imu_pub = rospy.Publisher("/d435_imu", Imu, queue_size=rate_speed)
    obs_pub = rospy.Publisher("/obstacles", obstacle_collection, queue_size=rate_speed)
    rospy.Subscriber("/status", Status, handle_controller)
    
    #not sure what a good rate is, will need to play with this
    rate = rospy.Rate(rate_speed)
    return rate, imu_pub, obs_pub

def create_IMU_message(accel, gyro):
    '''
    Creates an IMU message from accel and gyro data

    Parameters:
    accel (list): list of accel data (x, y, z)
    gyro (list): list of gyro data (x,y,z)

    Returns:
    msg (sensor_msgs.msg 'Imu'): rospy imu message
    '''
    #create msg header
    msg = Imu()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    #put gryo and accel info into msg
    msg.angular_velocity.x = gyro[0]
    msg.angular_velocity.y = gyro[1]
    msg.angular_velocity.z = gyro[2]
    msg.angular_velocity_covariance = np.zeros(9)
    msg.angular_velocity_covariance[0] = .00017
    msg.angular_velocity_covariance[4] = .00017
    msg.angular_velocity_covariance[8] = .00017

    msg.linear_acceleration.x = accel[0]
    msg.linear_acceleration.y = accel[1]
    msg.linear_acceleration.z = accel[2]
    msg.linear_acceleration_covariance = np.zeros(9)
    msg.linear_acceleration_covariance[0] = .01
    msg.linear_acceleration_covariance[4] = .01
    msg.linear_acceleration_covariance[8] = .01
    return msg

class ObstacleCircle:
    '''
    Helper class to create circles to organize obstacles into
    '''
    def __init__(self,firstpoint, x):
        '''
        Creates a new circle with a single point
        '''
        self.points = [firstpoint]
        self.diameter = 0
        self.first_x = x
        self.last_x =x
    def attemptToAddPoint(self, trialpoint, max_diameter, x):
        '''
        Tries to add a point to the circle, unless it would make the resultant circle too large
        '''
        test_diameter = (self.points[0][0] - trialpoint[0])**2 + (self.points[0][2] - trialpoint[2])**2 
        if test_diameter < max_diameter ** 2:
            self.points.append(trialpoint)
            self.diameter = max(test_diameter, self.diameter)
            self.last_x = x
            return True
        else:
            return False

def organizeIntoCircles(arr, max_distance, max_size, min_point_amount, pixel_border):
    '''
    Organizes distances to obstacle for each column into a set of circles

    Parameters:
    arr (list): list of xz coordinates corresponding to depth to nearest obstacle for each column of the image
    max_distance (int): max z distance to care about obstacles
    max_size: max size a cirlce can be before splitting into two smaller ones
    min_point_amount: the number of points needed by each circle to not be rejected as noise

    Returns:
    finalCircleList (list): list of Circles
    '''
    #stores list of completed circles
    completedCircles = []
    #current circle
    currentCircle = None
    #loop through array
    for x in range(arr.shape[0]):
        
        #if the z coordinate of that point is within the max depth
        if arr[x][2] < max_distance:
            #if there is no current cirlce create a new one
            if (currentCircle == None):
                currentCircle = ObstacleCircle(arr[x],x+pixel_border)
            #else try to add to existing cirlce
            else:
                if not currentCircle.attemptToAddPoint(arr[x], max_size, x+pixel_border):
                    #if you cant add, create a new circle
                    completedCircles.append(currentCircle)
                    currentCircle = ObstacleCircle(arr[x],x+pixel_border)
    #if there is a still a current circle at the end
    if (currentCircle != None):
        completedCircles.append(currentCircle)
    #final circle list keeps the circles with enough points to not be assumed to be noise obstacles
    finalCircleList = []
    for circle in completedCircles:
        if len(circle.points) > min_point_amount:
            finalCircleList.append(circle)
            

    return finalCircleList

def setup_cam():
    """
    theoretically, this set up cam function exists 
    so that way the cam be set up before calling run() 
    so that it the case of a ROSInterruptException 
    the camera pipeline can be closed and the camera 
    will not stop working until being re-plugged in. This didn't work...
    """
    camera_width = rospy.get_param("/obs_camera_width")
    camera_height = rospy.get_param("/obs_camera_height")
    d435_cam = DepthCamera(camera_width, camera_height, 'd435')
    return d435_cam

def run(d435_cam):
    '''
    Main Run Function
    '''
    global running
    
    #get a lot of parameters
    camera_width = rospy.get_param("/obs_camera_width")
    camera_height = rospy.get_param("/obs_camera_height")
    process_rate = rospy.get_param("/process_rate")
    running = rospy.get_param("/start_running")
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
    #border is the max of several of the image functions
    #basically the true image falls within the left top right down coords of (border, border, camera_width-border, camera_depth-border)
    #everything outside that border is potentially random
    border = max(height_change_radius, first_filter_radius, second_filter_radius)

    folder = rospy.get_param("/foldername")
    if (folder == ""):
       folder = datetime.datetime.now().strftime("%m-%d-%H_%M_%S")
    foldername = '/home/rover/logs/'+folder+'/'
    #several nodes try to create the same folder so this way they will not fail if created before
    try:
        os.mkdir(foldername)
    except FileExistsError:
        pass
    video_output = jetson.utils.videoOutput(foldername+"obstacle.mp4", argv=sys.argv)

    #allocates image buffers for various stages of computation
    #technically should be able to get away with only the pointcloud and two that flip back and forth
    #but this is clearer and I encountered bugs

    pointcloud_img = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    buffer_img = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    height_mask = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    first_filter = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    small_obstacle_filter = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    second_filter = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')
    final_obstacles = jetson.utils.cudaImage(width=camera_width, height=camera_height, format='rgb32f')



    #setup ros
    rate, imu_pub, obs_pub = ros_setup(process_rate)

    #intrinsics are the cameras values about its focal length and pixel center
    intrinsics = d435_cam.get_intrinsics()
    font = jetson.utils.cudaFont()


    while not rospy.is_shutdown():
        #grab frame of the camera, I have run into the error where it just doesnt grab new frames?
        #I dont know what to do about that
        ret, depth_frame, col_frame, accel, gyro = d435_cam.get_frame()
        #grab the imu data
        imu_msg = create_IMU_message(accel, gyro)

        #convert depth to nvidia
        rgb = cv2.cvtColor(col_frame,cv2.COLOR_BGR2RGB)
        cuda_color = jetson.utils.cudaFromNumpy(rgb)
        cuda_depth = jetson.utils.cudaFromNumpy(depth_frame)


        #convert from grayscale to an rgb image that cuda expects
        jetson.utils.cudaConvertColor(cuda_depth, buffer_img)
        #deproject the pointcloud
        jetson.utils.cudaDeproject(buffer_img, pointcloud_img, 
                intrinsics.fx,intrinsics.fy,intrinsics.ppx,intrinsics.ppy)

        #mask height changes
        jetson.utils.cudaMaskHeightChanges(pointcloud_img, height_mask, 
                height_change_radius, height_change_threshold, max_depth)

        #run the first noise filter
        jetson.utils.cudaCustomNoiseFilter(height_mask, first_filter, 
                first_filter_radius, first_filter_cutoff, max_depth)

        #bridge gaps caused by small obstacles
        jetson.utils.cudaSmallObstacleFilter(first_filter, small_obstacle_filter, 
                pointcloud_img, small_obstacle_pixels_needed, max_obstacle_height, 
                max_depth_change, top_min_dist, allowed_slope)

        #run the second filter
        jetson.utils.cudaCustomNoiseFilter(small_obstacle_filter, second_filter, 
                second_filter_radius, second_filter_cutoff, max_depth)
        
        #actualy find the obstacles for each column
        jetson.utils.cudaComputeActualObstacles(second_filter, final_obstacles, 
                pointcloud_img, border, max_depth)

        #convert those obstacles to a numpy array, yeah this array contains the whole image while the day exists within one row. T
        npObstacles = jetson.utils.cudaToNumpy(final_obstacles)
        #send the obstacle array to be processed into circles
        circles = organizeIntoCircles(npObstacles[border, border:-border,:], max_depth, max_obstacle_size, min_pixel_amount, border)
        obstacles = []
        #grab data from cirlces and put them into obstale message
        for circle in circles:
            ob = obstacle()
            ob.center_x = (circle.points[-1][0]+ circle.points[0][0])/2
            ob.center_z = (circle.points[-1][2]+ circle.points[0][2])/2
            ob.radius = (circle.diameter**.5/2)
            obstacles.append(ob)
        ob_collection = obstacle_collection()
        ob_collection.obstacles = obstacles
        
        if running == True:
            font.OverlayText(cuda_color,1280, 720, datetime.datetime.now().strftime("Time: %H_%M_%S"), 25, 25, (255,255,255,255), (0,0,0,0))
            video_output.Render(cuda_color)
            obs_pub.publish(ob_collection)
            imu_pub.publish(imu_msg)


        rate.sleep()


    d435_cam.release()

if __name__ == "__main__":
    cam = None
    try:
        cam = setup_cam()
        run(cam)
    except rospy.ROSInterruptException:
        cam.release()
