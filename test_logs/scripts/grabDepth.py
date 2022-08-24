#!/usr/bin/env python3
#This script is just a toy script to play around with custom cuda commands
import numpy as np

import cv2
import pyrealsense2
from realsense_depth import DepthCamera

import jetson.inference
import jetson.utils

import sys
import time

class ObstacleCircle:
    def __init__(self,firstpoint, x):
        self.points = [firstpoint]
        self.diameter = 0
        self.first_x = x
        self.last_x =x
    def attemptToAddPoint(self, trialpoint, max_diameter, x):
        test_diameter = (self.points[0][0] - trialpoint[0])**2 + (self.points[0][2] - trialpoint[2])**2 
        #print(f'FP ({self.points[0][0]}, {self.points[0][2]}) | TP ({trialpoint[0]}, {trialpoint[2]})')
        #print(f'Testing Diameter: {test_diameter}')
        if test_diameter < max_diameter ** 2:
            self.points.append(trialpoint)
            self.diameter = max(test_diameter, self.diameter)
            self.last_x = x
            return True
        else:
            return False
    def computeCenter(self):
        return np.average(np.array(self.points), axis=0)


            
def nothing(x):
    pass

def organizeIntoCircles(arr, max_distance, max_size, min_point_amount, debug_img, pixel_border):
    
    completedCircles = []
    currentCircle = None
    for x in range(arr.shape[0]):
        if arr[x][2] < max_distance:
            if (currentCircle == None):
                currentCircle = ObstacleCircle(arr[x],x+pixel_border)
            else:
                #print(x)
                if not currentCircle.attemptToAddPoint(arr[x], max_size, x+pixel_border):
                    completedCircles.append(currentCircle)
                    currentCircle = ObstacleCircle(arr[x],x+pixel_border)
    if (currentCircle != None):
        completedCircles.append(currentCircle)
    finalCircleList = []
    for circle in completedCircles:
        if len(circle.points) > min_point_amount:
            finalCircleList.append(circle)
            pixel_radius = (circle.last_x+1 - circle.first_x)/2
            print(circle.first_x, circle.last_x, pixel_radius)
            jetson.utils.cudaDrawCircle(debug_img, (circle.first_x+pixel_radius, 200), pixel_radius, (255,255,0,200)) 
            

    print(len(finalCircleList))


width = 848
height = 480
img_output = jetson.utils.videoOutput("", argv = sys.argv)
output =  jetson.utils.videoOutput("", argv = sys.argv)
secondOutput = jetson.utils.videoOutput("", argv = sys.argv)
thirdOutput = jetson.utils.videoOutput("", argv = sys.argv)
fourthOutput = jetson.utils.videoOutput("", argv = sys.argv)
fifthOutput = jetson.utils.videoOutput("", argv = sys.argv)
converted_depth = jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
pointcloud_depth = jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
gradient_depth =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
first_filter_depth =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
average_filter_depth =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
vertical_banding_depth =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
actual_obstacles =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')
obstacles_converted =  jetson.utils.cudaImage(width=width, height=height, format='rgb32f')

dc = DepthCamera(width,height, 'd435')
intrinsics = dc.get_intrinsics()
print(intrinsics.fx)
print(intrinsics.fx * 50/195)
pc = pyrealsense2.pointcloud()
pastTime = time.time();

cv2.namedWindow('TrackBars')
cv2.moveWindow('TrackBars',1320,0)
cv2.createTrackbar("height_sample_size", 'TrackBars', 9, 20, nothing)
cv2.createTrackbar("height_threshold", "TrackBars", 45,1000, nothing) 
cv2.createTrackbar("depth_threshold", "TrackBars", 2000,5000, nothing) 
cv2.createTrackbar("first_filter_size", "TrackBars", 5,30, nothing) 
cv2.createTrackbar("first_filter_cutoff", "TrackBars", 26, 3600, nothing)
cv2.createTrackbar("filter_size", "TrackBars", 22,50, nothing) 
cv2.createTrackbar("filter_cutoff", "TrackBars", 200, 5000, nothing)
cv2.createTrackbar("vertical_pixel_cutoff", "TrackBars", 10, 20, nothing)
cv2.createTrackbar("vertical_filter_height", "TrackBars", 165, 1000, nothing)
cv2.createTrackbar("vertical_depth_cutoff", "TrackBars", 315, 1000, nothing)
cv2.createTrackbar("vertical_depth_top_min_dist", "TrackBars", 400, 2000, nothing)
cv2.createTrackbar("vertical_depth_top_slope", "TrackBars", 500, 1000, nothing)
cv2.createTrackbar("obstacle_size", "TrackBars", 200, 500, nothing)
cv2.createTrackbar("obstacle_pixel_min_size", "TrackBars", 10, 50, nothing)
prevTime = time.time()
while True:
    depth_frame, color_frame = dc.get_raw_frame()
    depth = np.asanyarray(depth_frame.get_data())
    cuda_depth = jetson.utils.cudaFromNumpy(depth)
    color = np.asanyarray(color_frame.get_data())
    cuda_color = jetson.utils.cudaFromNumpy(cv2.cvtColor(color,cv2.COLOR_BGR2RGB))
    jetson.utils.cudaConvertColor(cuda_depth, converted_depth)
    jetson.utils.cudaDeproject(converted_depth, pointcloud_depth, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

    height_sample    = cv2.getTrackbarPos("height_sample_size", 'TrackBars')
    height_threshold = cv2.getTrackbarPos("height_threshold", 'TrackBars')
    depth_threshold = cv2.getTrackbarPos("depth_threshold", 'TrackBars')
    first_filter_size = cv2.getTrackbarPos("first_filter_size", 'TrackBars')
    first_filter_cutoff = cv2.getTrackbarPos("first_filter_cutoff", 'TrackBars')
    filter_size = cv2.getTrackbarPos("filter_size", 'TrackBars')
    filter_cutoff = cv2.getTrackbarPos("filter_cutoff", 'TrackBars')
    vertical_pixel_cutoff = cv2.getTrackbarPos("vertical_pixel_cutoff", 'TrackBars')
    vertical_height_cutoff = cv2.getTrackbarPos("vertical_filter_height", 'TrackBars')
    vertical_depth_cutoff= cv2.getTrackbarPos("vertical_depth_cutoff", 'TrackBars')
    vertical_depth_top_min_dist  = cv2.getTrackbarPos("vertical_depth_top_min_dist", 'TrackBars')
    vertical_depth_top_slope= cv2.getTrackbarPos("vertical_depth_top_slope", 'TrackBars')/1000
    obstacle_size = cv2.getTrackbarPos("obstacle_size", 'TrackBars')
    obstacle_pixel_min_size = cv2.getTrackbarPos("obstacle_pixel_min_size", 'TrackBars')
    border = max(first_filter_size, height_sample, filter_size)
    jetson.utils.cudaMaskHeightChanges(pointcloud_depth, gradient_depth, height_sample, height_threshold, depth_threshold)
    jetson.utils.cudaCustomNoiseFilter(gradient_depth, first_filter_depth, first_filter_size, first_filter_cutoff, depth_threshold)
    jetson.utils.cudaSmallObstacleFilter(first_filter_depth, vertical_banding_depth, pointcloud_depth, vertical_pixel_cutoff, vertical_height_cutoff, vertical_depth_cutoff, vertical_depth_top_min_dist, vertical_depth_top_slope)
    
    jetson.utils.cudaCustomNoiseFilter(vertical_banding_depth, average_filter_depth, filter_size, filter_cutoff, depth_threshold)
    jetson.utils.cudaComputeActualObstacles(average_filter_depth, actual_obstacles, pointcloud_depth, border, depth_threshold)
    npObstacles = jetson.utils.cudaToNumpy(actual_obstacles)
    #organizeIntoCircles(npObstacles[border,border:-border,:], depth_threshold, obstacle_size, obstacle_pixel_min_size, average_filter_depth, border)
    #print (npObstacles[border,border:-border,:].shape)
    #print (npObstacles[border,border:-border,0])
    #jetson.utils.cudaRenderDepthBounds(actual_obstacles, 0, depth_threshold-1)

    #pc.map_to(color_frame)
    #points = pc.calculate(depth_frame)
    #vtx = np.asanyarray(points.get_vertices())
    #vtx = vtx.reshape((height,width))
    #depths = np.zeros((height,width))
    #
    #with np.nditer(depths, flags=['multi_index'], op_flags=['writeonly'])as it:
    #    for x in it:
    #        depths[it.multi_index[0], it.multi_index[1]] = vtx[it.multi_index[0], it.multi_index[1]][2]
    
    #depths *= 1000
    #depths = depths.astype('uint16')

    #depth = np.asanyarray(depth_frame.get_data())
    #barray = np.full((height,1), 1)
    #averaged = np.expand_dims(np.average(depth, axis=0), axis=0)
    
    #average_depth_over_y = np.matmul(barray, averaged).astype('uint16')


    jetson.utils.cudaRenderDepthBounds(converted_depth, 200, 10000)
    t = time.time() - prevTime
    prevTime = time.time()
    img_output.Render(cuda_color)
    output.Render(gradient_depth)
    output.SetStatus("Raw Height Change Mask")
    secondOutput.Render(average_filter_depth)
    secondOutput.SetStatus(f"Average Filter (Final Output) FPS: {1/t}")
    thirdOutput.Render(vertical_banding_depth)
    thirdOutput.SetStatus("Vertical Banding")
    fourthOutput.Render(first_filter_depth)
    fourthOutput.SetStatus("First Filter Applied")
    fifthOutput.Render(converted_depth)
    fifthOutput.SetStatus(f"Raw Depth")
    #sixthOutput.Render(converted_depth)
    
    
    cv2.waitKey(1)

dc.release()
