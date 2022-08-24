#!/usr/bin/env python3
# -*- utf-8 -*-
'''
VHF Functions

Set of functions to use vector field histograms for the rocket
'''
import math
import jetson.utils
import datetime

from object_tracking.msg import trajectory


def computeAngleAvailability(obstacles, robot_radius):
    '''
    Finds how far the robot could travel at every available angle

    Paramaters:
    obstacles (list): list of obstacles to avoid
    robot_radus (float): radius of the robot
    
    Returns
    angle_unit_vectors (list): unit vectors for every ray
    angle_distances (list): distance robot could travel in corresponding unit vector
    '''

    angle_unit_vectors = [(math.sin(math.radians(x)), math.cos(math.radians(x))) for x in range(-180,180)]
    angle_distances = [float('inf') for x in range(len(angle_unit_vectors))]
    i = 0
    for unit_vector in angle_unit_vectors:
        for obstacle in obstacles:
            squared_radius = (obstacle.radius+robot_radius)**2
            #distance to circle projected onto the ray
            d1 = unit_vector[0] * obstacle.center_x + unit_vector[1] * obstacle.center_z
            #squared distance from center of circle to ray
            d2 = obstacle.center_x * obstacle.center_x +obstacle.center_z * obstacle.center_z- d1*d1 
            if (d2 < squared_radius and d1 > 0):
                d3 = d1 - math.sqrt(squared_radius - d2)
                angle_distances[i] = max(1,min(d3, angle_distances[i]))
        #print (angle_unit_vectors[i], angle_distances[i])
        i += 1
    return angle_unit_vectors, angle_distances

def get_index_of_closest_choice(goal_angle, flip):
    '''
    Gets the index in the possible arrays closest to the goal angle
    
    Parameters:
    goal_angle (float): the angle in radians of the goal relative to forward
    flip (bool): flag set if goal is behind target

    Returns:
    index: the index of the ray closest to the target

    '''
    angle_of_goal_vector_int = 359 + int(math.degrees(goal_angle))
    if (not flip):
        angle_of_goal_vector_int -= 180
        
    index = angle_of_goal_vector_int
        
    return index % 360

def choose_location(goal_vector, goal_index, distances):
    '''
    Chooses the nearest free path to the target goal

    Paramaters:
    goal_vector(tuple(2)): the vector to the goal
    goal_index (int): index corresponding to the goal vector - angle_unit_vectors[goal_index] returns the direction of the goal
    distances (list): list of distances the robot can go for each available angle

    Returns:
    choice: the index chosen for the robot to travel in the corresponding direction
    '''
    #starts and goal index
    startIndex = goal_index
    
    #choice is set to a dummy value
    choice = -1
    #loops from 0 until whatever is bigger- startIndex or len(distances)-startIndex
    #this garuntees that this loop will eventually hit every ray
    for x in range(max(startIndex, len(distances)-startIndex)):
        #testA increases index, capping at max
        testA = min(len(distances)-1, startIndex + x)
        #testb decreases, capping at 0
        testB = max(0, startIndex - x)
        #for both testA and testB checks that if the robot where to travel in that direction
        #it would reach the destination before hitting an obstacle
        if distances[testA] ** 2 > goal_vector[0] ** 2 + goal_vector[1] ** 2:
            choice = testA
            break
        if distances[testB] ** 2 > goal_vector[0] ** 2 + goal_vector[1] ** 2:
            choice = testB
            break
    return choice

obstacle_render_img = jetson.utils.cudaImage(width=1280,height=720, format='rgb8')
font = jetson.utils.cudaFont()

def renderObstacleAndVectorMap(video_output, obstacles, 
        angle_unit_vectors, distances, max_depth, 
        goal_vec, chosen_index, goal_index):
    '''
    Renders the VHF top down visualization

    video_output(jetson.utils.videoOutput): video output to render image to
    obstacles(list): list of obstacles in front of robot
    angle_unit_vectors (list): unit vectors for every ray
    distances (list): distance robot could travel in corresponding unit vector
    max_depth(int): max depth robot looks ahead
    goal_vector(tuple(2)): the vector to the goal
    chosen_index (int): index of ray chosen to follow down
    goal_index (int): index corresponding to the goal vector

    '''
    jetson.utils.cudaDrawRect(obstacle_render_img, (0,0,1280,720), (0,0,0,255))
    origin_x = 640
    origin_y = 600
    scale = 600/max_depth

    for obstacle in obstacles:
        if (obstacle.radius != 0):
            jetson.utils.cudaDrawCircle(obstacle_render_img, (origin_x + obstacle.center_x * scale, origin_y- obstacle.center_z * scale), obstacle.radius * scale, (255,0,0,255))
    for i in range(len(angle_unit_vectors)):
        line_length = distances[i]
        if (line_length > max_depth):
            line_length = max_depth
        jetson.utils.cudaDrawLine(obstacle_render_img, (origin_x, origin_y), (origin_x + line_length * angle_unit_vectors[i][0]*scale,origin_y - line_length * angle_unit_vectors[i][1]*scale), (255,255,0,255),1)

    jetson.utils.cudaDrawLine(obstacle_render_img, (origin_x, origin_y), (origin_x + goal_vec[0]*scale,origin_y - goal_vec[1]*scale), (0,0,255,255),3)

    if (chosen_index != -1):
        line_length = distances[chosen_index]
        if (line_length > max_depth):
            line_length = max_depth
        jetson.utils.cudaDrawLine(obstacle_render_img, (origin_x, origin_y), (origin_x + line_length * angle_unit_vectors[chosen_index][0]*scale,origin_y - line_length * angle_unit_vectors[chosen_index][1]*scale), (0,255,0,255),3)
    if (goal_index != -1):
        line_length = distances[goal_index]
        if (line_length > max_depth):
            line_length = max_depth
        jetson.utils.cudaDrawLine(obstacle_render_img, (origin_x, origin_y), (origin_x + line_length * angle_unit_vectors[goal_index][0]*scale,origin_y - line_length * angle_unit_vectors[goal_index][1]*scale), (0,255,255,255),3)
        

    font.OverlayText(obstacle_render_img,1280, 720, datetime.datetime.now().strftime("Time: %H_%M_%S"), 25, 25, (255,255,255,255), (0,0,0,0))


    video_output.Render(obstacle_render_img)

        
def compute(goal_vec, target_angle, obstacles, robot_radius, max_depth, video_output):
    '''
    Main Compute Function

    Parameters:
    goal_vec (tuple(2)): x and z distance to goal
    target_angle (float): angle in radians from straight ahead
    obstacles (list): list of obstacles to avoid
    robot_radius (int): radius of robot, in mm
    max_depth (int): max_depth the robot is looking forwards at in mm
    video_output (jetson.utils.videoOutput) the video output object to render to
    
    Returns:
    traj (object_tracking.msg trajectory): trajectory for the robot to move towards
    '''
    traj = trajectory()
    backwards = False
    if (goal_vec[1] < 0):
        backwards = True
    vecs, distances = computeAngleAvailability(obstacles, robot_radius)
    goal_vec_mag = math.sqrt(goal_vec[0] ** 2 + goal_vec[1] ** 2)
    goal_index = get_index_of_closest_choice(target_angle, backwards)
    closest_index = choose_location(goal_vec, goal_index, distances)
    traj.angular_error = math.atan(vecs[closest_index][0]/vecs[closest_index][1])
    lin_error = vecs[closest_index][0] * goal_vec[0]+ vecs[closest_index][1] * goal_vec[1]
    if (backwards): 
        lin_error *= -1
    traj.linear_error = lin_error
    renderObstacleAndVectorMap(video_output, obstacles, vecs, distances, max_depth, goal_vec, closest_index, goal_index)
    return traj
    

        


