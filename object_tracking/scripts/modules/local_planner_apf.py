#!/usr/bin/env python3
# -*- utf-8 -*-
'''
APF functions

Set of functions used to perform Artificial Potential Field Based Pathing for the rocket
'''
import math
import jetson.utils
import datetime

from object_tracking.msg import trajectory


old_total_f_repl_x = 0
old_total_f_repl_z = 0
#comptutes the resulting force vector
def computeForceViaAPF(goal_vec, obstacles, robot_radius, influence_radius, k_attr, k_repl, gamma, max_attr, max_repl, persistence_percent):
    '''
    Uses artifical potential fields to compute the force

    goal_vec (tuple(2)): x and z distance to goal
    target_angle (float): angle in radians from straight ahead
    obstacles (list): list of obstacles to avoid
    robot_radius (int): radius of robot, in mm
    influence_radius (int): max distance from obstacle that they affect
    k_attr (float): coefficient for attractive force
    k_repl (float): coefficient for repellent force
    gamma (float): shape of repellent force curve
    persistence_percent(float): percent of past repelling force to average with new

    Returns:
    resultant (tuple(2)): the resultant xz force
    repl_vec_list (list): list of individual forces for each obstacle

    '''
    global old_total_f_repl_x, old_total_f_repl_z
    #attractive force is capped at max to prevent super far away target from overpowering obstacles
    f_attraction_x = k_attr * goal_vec[0]
    f_attraction_z = k_attr * goal_vec[1]
    f_attraction_mag = math.sqrt(f_attraction_x**2+ f_attraction_x**2)
    attraction_division = max(1, f_attraction_mag/max_attr)
    f_attraction_x /= attraction_division
    f_attraction_z /= attraction_division

    #holds the sum of repulsive (obstacle) forces
    new_total_f_repl_x = 0
    new_total_f_repl_z = 0
    repl_vec_list = [[0,0] for x in range(len(obstacles))]

    i = 0
    for obstacle in obstacles:
        
        #compute the magnitude of obstacle difference
        obstacle_distance = math.sqrt(obstacle.center_x ** 2 + obstacle.center_z ** 2)
        #distance to the outside of the obstacle
        #cap min to 1mm so no divide by zero
        ni = max(obstacle_distance - obstacle.radius - robot_radius,1)/1000
        #only add repelling force if distance to hitting the obstacle is less than the influence radius
        if ni <= influence_radius:
            #repulsive force
            f_repl = max(-max_repl, min(max_repl, -k_repl /(influence_radius/1000) * ((1/ni-1/(influence_radius/1000))**gamma)))

            #compute the components 
            repl_x = f_repl * obstacle.center_x / max(1, obstacle_distance)
            repl_z = f_repl * max(1, obstacle.center_z-robot_radius) /max(1, obstacle_distance)
            repl_vec_list[i][0] = repl_x
            repl_vec_list[i][1] = repl_z
            new_total_f_repl_x += repl_x
            new_total_f_repl_z += repl_z
        i += 1
    
    total_f_repl_x = new_total_f_repl_x * (1-persistence_percent) + old_total_f_repl_x * persistence_percent
    total_f_repl_z = new_total_f_repl_z * (1-persistence_percent) + old_total_f_repl_z * persistence_percent
    old_total_f_repl_x = total_f_repl_x
    old_total_f_repl_z = total_f_repl_z
    sum_force_x = f_attraction_x + total_f_repl_x
    sum_force_z = f_attraction_z + total_f_repl_z
    sum_force_mag = math.sqrt(sum_force_x**2 + sum_force_z**2)
    #basically used to see if the toal force is higher than max value, than scale it proportionally so that the magnitude is that max value
    force_division = max(1, sum_force_mag/max_attr)
    
    return (sum_force_x/force_division, sum_force_z/force_division), repl_vec_list

#img used for rendering
obstacle_render_img = jetson.utils.cudaImage(width=1280,height=720, format='rgb8')
#font used for rendering
font = jetson.utils.cudaFont()
def renderObstacleAndForceMap(video_output, obstacles, goal_vec, force_vec, max_depth, repl_vecs, robot_radius):
    '''
    Renders a top-down visualization of the apf

    Parameters:
    video_output (jetson.utils.videoOutput) the video output object to render to
    obstacles (list): list of obstacles to avoid
    goal_vec (tuple(2)): x and z distance to goal
    force_vec (tuple(2)): the output force vec ofthe apf
    max_depth (int): max_depth the robot is looking forwards at in mm
    repl_vecs (list): list of repelling forces for each obstacle
    robot_radius (int): radius of robot, in mm
    
    '''
    #center, where the robot "is"
    origin_x = 640
    origin_y = 540
    #scale to make sure all the obstacles fit on the screen
    scale = 600/(max_depth*1.5)
    #draw robot circle
    jetson.utils.cudaDrawCircle(obstacle_render_img, (origin_x, origin_y), robot_radius * scale, (125,125,125,255))

    i = 0
    for obstacle in obstacles:
        obs_x = origin_x + obstacle.center_x * scale
        obs_y = origin_y - obstacle.center_z * scale
        #draw the obstacle circe
        jetson.utils.cudaDrawCircle(obstacle_render_img, 
                (obs_x, obs_y), 
                max(1,obstacle.radius * scale), 
                (255,0,0,255))
        print(repl_vecs[i])
        jetson.utils.cudaDrawLine(obstacle_render_img,
                (obs_x, obs_y),
                (obs_x + repl_vecs[i][0] * scale, obs_y - repl_vecs[i][1] * scale),
                (200,50,50,255))
        i +=1

    #draw the line to the goal vector
    jetson.utils.cudaDrawLine(obstacle_render_img, 
            (origin_x, origin_y), 
            (origin_x + goal_vec[0]*scale,origin_y - goal_vec[1]*scale), 
            (0,255,0,255),3)

    #draw the chosen force vector
    jetson.utils.cudaDrawLine(obstacle_render_img, 
            (origin_x, origin_y), 
            (origin_x + force_vec[0] * scale, origin_y - force_vec[1] * scale), 
            (255,255,0,255),3)

    #overlay current time onto image
    font.OverlayText(obstacle_render_img, 1280, 720, 
            datetime.datetime.now().strftime("Time: %H_%M_%S"), 
            25, 25, (255,255,255,255), (0,0,0,0))
    #render image
    video_output.Render(obstacle_render_img)
    #wipe the screen for next frame
    jetson.utils.cudaDrawRect(obstacle_render_img, (0,0,1280,720), (0,0,0,255))
    
        
def compute(goal_vec, target_angle, obstacles, robot_radius, max_depth, video_output, influence_radius, k_attr, k_repl, max_attr, max_repl, gamma, persistence_percent):
    '''
    Main Compute Function

    Parameters:
    goal_vec (tuple(2)): x and z distance to goal
    target_angle (float): angle in radians from straight ahead
    obstacles (list): list of obstacles to avoid
    robot_radius (int): radius of robot, in mm
    max_depth (int): max_depth the robot is looking forwards at in mm
    video_output (jetson.utils.videoOutput) the video output object to render to
    influence_radius (int): max distance from obstacle that they affect
    k_attr (float): coefficient for attractive force
    k_repl (float): coefficient for repellent force
    gamma (float): shape of repellent force curve
    persistence_percent(float): percent of past repelling force to average with new
    
    Returns:
    traj (object_tracking.msg trajectory): trajectory for the robot to move towards
    '''
    
    traj = trajectory()
        

    #compure the force vector
    force_vec, repl_vecs =  computeForceViaAPF(goal_vec, 
            obstacles, robot_radius, 
            influence_radius, k_attr, k_repl, gamma, max_attr, max_repl, persistence_percent)
    #render to image
    renderObstacleAndForceMap(video_output, obstacles, goal_vec, force_vec, max_depth, repl_vecs, robot_radius)
    z_component = force_vec[1]
    #add a small amount if 0 to prevent divide by zero error
    if (z_component == 0):
        z_component += .001
    traj.angular_error = math.atan(force_vec[0]/z_component)

    #linear error is magnitude because we still want to move even if needing to drive perpendicular to where the robot is, maybe that is not quite right I dont know.
    traj.linear_error = math.sqrt(force_vec[0]**2 + z_component ** 2) * z_component/abs(z_component)
    return traj


