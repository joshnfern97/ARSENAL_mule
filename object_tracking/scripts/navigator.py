#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigator Node

Controls PID loops to control robot wheel motors

Subscribes To:
    /status: the status of the PS4 controller
    /trajectory: the destination to head towards
    /odom/ekf/enk_imu: the filtered odometry info of the robot
Publishes:
    /cmd_vel: motor commands to be interpreted by the motor driver
        coordinate system:
            x: +forward -backward
            y: +right - left
            z: +up -down
    
"""
import rospy
from object_tracking.msg import trajectory
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status
from nav_msgs.msg import Odometry
from modules.PID import PID
import time


running = False




def process_error(angular_error, linear_error, 
        angularPID, angular_deadband, max_angular, 
        linearPID, linear_deadband, max_linear, 
        dt, d_vel_err):
    '''
    Feeds the errors into PID loops

    Parameters:
    angular_error (float): how far angularly robot is from desired
    linear_error (float): how far linear robot is from desired
    angularPID (PID): pid controller for angular velocity
    angular_deadband (float): max allowable linear before correcting
    max_angular (float): cap on angular command
    linearPID (PID): pid controller for linear velocity
    linear_deadband (float): max allowable linear error before correcting
    max_linear (float): cap on linear command
    dt (float): time since last called
    d_vel_err (float): optional other estimate for velocity (potentially make use of better velocity estimation and feed it here for feedback control)

    Returns:
    vel (geometry_msgs.msg Twist)

    '''
    vel = Twist()
    #checks if angular error outside deadband
    if(abs(-angular_error) > angular_deadband):
        #if so compute the error using the pid and clamping it below max
        vel.angular.z = min(max_angular, max(-max_angular, angularPID.compute(-angular_error,dt)))
    else:
        angularPID.feed_sample(-angular_error, dt)
    #checks if linear error outside deadband
    if(abs(linear_error) > linear_deadband):
        #if so compute the error using the pid and clamping it below max
        vel.linear.x = min(max_linear, max(-max_linear, linearPID.compute(linear_error,dt, d_vel_err)))
    else:
        linearPID.feed_sample(linear_error, dt)
    return vel

def handle_trajectory(trajectory, container):
    '''
    Callback for reading trajectory info

    Parameters:
    trajectory (object_tracking.msg trajectory): current desired relative location of robot
    container (Dictionary): container to store info in

    '''
    container["angular_error"] = trajectory.angular_error
    container["linear_error"] = trajectory.linear_error
#checks controller state
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
def handle_odometry(msg, container):
    '''
    Callback for handlng odometry info

    Parameters:
    msg (sensor_msgs.msg Odometry): odemetry msg from the robot
    container (dictionary): container to store data in

    '''
    container["robot_linear_vel"] = msg.twist.twist.linear.x


def run():
    '''
    Main function controlling node
    '''
    global running

    #grabs parameters
    process_rate = rospy.get_param("/process_rate")
    running = rospy.get_param("/start_running")
    angular_p_gain  = rospy.get_param("/angular_p_gain")
    angular_d_gain  = rospy.get_param("/angular_d_gain")
    angular_deadband= rospy.get_param("/angular_deadband") 
    max_angular = rospy.get_param("/max_angular")

    linear_p_gain   = rospy.get_param("/linear_p_gain")
    linear_d_gain   = rospy.get_param("/linear_d_gain")
    linear_deadband = rospy.get_param("/linear_deadband")
    const_vel = rospy.get_param("/const_vel")
    feed_forward_k = rospy.get_param("/feed_forward_k")
    max_linear = rospy.get_param("/max_linear")

    #creates pid controllers
    linearPID = PID(linear_deadband, linear_p_gain, 0, linear_d_gain, 0)
    angularPID = PID(angular_deadband, angular_p_gain, 0, angular_d_gain, 0)

    #initializes ros stuff
    rospy.init_node("navigator", anonymous=False)
    rate = rospy.Rate(process_rate)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size =process_rate)

    prev_time = time.time()
    #container for storing callback info
    container = {
        "angular_error": 0,
        "linear_error": 0,
        "robot_linear_vel":0,
        
    }
    #initialize subscriber
    rospy.Subscriber("/trajectory", trajectory, handle_trajectory,
                (container)
            ) 

    rospy.Subscriber("/status", Status, handle_controller)
    
    rospy.Subscriber("/odom/ekf/enc_imu", Odometry, handle_odometry, container)

    prev_time = time.time()


    while not rospy.is_shutdown():
        curr_time = time.time()
        #initilize d_vel_error to be none
        d_vel_error = None
        #checks if a const vel (exterior vel element is specified)
        if (const_vel != ""):
            #if so d_vel_error is configured to be the combination of the robots velocity and the velocity of the person (converted to mm because that is what the pid expects
            d_vel_error = 1000 * (container['robot_linear_vel'] + const_vel)
        #run all the info through the process error command
        vel = process_error(container['angular_error'], container['linear_error'], angularPID, angular_deadband, max_angular, linearPID, linear_deadband, max_linear, curr_time-prev_time, d_vel_error)
        #optionally apply the feed forward part if const vel exists (you can set feed_forward_k to zero to stop this from having an effect)
        if (const_vel != ""):
            vel.linear.x += feed_forward_k * const_vel

        #if running publish
        if (running):
            pub.publish(vel)
        prev_time = curr_time
        rate.sleep()
    

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
