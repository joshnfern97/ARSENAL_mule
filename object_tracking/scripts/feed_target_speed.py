#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from object_tracking.msg import trajectory
from geometry_msgs.msg import Twist
from ds4_driver.msg import Status
from nav_msgs.msg import Odometry
from modules.PID import PID
import time
from std_msgs.msg import Float32


running = False


def callback(data,speed):
    speed[0] = data.data


def run():
    '''
    Main function controlling node
    '''
    global running
    process_rate = 100
    #initializes ros stuff
    rospy.init_node("feed_velocity", anonymous=False)
    rate = rospy.Rate(process_rate)
    speed = [0]
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size =process_rate)

    prev_time = time.time()
 
    #initialize subscriber
    rospy.Subscriber("/filtered_speed", Float32, callback,speed) 

   
    vel = Twist()
    prev_time = time.time()


    while not rospy.is_shutdown():
        #print(speed[0])
        vel.linear.x = speed[0]
        pub.publish(vel)
        rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
