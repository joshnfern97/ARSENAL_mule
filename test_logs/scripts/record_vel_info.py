#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
import csv

import datetime
def handle_cmd_vel(msg, container):
    container["commanded_vel_linear"] = msg.linear.x
    container["commanded_vel_angular"] = msg.angular.z
def handle_odom(msg, container):
    container["actual_vel_linear"] = msg.twist.twist.linear.x
    container["actual_vel_angular"] = msg.twist.twist.angular.z

def run():
    rospy.init_node("vel_logger", anonymous=False)
    folder = '/home/rover/logs/vel_logger_' + datetime.datetime.now().strftime("%m-%d-%H_%M_%S")+'/'
    os.mkdir(folder)

    container = {
            "time" : 0,
            "commanded_vel_linear" : 0,
            "actual_vel_linear" : 0,
            "commanded_vel_angular" : 0,
            "actual_vel_angular" : 0,
    }
    rospy.Subscriber("/cmd_vel/managed", Twist, handle_cmd_vel, container)
    rospy.Subscriber("/odom", Odometry, handle_odom, container)
    rate = rospy.Rate(20)
    fieldnames = container.keys()
    startTime = time.time()
    with open (folder+'log.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames)
        writer.writeheader()
        while not rospy.is_shutdown():
            container["time"] = time.time()-startTime
            writer.writerow(container)
            rate.sleep()




if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
