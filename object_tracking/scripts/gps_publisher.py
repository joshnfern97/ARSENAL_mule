#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from numpy import *
import serial




def talker():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    try:
        serial_port = serial.Serial(
        port="/dev/ttyUSB1",
        baudrate=115200,timeout = 1

        )
        # Wait a second to let the port initialize
        time.sleep(1)


        rospy.init_node('gps_rover', anonymous=True)

        process_rate = 100
        rate = rospy.Rate(process_rate)
        heading = 0
        gps_lat = 0
        gps_lng = 0
        
        heading_pub = rospy.Publisher('rover_heading',Float32, queue_size=10)
        lat_pub = rospy.Publisher('rover_latitude',Float32, queue_size=10)
        lng_pub = rospy.Publisher('rover_longitude',Float32, queue_size=10)

        
        while not rospy.is_shutdown():
            try:
                #print('reading')
                data = serial_port.readline().rstrip()
                array = data.decode('utf-8').split(',')
                heading = float(array[0])
                
                
                gps_lat =  float(array[1])
                gps_lng = float(array[2])
                
                
                heading_pub.publish(heading)
                lat_pub.publish(gps_lat)
                lng_pub.publish(gps_lng)
                rate.sleep()
                
            except KeyboardInterrupt:
          
                serial_port.close()
                pass

            except:
                pass
    except:
        #If nothing is connected it just passes
        pass

if __name__ == '__main__':
    talker()