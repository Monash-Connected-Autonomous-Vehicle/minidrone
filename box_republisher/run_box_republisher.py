#!/usr/bin/env python2.7
'''
MCAV - Box 

This file runs the Box's GPS and IMU ROS conversion Software  
Additional parameters can be specified, see below or use "-h" after typing in command to run to view. 
'''

import rospy
import argparse 
from BoxRosify import BoxRosify

if __name__ == "__main__":

    # defining optional command line arguements 
    parser = argparse.ArgumentParser(description='Box Sensor Data to ROS')
    parser.add_argument('-port_name', action='store',type=str, default='/dev/ttyACM0', dest='port', help='Define port name')
    parser.add_argument('-baud_rate', action='store',type=int, default=115200, dest='baud', help='Set receiver baud rate')

    args = parser.parse_args()

    # initalise ROS node
    rospy.init_node("mcav_box_sensor_publisher")

    box_rosify = BoxRosify(args.port, args.baud)
    box_rosify.republish() # start republishing process 