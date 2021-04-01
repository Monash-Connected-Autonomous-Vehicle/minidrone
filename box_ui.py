#!/usr/bin/env/python2.7

import rospy

from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout


def display_calibration_status(calibration_state):

    rospy.loginfo("----------------- CALIBRATION STATUS -----------------")

    if calibration_state["sys"] == 3:
        rospy.loginfo("IMU System Calibrated")
    else:
        rospy.loginfo("IMU System NOT Calibrated")

    if calibration_state["gyro"] < 3:
        rospy.loginfo("IMU Gyroscope NOT Calibrated")

    if calibration_state["acc"] < 3:
        rospy.loginfo("IMU Accelerometer NOT Calibrated")
    
    if calibration_state["mag"] < 3:
        rospy.loginfo("IMU Magnetometer NOT Calibrated")

    if calibration_state["temp"] > 30:
        rospy.loginfo("IMU Temperature is elevated. Please check...")

    rospy.loginfo("-----------------------------------------------------")

def calibration_callback(data):
    
    rospy.loginfo("----------------- CALIBRATION DATA -----------------")
    
    calibration_state = dict()
    calibration_data = list(map(ord, data.data)) # convert from hex 

    # save calibration data as dict
    for i, val in enumerate(calibration_data):
        
        label = str(data.layout.dim[i].label)
        calibration_state[label] = val

    # display raw calibration data
    rospy.loginfo(calibration_state)

    # display calibration status
    display_calibration_status(calibration_state)


if __name__ == "__main__":

    # initialise ros node
    rospy.init_node("mcav_box_ui")

    # setup subscriber
    rospy.Subscriber("/imu/calibration", UInt8MultiArray, calibration_callback)

    rospy.spin()

