#!/usr/bin/env python2.7
'''
MCAV - Box

BoxRosify is a class that receives serial data from the arduino
and republishes the raw information into an appropriate ROS message type.

CURRENT
- message type published is dictated by length of decoded array
- data frame start and end characterised by an alphabetical letter (see arduino code)
- works for IMU and GPS

TODO:
    - check scaling on imu sensor (acc/vel) values
'''

import os
import rospy
import serial

from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout

# scaling constants
MICRO_TESLA_TO_TESLA = 1e-6

class BoxRosify:
    """
        Convert sensor messages received on serial to ROS messages
            Supports:
                - imu: raw, mag, calibration
                - gps:
    """
    def __init__(self, port_name, baud_rate):

        if os.path.exists(port_name): # continues only if the correct port name is used
            rospy.loginfo("\n\n------Box ROS data Republisher Started!------\n")
            rospy.loginfo("\n\n------TO KILL NODE, CLOSE THIS TERMINAL------\n")
            self.port_name = port_name
            self.baud_rate = baud_rate # should match Arduino
            self.arduino = serial.Serial(self.port_name, self.baud_rate)

            self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=100)
            self.mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=100)
            self.calibration_pub = rospy.Publisher("/imu/calibration", UInt8MultiArray, queue_size=100)
            self.gps_pub = rospy.Publisher("/gps", NavSatFix, queue_size=100)

            self.num_gps_messages_required = 4  # minimum messages required for NavSatFix message
            self.num_imu_messages_required = 10 # minimum messages requred for Imu message
            self.num_mag_messages_required = 3  # minimum messages requred for MagneticField message
            self.num_calib_messages_required = 5  # minimum messages requred for UInt8MultiArray message



        else: rospy.logerr("\n\nERROR: Port does not exist, check USB connection to Arduino\n")

    def republish(self):
        """Republish serial sensor data as ROS message"""

        self.arduino.flushInput() # clear any exisiting overlapped data in input buffer

        while self.arduino.isOpen():

            arduino_data = self.arduino.readline()

            if arduino_data: # check serial data is not empty

                # decode data from serial frame
                decoded_frame, frame_type = self.decode_data(arduino_data)
                # print(decoded_frame, arduino_data)

                # publish frame as ros message
                self.publish_frame(decoded_frame, frame_type)

        rospy.logerr("\n\nERROR: Port exists but no data received, check if Arduino is sending information\n")

    def publish_frame(self, decoded_frame, frame_type):
        """publish frame as correct message type"""

        if len(decoded_frame) == self.num_gps_messages_required and frame_type == "gps":
            gps_msg = self.fill_gps_msg(decoded_frame)
            self.publish_message(self.gps_pub, gps_msg)

        if len(decoded_frame) == self.num_imu_messages_required and frame_type == "imu":
            imu_msg = self.fill_imu_msg(decoded_frame)
            self.publish_message(self.imu_pub, imu_msg)

        if len(decoded_frame) == self.num_mag_messages_required and frame_type == "mag":
            mag_msg = self.fill_mag_msg(decoded_frame)
            self.publish_message(self.mag_pub, mag_msg)
        
        if len(decoded_frame) == self.num_calib_messages_required and frame_type == "cal":
            calib_msg = self.fill_calibration_msg(decoded_frame)
            self.publish_message(self.calibration_pub, calib_msg)

    def decode_data(self, data):
        """ decode serial data packet into ros message components"""

        # decode serial frame
        decoded_frame = data.split(",")

        # header is frame type
        frame_type = decoded_frame[0]

        # remove header and tail characters
        decoded_frame = decoded_frame[1:-1]

        return decoded_frame, frame_type

    def fill_gps_msg(self, array):
        """fill ROS gps message"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.get_rostime()
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = float(array[0])
        gps_msg.longitude = float(array[1])
        gps_msg.altitude = float(array[2])
        
        return gps_msg

    def fill_imu_msg(self, array):
        """fill ROS imu message"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation.x = float(array[0])
        imu_msg.orientation.y = float(array[1])
        imu_msg.orientation.z = float(array[2])
        imu_msg.orientation.w = float(array[3])
        imu_msg.angular_velocity.x = float(array[4])
        imu_msg.angular_velocity.y = float(array[5])
        imu_msg.angular_velocity.z = float(array[6])
        imu_msg.linear_acceleration.x = float(array[7])
        imu_msg.linear_acceleration.y = float(array[8])
        imu_msg.linear_acceleration.z = float(array[9])
        
        return imu_msg
    
    def fill_mag_msg(self, array):
        """ fill ROS magnetic field message """
        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.get_rostime()
        mag_msg.header.frame_id = "imu"
        mag_msg.magnetic_field.x = float(array[0]) * MICRO_TESLA_TO_TESLA
        mag_msg.magnetic_field.y = float(array[1]) * MICRO_TESLA_TO_TESLA
        mag_msg.magnetic_field.z = float(array[2]) * MICRO_TESLA_TO_TESLA

        return mag_msg

    def fill_calibration_msg(self, array):
        """ fill ROS imu calibration message"""

        # calibration data
        calib_msg = UInt8MultiArray(data=map(int, array))
        
        # format of message
        calib_msg_labels = ["sys", "gyro", "acc", "mag", "temp"]
        calib_msg.layout = MultiArrayLayout()

        for msg_label in calib_msg_labels:

            arr_dim = MultiArrayDimension()
            arr_dim.label = msg_label
            calib_msg.layout.dim.append(arr_dim)

        return calib_msg

            
    def publish_message(self, publisher, msg):
        """ Generic wrapper for publishing ros messages"""

        publisher.publish(msg)
