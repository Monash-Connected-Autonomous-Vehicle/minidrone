#!/usr/bin/env python2.7

import pytest
import rospy

from BoxRosify import BoxRosify


# todo: gps
# todo: roscore needs to be running?

# imu
imu_data = 'imu,-0.0402,0.0076,0.0000,0.9991,0.0625,-0.1250,-0.2500,0.0000,0.0300,-0.2500,end\r\n' 
decoded_imu_data = ['-0.0402', '0.0076', '0.0000', '0.9991', '0.0625', '-0.1250', '-0.2500', '0.0000', '0.0300', '-0.2500']
decoded_imu_frame_type = 'imu'

# mag
mag_data = 'mag,31.37,-2.88,31.37,end\r\n'
decoded_mag_data = ['31.37', '-2.88', '31.37']
decoded_mag_frame_type = "mag" 

# calib
calib_data = "cal,3,3,3,3,28,end"
decoded_calib_data = ["3", "3", "3", "3","28"]
decoded_calib_frame_type = "cal"

# gps
gps_data = "gps,-37.75693,145.123456,64.234,9,end"
decoded_gps_data = ["-37.75693", "145.123456", "64.234", "9"]
decoded_gps_frame_type = "gps"


def initialise_node(port_name='/dev/ttyACM0', baud_rate=115200):
    
    # initialise the ros node
    rospy.init_node("sensor_Data_Republisher")

    # initialise class
    box_rosify = BoxRosify(port_name=port_name, baud_rate=baud_rate)

    return box_rosify  


def test_should_decode_imu_data():

    box_rosify = initialise_node()

    decoded_frame, frame_type = box_rosify.decode_data(data=imu_data)

    assert decoded_frame == decoded_imu_data
    assert frame_type == decoded_imu_frame_type

def test_should_decode_mag_data():

    box_rosify = initialise_node()

    decoded_frame, frame_type = box_rosify.decode_data(data=mag_data)

    assert decoded_frame == decoded_mag_data
    assert frame_type == decoded_mag_frame_type


def test_should_decode_calib_data():

    box_rosify = initialise_node()

    decoded_frame, frame_type = box_rosify.decode_data(data=calib_data)

    assert decoded_frame == decoded_calib_data
    assert frame_type == decoded_calib_frame_type


def test_should_decode_gps_data():
    box_rosify = initialise_node()

    decoded_frame, frame_type = box_rosify.decode_data(data=gps_data)

    assert decoded_frame == decoded_gps_data
    assert frame_type == decoded_gps_frame_type