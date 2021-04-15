#!/usr/bin/env python3
'''
MCAV - MiniDrone 
Last editied: 16/04/21

Takes data from BNO055 IMU via I2C and republishes to ROS messages

'''
import board
import busio
import adafruit_bno055
import rospy
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout

#scaling constant for m-field
MICRO_TESLA_TO_TESLA = 1e-6

class ImuExtract():
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
    
    def get_quaternion(self):
        return self.sensor.quaternion
    
    def get_gyro(self):
        return self.sensor.gyro
    
    def get_linear_accel(self):
        return self.sensor.linear_acceleration
    
    def get_magnetic(self):
        return self.sensor.magnetic

    def get_calibration_data(self):
        return self.sensor.calibration_status
    
    def get_calibration_success(self):
        return self.sensor.calibrated
    
    # fill ROS Imu message
    def get_imu_msg(self):
        orientation = self.get_quaternion()
        angular_velocity = self.get_gyro()
        linear_acceleration = self.get_linear_accel()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation.w = orientation[0]
        imu_msg.orientation.x = orientation[1]
        imu_msg.orientation.y = orientation[2]
        imu_msg.orientation.z = orientation[3]
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        
        return imu_msg
    
    # fill ROS MagneticField message
    def get_mag_msg(self):
        magnetic_field = self.get_magnetic()

        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.get_rostime()
        mag_msg.header.frame_id = "imu"
        mag_msg.magnetic_field.x = magnetic_field[0]*MICRO_TESLA_TO_TESLA
        mag_msg.magnetic_field.y = magnetic_field[1]*MICRO_TESLA_TO_TESLA
        mag_msg.magnetic_field.z = magnetic_field[2]*MICRO_TESLA_TO_TESLA
        
        return mag_msg
    
    # fill ROS UInt8 array message with calibration data 
    def get_calib_msg(self):
        calib_msg = UInt8MultiArray(data=self.get_calibration_data())
        calib_msg_labels = ["sys","gyro","acc","mag"]

        for msg_label in calib_msg_labels:
            arr_dim = MultiArrayDimension()
            arr_dim.label = msg_label
            calib_msg.layout.dim.append(arr_dim)
        
        return calib_msg
