
# Package: Piksi_gps
# Node: Piksi_gps_pub
# Node Name: gps
# Topic: /gps/fix

# Cspell:ignore Piksi, rclpy
import math
import argparse
import rclpy
import sensor_msgs.msg as sensor_msgs
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED

class GPSPublisher(Node):
    """ 
    A ROS node that publishes GPS, magnetometer and accelerometer data
    ...

    Parameters
    ----------      
    serial_path : String, default = '/dev/ttyUSB0'
        The path where the Piksi is mounted (teletype / usb locations only).
    Piksi_baud : int, default = 115200
        The baud rate in which the Piksi is communicating to the system, incompatible baud rates can lead to message corruption.
    

    Topics
    ------
    raw_data : L{std_msgs.Int8}
        Data representing some quantity like speed or force

    modified_data : L{std_msgs.Int8}
        Data representing some other quantity
        
    Publishes
    ---------
    gps : L{sensor_msgs.msg.NavSatFix}
        Coordinates from GPS
    mag : L{sensor_msgs.msg.MagneticField}
        Magnetic Field values (direction)
    mpu : L{sensor_msgs.msg.Imu}
        Acceleration and gyroscope values

    """
    def __init__(self):
        super().__init__('gps')
        # Serial Communication Parameters
        baud_rate = self.declare_parameter('Piksi_baud', 115200)
        usb_port = self.declare_parameter('serial_path', '/dev/ttyUSB0')
        parser = argparse.ArgumentParser(
        description="Swift Navigation SBP Example.")
        parser.add_argument(
            "-p",
            "--port",
            default=[usb_port.value],
            nargs=1,
            help="specify the serial port to use.")
        args = parser.parse_args()
        self.driver = PySerialDriver(args.port[0], baud=baud_rate)
        self.framer = Framer(self.driver.read, None, verbose=True)
        self.handler = Handler(self.framer)
        self.handler.add_callback(self.Piksi_log_callback)
        self.handler.start()
        # Creates respective publishers for gps, compass and accelerometer
        self.publisher_gps = self.create_publisher(NavSatFix, 'gps', 10)
        self.publisher_mag = self.create_publisher(MagneticField, 'mag', 10)
        self.publisher_mpu = self.create_publisher(Imu, 'mpu', 10)


    def piksi_log_callback(self, signal, *args, **kwargs):
        '''
        This function is called when the GPS sends data over the communication medium
        It filters the incoming message into the required data points and publishes them to their respective ros topics
        
        Arguments
        ----------
        signal: the incoming message object
        
        '''
        #Because we are reading raw serial inputs, we check for conversion errors
        try:
            # If the signal contains gps values
            if str(type(signal)) == "<class 'sbp.navigation.MsgPosLLH'>": #GPSsensor_msgs/MagneticField.msg
                gps_msg=NavSatFix()
                gps_msg.longitude = signal.lat
                gps_msg.latitude = signal.lon
                self.publisher_gps.publish(gps_msg) #needs covariance, etc etc
                
            # If the signal contains IMU / Accelerometer values
            if str(type(signal)) == "<class 'sbp.imu.MsgImuRaw'>": #Accelerometer / Gyro
                mpu_msg=Imu()
                mpu_msg.orientation = Quaternion(x=float(signal.gyr_x), y=float(signal.gyr_y), z=float(signal.gyr_z))
                metres_per_sec2_per_g = 9.8 # since 1 g is equal to 9.8m/s^2
                mpu_msg.linear_acceleration = Vector3(x=signal.acc_x*metres_per_sec2_per_g, y=signal.acc_y*metres_per_sec2_per_g, z=signal.acc_z*metres_per_sec2_per_g)
                
                self.publisher_mpu.publish(mpu_msg) #needs covariance, etc etc
            
            # If the signal contains Magnetometer values 
            if str(type(signal)) == "<class 'sbp.mag.MsgMagRaw'>": #magnetometer
                mag_msg=MagneticField()
                mag_msg.magnetic_field = Vector3(x=float(signal.mag_x), y=float(signal.mag_y), z=float(signal.mag_z))
                self.publisher_mag.publish(mag_msg)

        except ValueError:
            print("WARN: Error in parsing values from Piksi_GPS, possibly missing data")


def main(args=None):
    '''
    Initializes ROS node for GPS module and then spins it indefinitely.
    Arguments
    ----------
    any (passed into node object)
    '''
    rclpy.init(args=args)
    piksi_gps_pub = GPSPublisher()
    rclpy.spin(piksi_gps_pub)
    piksi_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()