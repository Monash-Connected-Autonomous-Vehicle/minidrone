import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import SBP_MSG_BASELINE_NED
import sensor_msgs.msg as sensor_msgs
import argparse

# Package: piksi_gps
# Node: piksi_gps_pub
# Node Name: gps
# Topic: /gps/fix



class GPSPublisher(Node):
    """ A template node containing a publisher, subscriber, and a parameter. 
		Calculates the sum of an input and some fixed integer

    ...

    Parameters
    ----------
    serial_path : String, defualt = '/dev/ttyUSB0'
        The path where the piksi is mounted (teletype / usb locations only).
    piksi_baud : int, defualt = 115200
        The baud rate in which the piksi is communicating to the system, incompariable baud rates can lead to message corruption.
    

    Topics
    ------
    raw_data : L{std_msgs.Int8}
        Data representing some quantity like speed or force

    modified_data : L{std_msgs.Int8}
        Data representing some other quantity

    """
    def __init__(self):
        super().__init__('gps')
        # Serial Comms Parameters
        baudRate = self.declare_parameter('piksi_baud', 115200)
        usbPort = self.declare_parameter('serial_path', '/dev/ttyUSB0')
        parser = argparse.ArgumentParser(
        description="Swift Navigation SBP Example.")
        parser.add_argument(
            "-p",
            "--port",
            default=[usbPort.value],
            nargs=1,
            help="specify the serial port to use.")
        args = parser.parse_args()

        self.driver = PySerialDriver(args.port[0], baud=baudRate)  
        self.framer = Framer(self.driver.read, None, verbose=True)
        self.handler = Handler(self.framer)
        self.handler.add_callback(self.piksi_log_callback)
        self.handler.start()
        # Creates respective publishers for gps, compass and accelerometer
        self.publisher_gps = self.create_publisher(NavSatFix, 'gps', 10)
        self.publisher_mag = self.create_publisher(MagneticField, 'mag', 10) 
        self.publisher_mpu = self.create_publisher(Imu, 'mpu', 10) 


    def piksi_log_callback(self, signal, *args, **kwargs):
        #Because we are reading raw serial inputs, we check for conversion errors 
        try:
            # If the signal contains gps values
            if str(type(signal)) == "<class 'sbp.navigation.MsgPosLLH'>": #GPSsensor_msgs/MagneticField.msg
                gpsMsg=NavSatFix()
                gpsMsg.longitude = signal.lat
                gpsMsg.latitude = signal.lon
                self.publisher_gps.publish(gpsMsg) #needs covariance, etc etc
                
            # If the signal contains IMU / Accelerometer values 
            if str(type(signal)) == "<class 'sbp.imu.MsgImuRaw'>": #Accelerometer / Gyro
                mpuMsg=Imu()
                mpuMsg.orientation = Quaternion(x=float(signal.gyr_x), y=float(signal.gyr_y), z=float(signal.gyr_z))
                metres_per_sec2_per_g = 9.8 # since 1 g is equal to 9.8m/s^2
                mpuMsg.linear_acceleration = Vector3(x=signal.acc_x*metres_per_sec2_per_g, y=signal.acc_y*metres_per_sec2_per_g, z=signal.acc_z*metres_per_sec2_per_g)
                
                self.publisher_mpu.publish(mpuMsg) #needs covariance, etc etc
            
            # If the signal contains Magnometer values 
            if str(type(signal)) == "<class 'sbp.mag.MsgMagRaw'>": #magnometer
                #Get a heading from magnometer measures
                #NOT TESTED AS WORKING, needs to be tested and modified as needed
                
                #checks what side the angle is at (-180 to 180) then converts it into a continuious (0-360) range for output
                if ((180/math.pi)*math.atan2(signal.mag_y,signal.mag_x)) < 0:
                    heading = (360+(180/math.pi)*math.atan2(signal.mag_x,signal.mag_y))
                else:
                    heading = ((180/math.pi)*math.atan2(signal.mag_x,signal.mag_y))
                magMsg=MagneticField()
                magMsg.magnetic_field = Vector3(x=float(signal.mag_x), y=float(signal.mag_y), z=float(signal.mag_z))
                self.publisher_mag.publish(magMsg)

        except ValueError:
            print("WARN: Error in parsing values from Piksi_GPS, possibly missing data")


def main(args=None):

    rclpy.init(args=args)
    piksi_gps_pub = GPSPublisher()
    rclpy.spin(piksi_gps_pub)
    piksi_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()