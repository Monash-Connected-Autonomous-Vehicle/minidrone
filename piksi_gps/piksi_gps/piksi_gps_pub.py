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

    def __init__(self):
        super().__init__('gps')
        self.publisher_ = self.create_publisher(sensor_msgs.NavSatFix, 'sensor_msgs/MagneticField/gps/fix', 10)
        #timer_period = 2  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # Serial Comms Parameters
        usbPort = self.declare_parameter('serial_path', '/dev/ttyUSB0')
        parser = argparse.ArgumentParser(
        description="Swift Navigation SBP Example.")
        parser.add_argument(
            "-p",
            "--port",
            default=[usbPort.value],
            #default=['/dev/ttyUSB0'],
            nargs=1,
            help="specify the serial port to use.")
        args = parser.parse_args()

        self.driver = PySerialDriver(args.port[0], baud=115200)  # TODO: add baudrate as parameter
        self.framer = Framer(self.driver.read, None, verbose=True)
        self.handler = Handler(self.framer)
        self.handler.add_callback(self.piksi_log_callback)
        self.handler.start()
        self.publisher_GPS = self.create_publisher(NavSatFix, 'gps', 10)
        self.publisher_MAG = self.create_publisher(MagneticField, 'mag', 10) 
        self.publisher_MPU = self.create_publisher(Imu, 'mpu', 10) 


    def piksi_log_callback(self, signal, *args, **kwargs):
        #Jai Notes
        #MsgImuRaw -> IMU measurements (Gyro)
        #MsgPosLLH -> lat lon
        
        
        #gpsmsg = sensor_msgs.NavSatFix()

        # Publish position read from GPS
        #gpsmsg.latitude = item[0].lat
        #gpsmsg.longitude = item[0].lon
        #self.publisher_.publish(gpsmsg)
        #print('hey', type(msg))

        #self.get_logger().info(f'lon test: {(signal.MsgPosLLH.lon)}')


        #Set to true to see all incoming types
        spamMyConsoleForDebug = False
        

        if spamMyConsoleForDebug ==True:
            self.get_logger().info(f'Data below belongs too: {(type(signal))}')
            self.get_logger().info(f'I heard: {((signal))}')
            print("\n")
        
        if True:   #Spit out sesnor values?

            #testing data catching, 
            #checking type so we dont get spammed by repeat values every time a new datapoint arrives
            
            if True: #spam GPS updates?
                try:
                    if str(type(signal)) == "<class 'sbp.navigation.MsgPosLLH'>": #GPSsensor_msgs/MagneticField.msg
                        print("gps Fix identified")
                        
                        print(f"Latitude: {signal.lat}°")
                        print(f"Longitude: {signal.lon}°")
                        print(f"Satilite #: {signal.n_sats}")
                        gpsMsg=NavSatFix()
                        gpsMsg.longitude = signal.lat
                        gpsMsg.latitude = signal.lon
                        self.publisher_GPS.publish(gpsMsg) #needs covariance, etc etc



                except ValueError:
                    print("bruh moment (GPS data)")
            if True: #spam MPU updates?
                try:
                    if str(type(signal)) == "<class 'sbp.imu.MsgImuRaw'>": #Accelerometer / Gyro
                        print(f"Accel X: {signal.acc_x}   Accel Y{signal.acc_y}     Accel Z{signal.acc_z}")
                        print(f"Gyro  X: {signal.gyr_x}   Gyro  Y{signal.gyr_y}     Gyro  Z{signal.gyr_z}")
                        mpuMsg=Imu()

               
                        mpuMsg.orientation = Quaternion(x=float(signal.gyr_x), y=float(signal.gyr_y), z=float(signal.gyr_z))
                        metres_per_sec2_per_g = 9.8 # since 1 g is equal to 9.8m/s^2
                        mpuMsg.linear_acceleration = Vector3(x=signal.acc_x*metres_per_sec2_per_g, y=signal.acc_y*metres_per_sec2_per_g, z=signal.acc_z*metres_per_sec2_per_g)
                        
                        self.publisher_MPU.publish(mpuMsg) #needs covariance, etc etc

                except ValueError:
                    print("bruh moment (Mpu data)")
            if True: #spam MAG updates?                
                try:
                    if str(type(signal)) == "<class 'sbp.mag.MsgMagRaw'>": #magnometer
                        print(f"Mag X: {signal.mag_x}μT     Mag Y: {signal.mag_y}μT    Mag Z: {signal.mag_z}μT      Tow_f: {signal.tow_f}")
                        #Get a heading from magnometer measures
                        #NOT TESTED AS WORKING, needs to be tested and modified as needed
                        if ((180/math.pi)*math.atan2(signal.mag_y,signal.mag_x)) < 0:
                            heading = (360+(180/math.pi)*math.atan2(signal.mag_x,signal.mag_y))
                        else:
                            heading = ((180/math.pi)*math.atan2(signal.mag_x,signal.mag_y))

                        print(heading)
                        magMsg=MagneticField()
                        magMsg.magnetic_field = Vector3(x=float(signal.mag_x), y=float(signal.mag_y), z=float(signal.mag_z))
                        self.publisher_MAG.publish(magMsg)
                except ValueError:
                    print("bruh moment (Mag data)")



def main(args=None):

    rclpy.init(args=args)
    piksi_gps_pub = GPSPublisher()
    rclpy.spin(piksi_gps_pub)
    piksi_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()