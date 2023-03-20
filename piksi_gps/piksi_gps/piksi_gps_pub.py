import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
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
        self.publisher_ = self.create_publisher(sensor_msgs.NavSatFix, '/gps/fix', 10)
        #timer_period = 2  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # Serial Comms Parameters
        usbPort = self.declare_parameter('serial path', '/dev/ttyUSB1')
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
        self.handler.add_callback(self.piksi_log_callback, )
        self.handler.start()

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
                    if str(type(signal)) == "<class 'sbp.navigation.MsgPosLLH'>": #GPS
                        print("gps Fix identified")
                        
                        print(f"Latitude: {signal.lat} °")
                        print(f"Longitude: {signal.lon} °")
                        print(f"Satilite #: {signal.n_sats} °")
                except ValueError:
                    print("bruh moment (GPS data)")
            if True: #spam MPU updates?
                try:
                    if str(type(signal)) == "<class 'sbp.imu.MsgImuRaw'>": #Accelerometer / Gyro
                        print(f"Accel X: {signal.acc_x}   Accel Y{signal.acc_y}     Accel Z{signal.acc_z}")
                        print(f"Gyro  X: {signal.gyr_x}   Gyro  Y{signal.gyr_y}     Gyro  Z{signal.gyr_z}")
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