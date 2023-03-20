import rclpy
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
        parser = argparse.ArgumentParser(
        description="Swift Navigation SBP Example.")
        parser.add_argument(
            "-p",
            "--port",
            default=['/dev/ttyUSB0'],
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
        spamMyConsoleForDebug = True
        

        if spamMyConsoleForDebug == True:
            self.get_logger().info(f'I heard: {((signal))}')
        
        
        #testing data catching
        try:
            if str(type(signal)) == "<class 'sbp.navigation.MsgPosLLH'>":
                print("gps Fix identified")
                
                print(f"Latitude: {signal.lat}")
                print(f"Longitude: {signal.lon}")



                
        except ValueError:
            print("bruh moment")
        








def main(args=None):

    rclpy.init(args=args)
    piksi_gps_pub = GPSPublisher()
    rclpy.spin(piksi_gps_pub)
    piksi_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()