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
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        gpsmsg = sensor_msgs.NavSatFix()
        # test = self.handler.
        # msg.latitude = test.
        gpsmsg.longitude = 22.0
        self.publisher_.publish(gpsmsg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):

    rclpy.init(args=args)
    parser = argparse.ArgumentParser(
    description="Swift Navigation SBP Example.")
    parser.add_argument(
        "-p",
        "--port",
        default=['/dev/ttyUSB0'], # sudo dmesg | grep tty (view connected devices, ensure match)
        nargs=1,
        help="specify the serial port to use.")
    argsSerial = parser.parse_args()
    with PySerialDriver(argsSerial.port[0], baud=115200) as driver:
        with Handler(Framer(driver.read, None, verbose=True)) as source:
            try:
                for msg, metadata in source.filter(0x020A):
                    # Print out the lat and long coordinates
                    print("%.4f,%.4f" % (msg.lat, msg.lon)
                                              )
            except KeyboardInterrupt:
                pass

    piksi_gps_pub = GPSPublisher()
    
    rclpy.spin(piksi_gps_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    piksi_gps_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()