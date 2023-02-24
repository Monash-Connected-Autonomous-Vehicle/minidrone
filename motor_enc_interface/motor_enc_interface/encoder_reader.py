import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from math import pi as PI


class EncoderReader(Node):
    def __init__(self):
        # Publisher setup
        super().__init__('encoder_reader')
        self.publisher_ = self.create_publisher(String, 'encoder_data', 10)
        self.timer_period = 0.5  # seconds

        self.timer = self.create_timer(self.timer_period, self.publish_encoder_data)
        
        # GPIO setup
        MOTOR_ONE_PINS = [12, 16]  # For reference BOARD pins [12, 16] == BCM pins [18, 16]
        MOTOR_TWO_PINS = []  # TODO add after encoder 1 works as intended
        DEG_PER_INC = 0.18  # 360/2000 -> number of degrees per rotary encoder increment TODO we're using a new encoder now -> versaplanetary
        
        GPIO.setmode(GPIO.BOARD)  # refer to pins with BOARD scheme
        GPIO.setup(MOTOR_ONE_PINS, GPIO.IN)  # set pins as INPUT
        
        self.prev_dir_counter = 0  # for calculating speed TODO change var naming
        self.dir_counter = 0
        self.angular_pos = 0
        self.ang_speed = 0
        self.prev_pin_1 = GPIO.input(MOTOR_ONE_PINS[0])
        

    def publish_encoder_data(self):
        # TODO can try to calculate speed here
        # new position - old position/ timer period
        self.ang_speed = (self.direction_counter - self.prev_dir_counter)/self.timer_period
        
        msg = String()
        msg.data = "dir: " + str(direction_counter) + ", position: " + str(angular_pos * DEG_PER_INC)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
    def read_encoder_data(self):
        """
        Reads data from motor encoder to determine motor direction, position and speed
        """
        try:
            while True:  # TODO need to read from 2 encoders
                # reading current pin states            
                pin_1 = GPIO.input(MOTOR_ONE_PINS[0])
                pin_2 = GPIO.input(MOTOR_ONE_PINS[1])          
                
                # using pin states to determine direction of rotation (HI=True, LO=False)
                # further explanation @ shorturl.at/nuFO6
                if pin_1 is not prev_pin_1:  # TODO change direction indicator from incrementing var to binomial
                    if pin_2 is not pin_1:
                        self.dir_counter += 1
                    else:
                        self.dir_counter -= 1
                    
                    self.angular_pos = abs(direction_counter) % 2000
                
                self.prev_pin_1 = pin_1
              
        finally:
            print("cleaning up")
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    enc_data_pub = EncoderReader()
    rclpy.spin(enc_data_pub)

    # Destroy the node explicitly (optional - else garbge collecter does it)
    enc_data_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
