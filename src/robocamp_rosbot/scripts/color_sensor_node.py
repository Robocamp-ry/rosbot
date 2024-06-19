#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from gpiozero import DigitalOutputDevice, DigitalInputDevice
import time

class ColorSensorNode(Node):
    def __init__(self):
        super().__init__('color_sensor_node')

        # Pin configuration
        self.S0 = DigitalOutputDevice(17)
        self.S1 = DigitalOutputDevice(27)
        self.S2 = DigitalOutputDevice(22)
        self.S3 = DigitalOutputDevice(23)
        self.OUT = DigitalInputDevice(24)
        self.OE = DigitalOutputDevice(25)
        self.LED = DigitalOutputDevice(18)  # LED control pin

        # Enable the sensor
        self.OE.off()

        # Set scaling to 20%
        self.S0.on()
        self.S1.off()

        # Turn on the LED
        self.LED.on()

        # Publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, 'color_sensor_data', 10)
        self.timer = self.create_timer(5.0, self.publish_color_data)

    def read_frequency(self):
        self.OUT.wait_for_active()
        start = time.time()
        self.OUT.wait_for_inactive()
        end = time.time()
        return 1 / (end - start)

    def set_color_filter(self, s2, s3):
        self.S2.value = s2
        self.S3.value = s3
        time.sleep(0.1)

    def publish_color_data(self):
        self.set_color_filter(0, 0)  # Red
        red = self.read_frequency()
        self.set_color_filter(1, 1)  # Green
        green = self.read_frequency()
        self.set_color_filter(0, 1)  # Blue
        blue = self.read_frequency()

        msg = Float32MultiArray()
        msg.data = [red, green, blue]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Red: {red}, Green: {green}, Blue: {blue}')

        if green<red and blue<red:
            self.get_logger().info('Red detected')
        elif red<green and blue<green:
            self.get_logger().info('Green detected')
        elif red<blue and green<blue:
            self.get_logger().info('Blue detected')
        else:
            self.get_logger().info('No color detected')

def main(args=None):
    rclpy.init(args=args)
    color_sensor_node = ColorSensorNode()
    rclpy.spin(color_sensor_node)
    color_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
