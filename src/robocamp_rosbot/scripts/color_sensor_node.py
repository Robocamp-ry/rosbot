import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
import RPi.GPIO as GPIO
import time

# GPIO pin setup for TCS3200
S0 = 23
S1 = 24
S2 = 27
S3 = 22
OUT = 17

# Frequency scaling settings
GPIO.setmode(GPIO.BCM)
GPIO.setup(S0, GPIO.OUT)
GPIO.setup(S1, GPIO.OUT)
GPIO.setup(S2, GPIO.OUT)
GPIO.setup(S3, GPIO.OUT)
GPIO.setup(OUT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(S0, GPIO.HIGH)
GPIO.output(S1, GPIO.LOW)

def read_color():
    # Select red filter
    GPIO.output(S2, GPIO.LOW)
    GPIO.output(S3, GPIO.LOW)
    time.sleep(0.1)
    red = GPIO.input(OUT)

    # Select green filter
    GPIO.output(S2, GPIO.HIGH)
    GPIO.output(S3, GPIO.HIGH)
    time.sleep(0.1)
    green = GPIO.input(OUT)

    # Select blue filter
    GPIO.output(S2, GPIO.LOW)
    GPIO.output(S3, GPIO.HIGH)
    time.sleep(0.1)
    blue = GPIO.input(OUT)

    return red, green, blue

class ColorSensorNode(Node):
    def __init__(self):
        super().__init__('color_sensor_node')
        self.publisher_ = self.create_publisher(ColorRGBA, 'color_sensor/data', 10)
        self.timer = self.create_timer(5.0, self.publish_color_data) # Publish every 5 seconds

    def publish_color_data(self):
        red, green, blue = read_color()
        msg = ColorRGBA()
        msg.r = red
        msg.g = green
        msg.b = blue
        msg.a = 1.0  # Assuming full opacity for simplicity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing color data: R={red}, G={green}, B={blue}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
