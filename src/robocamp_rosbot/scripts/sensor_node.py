#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure, NavSatFix
import adafruit_bmp280
import board
import busio

class BMP280Node(Node):
    def __init__(self):
        super().__init__('bmp280_node')

        # Initialize I2C and BMP280
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

        # Create publishers for temperature, pressure, and altitude
        self.temp_publisher = self.create_publisher(Temperature, 'bmp280/temperature', 10)
        self.pressure_publisher = self.create_publisher(FluidPressure, 'bmp280/pressure', 10)
        self.altitude_publisher = self.create_publisher(NavSatFix, 'bmp280/altitude', 10)

        # Create a timer to publish the sensor data periodically
        self.timer = self.create_timer(5.0, self.publish_sensor_data)  # Publish every 5 seconds  

    def publish_sensor_data(self):
        # Create and publish temperature message
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = 'bmp280_frame'
        temp_msg.temperature = self.bmp280.temperature
        self.temp_publisher.publish(temp_msg)

        # Create and publish pressure message
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = self.get_clock().now().to_msg()
        pressure_msg.header.frame_id = 'bmp280_frame'
        pressure_msg.fluid_pressure = self.bmp280.pressure
        self.pressure_publisher.publish(pressure_msg)

        # Create and publish altitude message
        altitude_msg = NavSatFix()
        altitude_msg.header.stamp = self.get_clock().now().to_msg()
        altitude_msg.header.frame_id = 'bmp280_frame'
        altitude_msg.altitude = self.bmp280.altitude
        self.altitude_publisher.publish(altitude_msg)

        self.get_logger().info(f'Temperature: {temp_msg.temperature} C, Pressure: {pressure_msg.fluid_pressure} hPa, Altitude: {altitude_msg.altitude} m')

def main(args=None):
    rclpy.init(args=args)
    node = BMP280Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
