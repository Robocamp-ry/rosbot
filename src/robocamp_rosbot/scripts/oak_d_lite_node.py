#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='depthai_ros_driver_node',
            name='oakd_lite_node',
            output='screen',
            parameters=[{
                'camera_name': 'oakd_lite',
                'mx_id': '',
                'usb_speed': 'HIGH',
                'i2c_bus': 1,
                'i2c_address': 10,
                'sync_mode': 'ON_DEMAND',
                'usb_custom_reset': False,
                'depthai_board': 'OAK-D-LITE'
            }],
        ),
    ])
