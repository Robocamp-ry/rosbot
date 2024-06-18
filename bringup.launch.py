# bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocamp_rosbot',
            executable='robot_controller',
            name='robot_controller_cpp',
        ),
        Node(
            package='robocamp_rosbot',
            executable='camera_node',
            name='camera_node_cpp',
        ),
        Node(
            package='robocamp_rosbot',
            executable='lidar_node',
            name='lidar_node_cpp',
        ),
        Node(
            package='robocamp_rosbot',
            executable='motor_driver_node',
            name='motor_driver_node_cpp',
        ),
        Node(
            package='robocamp_rosbot',
            executable='sensor_node.py',
            name='sensor_node_py',
        ),
        Node(
            package='robocamp_rosbot',
            executable='color_sensor_node.py',
            name='color_sensor_node_py',
        ),
    ])
