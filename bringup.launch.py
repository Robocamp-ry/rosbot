# bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}], # Adjust this if needed
        ),
        Node(
            package='robocamp_rosbot',
            executable='robot_controller_node',
            name='robot_controller_node_cpp',
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
