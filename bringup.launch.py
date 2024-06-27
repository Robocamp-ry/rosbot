# bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ldlidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_stl_ros2'),
        'launch',
        'ld19.launch.py'
    )

    oak_d_lite_launch_path = os.path.join(
        get_package_share_directory('robocamp_rosbot'),
        'scripts',
        'oak_d_lite.node.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_d_lite_launch_path)
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ld19',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],
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

