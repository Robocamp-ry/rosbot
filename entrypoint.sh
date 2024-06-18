#!/bin/bash
set -e

# source the ROS 2 installation
source "/opt/ros/humble/setup.bash"
source "/ros2_ws/install/setup.bash"
exec "$@"
