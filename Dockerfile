# Dockerfile
FROM ros:humble-ros-base

# Install required packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-vcstool \
    git \
    curl \
    wget \
    nano \
    build-essential \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libglu1-mesa-dev \
    mesa-utils \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-joy \
    libopencv-dev \
    libgpiod-dev \
    python3-libgpiod \
    && rm -rf /var/lib/apt/lists/*

# Install required Python packages
RUN pip3 install \
    opencv-python-headless \
    adafruit-circuitpython-bmp280 \
    adafruit-blinka \
    board \
    RPi.GPIO \
    gpiozero \
    lgpio \
    pigpio \
    ds4drv

# Install DS4 Driver and Lidar packages
COPY . /ros2_ws
WORKDIR /ros2_ws
RUN cd /ros2_ws/src && \
    git clone https://github.com/naoki-mizuno/ds4drv --branch devel && \
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

# Reload udev rules
COPY . /ros2_ws
WORKDIR /ros2_ws
COPY 50-ds4drv.rules /etc/udev/rules.d/
COPY start.sh /opt/

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Copy launch file to ROS2 workspace
COPY bringup.launch.py /ros2_ws/install/robocamp_rosbot/share/robocamp_rosbot/bringup.launch.py

# Add ROS2 source to .bashrc
RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    sudo echo ". /install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
# We are running our entrypoint commands through the start.sh script
CMD ["bash", "/opt/start.sh"]
