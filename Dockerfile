# Dockerfile
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-vcstool \
    git \
    cmake \
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
    ros-humble-vision-msgs \
    ros-humble-camera-info-manager \
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
    ds4drv \
    depthai

# Reload udev rules
COPY . /ros2_ws
WORKDIR /ros2_ws
COPY 50-ds4drv.rules /etc/udev/rules.d/
COPY start.sh /opt/

# Clone depthai-core
RUN cd /ros2_ws/src && \
    git clone --branch main --depth 1 https://github.com/luxonis/depthai-core.git /depthai-core && \
    cd /depthai-core && \
    git submodule update --init --recursive && \
    mkdir -p build && \
    cd build && \
    cmake -DCMAKE_CXX_FLAGS="-fPIC" .. && \
    make -j$(nproc) && \
    make install

# Ensure the installation directory exists
RUN mkdir -p /ros2_ws/install/depthai/lib

# Copy the depthai-core library to the install directory
RUN cp -r /depthai-core/build/libdepthai-core* /ros2_ws/install/depthai/lib

ENV CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"

# Clone the remaining repositories
RUN cd /ros2_ws/src && \
    git clone --depth 1 https://github.com/naoki-mizuno/ds4drv --branch devel && \
    git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git && \
    git clone --depth 1 https://github.com/luxonis/depthai-ros.git
    
# Install dependencies with rosdep
RUN apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Create necessary directory structure
RUN mkdir -p /ros2_ws/build/ds4drv

# Create an empty README.rst file
RUN touch /ros2_ws/build/ds4drv/README.rst

# Create an empty HISTORY.rst file
RUN touch /ros2_ws/build/ds4drv/HISTORY.rst

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Copy launch file to ROS2 workspace
COPY bringup.launch.py /ros2_ws/install/robocamp_rosbot/share/robocamp_rosbot/bringup.launch.py

# Add ROS2 source to .bashrc
RUN sudo echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    sudo echo ". /install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

# We are running our entrypoint commands through the start.sh script
CMD ["bash", "/opt/start.sh"]
