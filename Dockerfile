# Use ROS2 Humble as base
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    usbutils \
    wget \
    git \
    libusb-1.0-0 \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-depthai-ros \
    ros-humble-depthai-bridge \
    ros-humble-depthai-descriptions \
    && rm -rf /var/lib/apt/lists/*

# Install depthai Python library
RUN pip3 install depthai opencv-python

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Build the workspace (will be empty initially)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

CMD ["/bin/bash"]
