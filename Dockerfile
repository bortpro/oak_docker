FROM ros:humble-ros-base

# OS deps
RUN apt-get update && apt-get install -y \
    python3-pip usbutils wget git libusb-1.0-0 python3-opencv \
    ros-humble-cv-bridge ros-humble-vision-msgs \
    build-essential python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python deps for DepthAI capture
RUN pip3 install --no-cache-dir depthai==2.30.0.0 opencv-python

# Workspace
WORKDIR /ros2_ws

# Copy and set entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# No colcon build here; build after volumes are mounted
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
# Default shell
CMD ["/bin/bash"]
