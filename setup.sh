#!/bin/bash

echo "=========================================="
echo "  ROS2 Camera Setup Script"
echo "=========================================="
echo ""

# Start Docker container
echo "[1/4] Starting Docker container..."
docker compose up -d

# Wait for container to be fully ready
echo "[2/4] Waiting for container to start..."
sleep 2

# Build ROS2 packages inside container
echo "[3/4] Building ROS2 packages..."
docker exec ros2_oak_dev bash -c "cd /ros2_ws && colcon build --packages-select oak_camera_capture"

# Source ROS2 environment
echo "[4/4] Sourcing ROS2 environment..."
docker exec ros2_oak_dev bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"

echo ""
echo "✓ Setup complete!"
echo ""
echo "Available commands:"
echo "  - Image capture (S2):    docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture ros_grab cam_grab_s2 18443010E157E40F00'"
echo "  - Video record (S2):     docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture ros_video oak_s2 18443010E157E40F00 --duration 5'"
echo "  - Device discovery:      docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture device_discovery'"
echo "  - Webcam capture:        docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture webcam_capture --num_images 1'"
echo "  - Webcam stream:         docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture webcam_stream'"
echo "  - POE capture:           docker exec -it ros2_oak_dev bash -c 'source install/setup.bash && ros2 run oak_camera_capture poe_capture 14442C10B1FFEBCF00'"
echo ""
echo "Or enter container interactively:"
echo "  docker exec -it ros2_oak_dev bash"
echo ""
