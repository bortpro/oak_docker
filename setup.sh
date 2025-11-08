#!/bin/bash

echo "=========================================="
echo "  ROS2 Camera Setup Script"
echo "=========================================="
echo ""

# Start Docker container
echo "[1/3] Starting Docker container..."
docker compose up -d

# Wait for container to be fully ready
echo "[2/3] Waiting for container to start..."
sleep 2

# Build ROS2 packages inside container
echo "[3/3] Building ROS2 packages..."
docker exec ros2_oak_dev bash -c "cd /ros2_ws && colcon build --packages-select oak_camera_capture --symlink-install"

echo ""
echo "✓ Setup complete!"
echo ""
echo "Available commands (sourcing handled automatically by entrypoint):"
echo ""
echo "  - Image capture (S2):    docker exec ros2_oak_dev ros2 run oak_camera_capture ros_grab cam_grab_s2 18443010E157E40F00"
echo "  - Video record (S2):     docker exec ros2_oak_dev ros2 run oak_camera_capture ros_video oak_s2 18443010E157E40F00 --duration 5"
echo "  - Device discovery:      docker exec ros2_oak_dev ros2 run oak_camera_capture device_discovery"
echo "  - Webcam capture:        docker exec ros2_oak_dev ros2 run oak_camera_capture webcam_capture --num_images 1"
echo "  - Webcam stream:         docker exec ros2_oak_dev ros2 run oak_camera_capture webcam_stream"
echo "  - POE capture:           docker exec ros2_oak_dev ros2 run oak_camera_capture poe_capture 14442C10B1FFEBCF00"
echo ""
echo "Or enter container interactively:"
echo "  docker exec -it ros2_oak_dev bash"
echo ""
