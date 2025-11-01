#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import argparse
from pathlib import Path
import time
import sys

class WebcamCaptureNode(Node):
    def __init__(self, num_images=1, output_dir='webcam_images', device_id=0):
        super().__init__('webcam_capture_node')

        self.num_images = num_images
        self.output_dir = output_dir
        self.device_id = device_id

        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Starting webcam capture')
        self.get_logger().info(f'Device ID: {device_id}')
        self.get_logger().info(f'Number of images: {num_images}')
        self.get_logger().info(f'Output: {output_dir}')

        # Start capture
        self.capture_images()

    def capture_images(self):
        """Capture images from webcam"""
        try:
            # Open webcam
            cap = cv2.VideoCapture(self.device_id)
            
            if not cap.isOpened():
                self.get_logger().error(f'Failed to open webcam (device {self.device_id})')
                raise RuntimeError(f'Cannot open webcam device {self.device_id}')

            self.get_logger().info('Webcam connected successfully')

            # Skip first frame for stabilization
            cap.read()

            count = 0
            while count < self.num_images:
                ret, frame = cap.read()
                
                if not ret:
                    self.get_logger().error('Failed to read frame from webcam')
                    break

                # Save image
                timestamp = int(time.time() * 1000)  # milliseconds for uniqueness
                filename = f"{self.output_dir}/webcam_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                
                count += 1
                self.get_logger().info(f'Captured: {filename}')

            cap.release()
            self.get_logger().info(f'Capture complete! Saved {count} images')

        except Exception as e:
            self.get_logger().error(f'Error during capture: {str(e)}')
            raise

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='ROS2 node for capturing images from webcam'
    )
    parser.add_argument('--num_images', type=int, default=1,
                       help='Number of images to capture')
    parser.add_argument('--output_dir', type=str, default='/webcam_images',
                       help='Output directory for images')
    parser.add_argument('--device_id', type=int, default=0,
                       help='Webcam device ID (0 for first camera)')

    # Filter out ROS arguments
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        node = WebcamCaptureNode(
            num_images=parsed_args.num_images,
            output_dir=parsed_args.output_dir,
            device_id=parsed_args.device_id
        )
        # Node completes execution, no spinning needed
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

