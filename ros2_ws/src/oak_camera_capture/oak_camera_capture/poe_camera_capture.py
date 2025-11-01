#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import depthai as dai
import argparse
from pathlib import Path
import time
import sys

# Default mxid for POE camera
poe_mxid = '14442C10B1FFEBCF00'

class POECameraCaptureNode(Node):
    def __init__(self, mxid, num_images=1, output_dir='poe_images'):
        super().__init__('poe_camera_capture_node')

        self.mxid = mxid
        self.num_images = num_images
        self.output_dir = output_dir
        self.image_count = 0

        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Starting POE camera capture')
        self.get_logger().info(f'MXID: {mxid}')
        self.get_logger().info(f'Number of images: {num_images}')
        self.get_logger().info(f'Output: {output_dir}')

        # Start capture
        self.capture_images()

    def setup_pipeline_poe(self):
        """Setup pipeline for POE camera"""
        pipeline = dai.Pipeline()

        # Define color camera (POE cameras typically use the default socket)
        camRgb = pipeline.createColorCamera()
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(30)

        # Create encoder for JPEG
        videoEnc = pipeline.createVideoEncoder()
        videoEnc.setDefaultProfilePreset(
            camRgb.getVideoSize(),
            camRgb.getFps(),
            dai.VideoEncoderProperties.Profile.MJPEG
        )
        camRgb.video.link(videoEnc.input)

        # Create output
        xoutJpeg = pipeline.createXLinkOut()
        xoutJpeg.setStreamName("jpeg")
        videoEnc.bitstream.link(xoutJpeg.input)

        return pipeline

    def capture_images(self):
        """Capture images from POE camera"""
        try:
            pipeline = self.setup_pipeline_poe()

            # Connect to device using network discovery
            # For POE cameras, we need to specify the device info differently
            with dai.Device(pipeline, dai.DeviceInfo(self.mxid)) as device:
                qJpeg = device.getOutputQueue(name="jpeg", maxSize=30, blocking=True)

                self.get_logger().info('POE camera connected successfully')

                # Capture the specified number of images
                while self.image_count < self.num_images:
                    encFrame = qJpeg.get()
                    
                    timestamp = int(time.time() * 1000)  # milliseconds for uniqueness
                    filename = f"{self.output_dir}/POE_{timestamp}.jpeg"

                    # Save image
                    with open(filename, "wb") as f:
                        f.write(bytearray(encFrame.getData()))

                    self.image_count += 1
                    self.get_logger().info(f'Saved: {filename}')

                self.get_logger().info(f'Capture complete! Saved {self.image_count} images')
                self.get_logger().info(f'Images location: {self.output_dir}')

        except Exception as e:
            self.get_logger().error(f'Error during capture: {str(e)}')
            raise

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='ROS2 node for capturing images from POE OAK cameras'
    )
    parser.add_argument('mxid', type=str, help='MXID of the POE camera')
    parser.add_argument('--num_images', type=int, default=1,
                       help='Number of images to capture')
    parser.add_argument('--output_dir', type=str, default='/poe_images',
                       help='Output directory for images')

    # Filter out ROS arguments
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        node = POECameraCaptureNode(
            mxid=parsed_args.mxid,
            num_images=parsed_args.num_images,
            output_dir=parsed_args.output_dir
        )
        # Node completes execution, no spinning needed
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

