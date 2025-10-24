#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import depthai as dai
import argparse
from pathlib import Path
import time
import sys

# Default mxids for the cameras
dipro_s2_mxid = '18443010E157E40F00'
dipro_sr_mxid = '194430109161782700'

class OAKImageCaptureNode(Node):
    def __init__(self, mxid, camera_mode, duration=2, output_dir='images'):
        super().__init__('oak_image_capture_node')

        self.mxid = mxid
        self.camera_mode = camera_mode
        self.duration = duration
        self.output_dir = output_dir
        self.image_count = 0

        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Starting OAK camera capture')
        self.get_logger().info(f'Mode: {camera_mode}')
        self.get_logger().info(f'MXID: {mxid}')
        self.get_logger().info(f'Duration: {duration}s')
        self.get_logger().info(f'Output: {output_dir}')

        # Start capture
        self.capture_images()

    def setup_pipeline_s2(self):
        """Setup pipeline for S2 camera"""
        pipeline = dai.Pipeline()

        # Define color camera
        camRgb = pipeline.createColorCamera()
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setIspScale(2, 3)

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

    def setup_pipeline_sr(self):
        """Setup pipeline for SR camera"""
        pipeline = dai.Pipeline()

        # Define color camera
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

        # Create encoder for JPEG
        videoEnc = pipeline.create(dai.node.VideoEncoder)
        videoEnc.setDefaultProfilePreset(
            camRgb.getVideoSize(),
            camRgb.getFps(),
            dai.VideoEncoderProperties.Profile.MJPEG
        )
        camRgb.video.link(videoEnc.input)

        # Create output
        xoutJpeg = pipeline.create(dai.node.XLinkOut)
        xoutJpeg.setStreamName("jpeg")
        videoEnc.bitstream.link(xoutJpeg.input)

        return pipeline

    def capture_images(self):
        """Capture images from camera"""
        try:
            # Setup pipeline based on mode
            if self.camera_mode == 'cam_grab_s2':
                pipeline = self.setup_pipeline_s2()
            else:
                pipeline = self.setup_pipeline_sr()

            # Connect to device
            with dai.Device(pipeline, dai.DeviceInfo(self.mxid)) as device:
                qJpeg = device.getOutputQueue(name="jpeg", maxSize=30, blocking=True)

                self.get_logger().info('Camera connected successfully')

                start_time = time.time()
                end_time = start_time + self.duration

                while time.time() < end_time:
                    elapsed = time.time() - start_time

                    # Skip first 1.5 seconds for camera stabilization
                    if elapsed > 1.5:
                        for encFrame in qJpeg.tryGetAll():
                            timestamp = int(time.time() * 1)
                            prefix = "S2" if self.camera_mode == 'cam_grab_s2' else "SR"
                            filename = f"{self.output_dir}/{prefix}_{timestamp}.jpeg"

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
        description='ROS2 node for capturing images from OAK cameras'
    )
    parser.add_argument('mode', choices=['cam_grab_s2', 'cam_grab_sr'],
                       help='Camera capture mode')
    parser.add_argument('mxid', type=str, help='MXID of the OAK-D camera')
    parser.add_argument('--duration', type=int, default=2,
                       help='Capture duration in seconds')
    parser.add_argument('--output_dir', type=str, default='/images',
                       help='Output directory for images')

    # Filter out ROS arguments
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        node = OAKImageCaptureNode(
            mxid=parsed_args.mxid,
            camera_mode=parsed_args.mode,
            duration=parsed_args.duration,
            output_dir=parsed_args.output_dir
        )
        # Node completes execution, no spinning needed
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
