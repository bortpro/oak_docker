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
dipro_sr_mxid = '19443010F156DF1200'

class OAKVideoRecorderNode(Node):
    def __init__(self, mxid, camera_mode, duration=5, output_dir='videos'):
        super().__init__('oak_video_recorder_node')

        self.mxid = mxid
        self.camera_mode = camera_mode
        self.duration = duration
        self.output_dir = output_dir

        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Starting OAK video recording')
        self.get_logger().info(f'Mode: {camera_mode}')
        self.get_logger().info(f'MXID: {mxid}')
        self.get_logger().info(f'Duration: {duration}s')
        self.get_logger().info(f'Output: {output_dir}')

        # Start recording
        self.record_video()

    def setup_pipeline_s2(self):
        """Setup pipeline for S2 camera (4K H265)"""
        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        videoEnc = pipeline.create(dai.node.VideoEncoder)
        xout = pipeline.create(dai.node.XLinkOut)

        xout.setStreamName('h265')

        # Properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        videoEnc.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

        # Linking
        camRgb.video.link(videoEnc.input)
        videoEnc.bitstream.link(xout.input)

        return pipeline

    def setup_pipeline_sr(self):
        """Setup pipeline for SR camera (800P H265)"""
        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        videoEnc = pipeline.create(dai.node.VideoEncoder)
        xout = pipeline.create(dai.node.XLinkOut)

        xout.setStreamName('h265')

        # Properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        videoEnc.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)

        # Linking
        camRgb.video.link(videoEnc.input)
        videoEnc.bitstream.link(xout.input)

        return pipeline

    def record_video(self):
        """Record video from camera"""
        try:
            # Setup pipeline based on mode
            if self.camera_mode == 'oak_s2':
                pipeline = self.setup_pipeline_s2()
            else:
                pipeline = self.setup_pipeline_sr()

            # Connect to device
            with dai.Device(pipeline, dai.DeviceInfo(self.mxid)) as device:
                q = device.getOutputQueue(name="h265", maxSize=30, blocking=True)

                self.get_logger().info('Camera connected successfully')

                # Create filename with timestamp
                timestamp = int(time.time())
                prefix = "S2" if self.camera_mode == 'oak_s2' else "SR"
                filename = f"{self.output_dir}/{prefix}_video_{timestamp}.h265"

                # Record video
                with open(filename, 'wb') as videoFile:
                    self.get_logger().info(f'Recording started: {filename}')
                    start_time = time.time()

                    while time.time() - start_time < self.duration:
                        h265Packet = q.get()
                        h265Packet.getData().tofile(videoFile)

                self.get_logger().info(f'Recording complete: {filename}')
                self.get_logger().info(f'Duration: {self.duration}s')

        except Exception as e:
            self.get_logger().error(f'Error during recording: {str(e)}')
            raise

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='ROS2 node for recording video from OAK cameras'
    )
    parser.add_argument('mode', choices=['oak_s2', 'oak_sr'],
                       help='Camera recording mode')
    parser.add_argument('mxid', type=str, help='MXID of the OAK camera')
    parser.add_argument('--duration', type=int, default=5,
                       help='Recording duration in seconds')
    parser.add_argument('--output_dir', type=str, default='/videos',
                       help='Output directory for videos')

    # Filter out ROS arguments
    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        node = OAKVideoRecorderNode(
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

