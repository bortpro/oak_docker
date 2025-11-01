#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import depthai as dai

class OAKDeviceDiscoveryNode(Node):
    def __init__(self):
        super().__init__('oak_device_discovery_node')
        self.discover_devices()

    def discover_devices(self):
        """List all available OAK cameras"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            
            if not devices:
                self.get_logger().info('No OAK devices found')
                return
            
            self.get_logger().info(f'Found {len(devices)} device(s):')
            
            for device in devices:
                mxid = device.getMxId()
                state = device.state
                self.get_logger().info(f'  MXID: {mxid} | State: {state}')
                
        except Exception as e:
            self.get_logger().error(f'Error discovering devices: {str(e)}')
            raise

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OAKDeviceDiscoveryNode()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

