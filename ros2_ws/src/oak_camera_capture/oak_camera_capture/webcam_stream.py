#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import argparse
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from threading import Thread
import time

class WebcamStreamNode(Node):
    def __init__(self, device_id=0, port=8080, fps=30, width=640, height=480):
        super().__init__('webcam_stream_node')

        self.device_id = device_id
        self.port = port
        self.fps = fps
        self.width = width
        self.height = height
        self.frame = None
        self.running = True

        self.get_logger().info(f'Starting webcam stream server')
        self.get_logger().info(f'Device ID: {device_id}')
        self.get_logger().info(f'Port: {port}')
        self.get_logger().info(f'Resolution: {width}x{height}')
        self.get_logger().info(f'FPS: {fps}')

        # Start camera capture in background thread
        self.capture_thread = Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        # Start HTTP server
        self.start_server()

    def capture_loop(self):
        """Continuously capture frames from webcam"""
        cap = cv2.VideoCapture(self.device_id)

        if not cap.isOpened():
            self.get_logger().error(f'Failed to open webcam (device {self.device_id})')
            self.running = False
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.get_logger().info('Webcam opened successfully')

        while self.running:
            ret, frame = cap.read()
            if ret:
                self.frame = frame
            time.sleep(1.0 / self.fps)

        cap.release()

    def generate_frames(self):
        """Generate MJPEG stream"""
        while self.running:
            if self.frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', self.frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(1.0 / self.fps)

    def start_server(self):
        """Start HTTP server for streaming"""
        node_instance = self

        class StreamHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    html = f'''
                    <!DOCTYPE html>
                    <html lang="en">
                    <head>
                        <meta charset="UTF-8">
                        <meta name="viewport" content="width=device-width, initial-scale=1.0">
                        <title>Webcam Stream</title>
                        <style>
                            * {{
                                margin: 0;
                                padding: 0;
                                box-sizing: border-box;
                            }}
                            body {{
                                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
                                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                                min-height: 100vh;
                                display: flex;
                                flex-direction: column;
                                align-items: center;
                                justify-content: center;
                                padding: 20px;
                            }}
                            .container {{
                                background: rgba(255, 255, 255, 0.95);
                                border-radius: 20px;
                                padding: 40px;
                                box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
                                max-width: 1200px;
                                width: 100%;
                            }}
                            h1 {{
                                color: #333;
                                margin-bottom: 10px;
                                font-size: 2em;
                                text-align: center;
                            }}
                            .info {{
                                color: #666;
                                text-align: center;
                                margin-bottom: 30px;
                                font-size: 0.95em;
                            }}
                            .stream-wrapper {{
                                position: relative;
                                width: 100%;
                                background: #000;
                                border-radius: 12px;
                                overflow: hidden;
                                box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
                            }}
                            img {{
                                width: 100%;
                                height: auto;
                                display: block;
                            }}
                            .status {{
                                position: absolute;
                                top: 15px;
                                right: 15px;
                                background: rgba(76, 175, 80, 0.9);
                                color: white;
                                padding: 8px 16px;
                                border-radius: 20px;
                                font-size: 0.85em;
                                font-weight: 600;
                                display: flex;
                                align-items: center;
                                gap: 8px;
                            }}
                            .status::before {{
                                content: '';
                                width: 8px;
                                height: 8px;
                                background: white;
                                border-radius: 50%;
                                animation: pulse 2s ease-in-out infinite;
                            }}
                            @keyframes pulse {{
                                0%, 100% {{ opacity: 1; }}
                                50% {{ opacity: 0.5; }}
                            }}
                            .footer {{
                                text-align: center;
                                margin-top: 20px;
                                color: #999;
                                font-size: 0.85em;
                            }}
                        </style>
                    </head>
                    <body>
                        <div class="container">
                            <h1>📹 Live Webcam Stream</h1>
                            <div class="info">Streaming from Raspberry Pi 5 • {node_instance.width}x{node_instance.height} @ {node_instance.fps}fps</div>
                            <div class="stream-wrapper">
                                <div class="status">● LIVE</div>
                                <img src="/stream" alt="Live webcam stream">
                            </div>
                            <div class="footer">Device {node_instance.device_id} • Port {node_instance.port}</div>
                        </div>
                    </body>
                    </html>
                    '''
                    self.wfile.write(html.encode())

                elif self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    try:
                        for frame in node_instance.generate_frames():
                            self.wfile.write(frame)
                    except Exception as e:
                        node_instance.get_logger().error(f'Stream error: {{str(e)}}')

            def log_message(self, format, *args):
                pass  # Suppress HTTP logs


        try:
            server = HTTPServer(('0.0.0.0', self.port), StreamHandler)
            self.get_logger().info(f'Stream server started at http://<rpi-ip>:{self.port}')
            self.get_logger().info('Press Ctrl+C to stop')
            server.serve_forever()
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down stream server')
            self.running = False
        except Exception as e:
            self.get_logger().error(f'Server error: {str(e)}')
            self.running = False

def main(args=None):
    parser = argparse.ArgumentParser(
        description='ROS2 node for streaming webcam over HTTP'
    )
    parser.add_argument('--device_id', type=int, default=0,
                       help='Webcam device ID (default: 0)')
    parser.add_argument('--port', type=int, default=8080,
                       help='HTTP server port (default: 8080)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Frames per second (default: 30)')
    parser.add_argument('--width', type=int, default=640,
                       help='Frame width (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                       help='Frame height (default: 480)')

    filtered_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]
    parsed_args = parser.parse_args(filtered_args)

    rclpy.init(args=args)

    try:
        node = WebcamStreamNode(
            device_id=parsed_args.device_id,
            port=parsed_args.port,
            fps=parsed_args.fps,
            width=parsed_args.width,
            height=parsed_args.height
        )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
