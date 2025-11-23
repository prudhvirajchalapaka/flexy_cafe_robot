#!/usr/bin/env python3
"""
Live Camera Viewer Node for Flexy Robot
Subscribes to simulated camera and displays live video feed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class LiveCameraViewer(Node):
    def __init__(self):
        super().__init__('live_camera_viewer')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('window_name', 'Flexy Robot - Live Camera Feed')
        self.declare_parameter('show_info', True)
        self.declare_parameter('resize_width', 0)  # 0 means no resize
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.show_info = self.get_parameter('show_info').value
        self.resize_width = self.get_parameter('resize_width').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.latest_image = None
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        self.fps = 0.0
        self.is_recording = False
        self.video_writer = None
        
        # Create subscriber with larger queue for smoother video
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Timer for display update (30 Hz)
        self.display_timer = self.create_timer(0.033, self.display_callback)
        
        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Live Camera Viewer Started')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info('=' * 60)
        self.print_controls()

    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize if requested
            if self.resize_width > 0:
                height, width = cv_image.shape[:2]
                aspect_ratio = height / width
                new_height = int(self.resize_width * aspect_ratio)
                cv_image = cv2.resize(cv_image, (self.resize_width, new_height))
            
            # Store latest image
            self.latest_image = cv_image
            self.frame_count += 1
            
            # Calculate FPS
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0.1:  # Update every 100ms
                self.fps = 1.0 / dt if dt > 0 else 0.0
                self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def display_callback(self):
        """Timer callback to display the image"""
        if self.latest_image is None:
            # Show "waiting for camera" message
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, 'Waiting for camera feed...', (120, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(blank, 'Make sure the robot is spawned in Gazebo', (80, 280),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow(self.window_name, blank)
            cv2.waitKey(1)
            return
        
        # Create a copy to draw on
        display_image = self.latest_image.copy()
        
        # Add overlay information
        if self.show_info:
            display_image = self.add_overlay(display_image)
        
        # Write to video if recording
        if self.is_recording and self.video_writer is not None:
            self.video_writer.write(display_image)
        
        # Display the image
        cv2.imshow(self.window_name, display_image)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        self.handle_keypress(key)

    def add_overlay(self, image):
        """Add information overlay to the image"""
        h, w = image.shape[:2]
        
        # Semi-transparent overlay panel
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.5, image, 0.5, 0)
        
        # FPS
        fps_text = f'FPS: {self.fps:.1f}'
        cv2.putText(image, fps_text, (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Frame count
        frame_text = f'Frame: {self.frame_count}'
        cv2.putText(image, frame_text, (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Resolution
        res_text = f'Resolution: {w}x{h}'
        cv2.putText(image, res_text, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Recording indicator
        if self.is_recording:
            cv2.circle(image, (w - 30, 30), 15, (0, 0, 255), -1)
            cv2.putText(image, 'REC', (w - 80, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Status
        status_text = 'LIVE'
        cv2.putText(image, status_text, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Timestamp in bottom right
        timestamp = datetime.now().strftime('%H:%M:%S')
        cv2.putText(image, timestamp, (w - 100, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return image

    def handle_keypress(self, key):
        """Handle keyboard input"""
        if key == ord('q') or key == 27:  # q or ESC
            self.get_logger().info('Quitting...')
            self.cleanup()
            rclpy.shutdown()
            
        elif key == ord('s'):
            self.save_snapshot()
            
        elif key == ord('r'):
            self.toggle_recording()
            
        elif key == ord('h'):
            self.print_controls()
            
        elif key == ord('i'):
            self.show_info = not self.show_info
            status = 'ON' if self.show_info else 'OFF'
            self.get_logger().info(f'Info overlay: {status}')
            
        elif key == ord('f'):
            # Toggle fullscreen
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN,
                                 cv2.WINDOW_FULLSCREEN)
            
        elif key == ord('n'):
            # Normal window
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN,
                                 cv2.WINDOW_NORMAL)

    def save_snapshot(self):
        """Save current frame as image"""
        if self.latest_image is not None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'flexy_snapshot_{timestamp}.png'
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f'Snapshot saved: {filename}')

    def toggle_recording(self):
        """Toggle video recording"""
        if not self.is_recording:
            # Start recording
            if self.latest_image is not None:
                h, w = self.latest_image.shape[:2]
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f'flexy_recording_{timestamp}.avi'
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (w, h))
                self.is_recording = True
                self.get_logger().info(f'Recording started: {filename}')
        else:
            # Stop recording
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            self.is_recording = False
            self.get_logger().info('Recording stopped')

    def print_controls(self):
        """Print control information"""
        controls = """
        ╔════════════════════════════════════════════════╗
        ║     FLEXY ROBOT LIVE CAMERA VIEWER CONTROLS    ║
        ╠════════════════════════════════════════════════╣
        ║  Q / ESC   - Quit viewer                       ║
        ║  S         - Save snapshot (PNG)               ║
        ║  R         - Toggle video recording (AVI)      ║
        ║  I         - Toggle info overlay               ║
        ║  F         - Fullscreen mode                   ║
        ║  N         - Normal window mode                ║
        ║  H         - Show this help                    ║
        ╚════════════════════════════════════════════════╝
        """
        self.get_logger().info(controls)

    def cleanup(self):
        """Cleanup resources"""
        if self.is_recording and self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()

    def destroy_node(self):
        """Override destroy to cleanup"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = LiveCameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            viewer.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()