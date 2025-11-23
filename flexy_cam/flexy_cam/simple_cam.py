#!/usr/bin/env python3
"""
Instant Camera Viewer - Just run this!
No package needed, just copy and execute.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class InstantCameraViewer(Node):
    def __init__(self):
        super().__init__('instant_camera_viewer')
        self.bridge = CvBridge()
        self.latest_image = None
        self.frame_count = 0
        
        # Subscribe to your camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        print("\n" + "="*70)
        print("🎥 FLEXY ROBOT LIVE CAMERA VIEWER".center(70))
        print("="*70)
        print("  ✓ Subscribed to: /camera/image")
        print("  ✓ Waiting for images...")
        print("\n  Controls:")
        print("    [Q] - Quit")
        print("    [S] - Save snapshot")
        print("    [SPACE] - Pause/Resume")
        print("="*70 + "\n")
        
        self.paused = False
        
    def image_callback(self, msg):
        """Receive and convert images"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def run(self):
        """Main display loop"""
        cv2.namedWindow('Flexy Robot Camera - LIVE', cv2.WINDOW_NORMAL)
        
        no_image_shown = False
        snapshot_count = 0
        
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            if self.latest_image is not None:
                # Create display image
                display = self.latest_image.copy()
                
                # Add frame counter
                h, w = display.shape[:2]
                cv2.putText(display, f'Frame: {self.frame_count}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Add pause indicator
                if self.paused:
                    cv2.putText(display, 'PAUSED', (w//2 - 60, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    cv2.circle(display, (w - 30, 30), 8, (0, 0, 255), -1)
                    cv2.putText(display, 'LIVE', (w - 80, 35),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Show the image
                cv2.imshow('Flexy Robot Camera - LIVE', display)
                
                if not no_image_shown:
                    print("✓ Camera feed is LIVE! 🎉")
                    no_image_shown = True
                    
            else:
                # Waiting screen
                waiting = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting, 'Waiting for camera feed...', (140, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow('Flexy Robot Camera - LIVE', waiting)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # Q or ESC
                print("\n👋 Closing camera viewer...")
                break
                
            elif key == ord('s'):  # Save snapshot
                if self.latest_image is not None:
                    filename = f'flexy_snapshot_{snapshot_count:04d}.png'
                    cv2.imwrite(filename, self.latest_image)
                    print(f"📸 Snapshot saved: {filename}")
                    snapshot_count += 1
                    
            elif key == ord(' '):  # Space to pause
                self.paused = not self.paused
                status = "PAUSED" if self.paused else "RESUMED"
                print(f"⏸️  {status}")
        
        cv2.destroyAllWindows()
        print("✓ Camera viewer closed.\n")


def main():
    rclpy.init()
    
    try:
        viewer = InstantCameraViewer()
        viewer.run()
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()