#!/usr/bin/env python3
"""
Camera viewer node for Flexy/Garuda robot.

Subscribes to a sensor_msgs/Image topic (default: /camera/image) and displays the live feed
using OpenCV. Uses cv_bridge to convert ROS images to OpenCV images.

Usage:
  ros2 run garuda_control camera_viewer
or
  ros2 run garuda_control camera_viewer --ros-args -r /camera/image:=/your_camera_topic

Parameters:
  image_topic (string) : topic to subscribe to (default: /camera/image)
  window_name (string) : OpenCV window title (default: "Flexy Camera")
  use_compressed (bool) : if true, subscribes to compressed image topic (default: false)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        # parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('window_name', 'Flexy Camera')
        self.declare_parameter('use_compressed', False)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value

        # CV bridge
        self.bridge = CvBridge()

        # QoS for image (best-effort is typical for cameras)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # create subscriber
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos
        )

        # create OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)

    def image_callback(self, msg: Image):
        try:
            # convert ROS Image to OpenCV image (BGR8 expected from Gazebo camera)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # show image
            cv2.imshow(self.window_name, cv_image)
            # waitKey is required for imshow to update. Non-blocking small delay.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # if user presses 'q' in the window close node
                self.get_logger().info("User requested quit (q). Shutting down.")
                rclpy.shutdown()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_callback: {e}")

    def destroy_node(self):
        # cleanup OpenCV windows
        try:
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down camera viewer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()