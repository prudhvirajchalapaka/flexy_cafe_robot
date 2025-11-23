#!/usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
import math

# Helper function to create a quaternion from yaw
def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

class Task4DeliveryRobot(Node):
    def __init__(self):
        super().__init__('task4_delivery_robot')
        
        # --- Predefined Positions (DO NOT CHANGE) ---
        self.poses = {
            'home': [-0.15, -0.26, 0.0],
            'kitchen': [4.20, -5.26, 1.57],
            1: [-0.47, -3.16, 0.0],
            2: [2.94, 2.74, -1.57],
            3: [5.58, -0.26, 3.14]
        }

        # --- Action Client for Navigation ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- State Variables ---
        self.goal_handle = None
        self.result_future = None
        self.action_done_event = threading.Event()
        self.cancel_requested = threading.Event()
        self.current_phase = None  # Track current navigation phase

        # --- Start the main task loop ---
        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def task_loop(self):
        """Main task execution loop"""
        while rclpy.ok():
            try:
                # Get table number from user
                table_input = input("\n=== TASK 4: Food Delivery with Cancellation ===\nEnter table number (1, 2, or 3) or 'q' to quit: ").strip()
                
                if table_input.lower() == 'q':
                    self.get_logger().info("Exiting delivery system...")
                    break
                
                if not table_input.isdigit():
                    self.get_logger().warn("Invalid input. Please enter a number.")
                    continue
                
                table_number = int(table_input)
                
                if table_number not in [1, 2, 3]:
                    self.get_logger().warn("Invalid table number. Choose 1, 2, or 3.")
                    continue
                
                # Reset cancellation flag
                self.cancel_requested.clear()
                
                # Execute delivery sequence with cancellation support
                self.execute_delivery_with_cancellation(table_number)
                
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in task loop: {e}")

    def execute_delivery_with_cancellation(self, table_number):
        """
        Execute the complete delivery sequence with cancellation handling:
        - Cancel while going to kitchen → Return home directly
        - Cancel while going to table → Return to kitchen first, then home
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting delivery to Table {table_number}")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info("💡 Type 'c' and press Enter at any time to cancel the current navigation")
        
        # Start cancellation listener thread
        cancel_thread = threading.Thread(target=self.cancellation_listener, daemon=True)
        cancel_thread.start()
        
        # Phase 1: Go to Kitchen
        self.current_phase = 'going_to_kitchen'
        self.get_logger().info("Phase 1: Moving to KITCHEN to pick up order...")
        
        nav_result = self.navigate_to('kitchen')
        
        if self.cancel_requested.is_set():
            # Canceled while going to kitchen
            self.get_logger().warn("⚠ Task canceled while going to KITCHEN!")
            self.get_logger().info("📋 Scenario: Returning HOME directly (no food picked up)")
            self.current_phase = 'returning_home_after_cancel'
            self.navigate_to('home')
            self.get_logger().info("✓ Returned home after cancellation")
            return
        
        if not nav_result:
            self.get_logger().error("❌ Failed to reach kitchen. Aborting delivery.")
            self.navigate_to('home')
            return
        
        self.get_logger().info("✓ Reached kitchen successfully!")
        self.get_logger().info("🍔 Food loaded on robot")
        time.sleep(1)  # Brief pause for pickup simulation
        
        # Phase 2: Go to Table
        self.current_phase = 'going_to_table'
        self.get_logger().info(f"Phase 2: Moving to TABLE {table_number} for delivery...")
        
        nav_result = self.navigate_to(table_number)
        
        if self.cancel_requested.is_set():
            # Canceled while going to table
            self.get_logger().warn(f"⚠ Task canceled while going to TABLE {table_number}!")
            self.get_logger().info("📋 Scenario: Returning to KITCHEN first (food needs to be returned)")
            
            self.current_phase = 'returning_to_kitchen_after_cancel'
            if not self.navigate_to('kitchen'):
                self.get_logger().error("❌ Failed to return to kitchen")
            else:
                self.get_logger().info("✓ Returned food to kitchen")
            
            self.current_phase = 'returning_home_after_cancel'
            self.navigate_to('home')
            self.get_logger().info("✓ Returned home after cancellation")
            return
        
        if not nav_result:
            self.get_logger().error(f"❌ Failed to reach Table {table_number}.")
            self.get_logger().info("📋 Returning to KITCHEN first")
            self.navigate_to('kitchen')
            self.navigate_to('home')
            return
        
        self.get_logger().info(f"✓ Reached Table {table_number} successfully!")
        self.get_logger().info("🎉 Food delivered!")
        time.sleep(1)  # Brief pause for delivery simulation
        
        # Phase 3: Return Home (successful completion)
        self.current_phase = 'returning_home'
        self.get_logger().info("Phase 3: Returning to HOME position...")
        
        if not self.navigate_to('home'):
            self.get_logger().error("❌ Failed to reach home position.")
            return
        
        self.get_logger().info("✓ Delivery complete! Robot returned home.")
        self.get_logger().info(f"{'='*60}\n")

    def cancellation_listener(self):
        """Thread to listen for cancellation input from user"""
        while not self.cancel_requested.is_set():
            try:
                user_input = input().strip().lower()
                if user_input == 'c':
                    self.get_logger().info("🛑 User requested cancellation!")
                    self.cancel_requested.set()
                    if self.goal_handle:
                        self.cancel_navigation()
                    break
            except EOFError:
                break

    def navigate_to(self, pose_key):
        """
        Navigate to a specific pose
        Returns True if successful, False otherwise
        Monitors for cancellation during navigation
        """
        if pose_key not in self.poses:
            self.get_logger().error(f"Unknown location: {pose_key}")
            return False
        
        pose = self.poses[pose_key]
        
        # Create goal message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = quaternion_from_yaw(pose[2])
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal to action server
        self.get_logger().info(f"🚗 Navigating to {pose_key}: [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]")
        self.nav_to_pose_client.wait_for_server()
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected!')
            return False

        # Wait for result
        self.action_done_event.clear()
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

        # Wait for navigation to complete or cancellation
        while not self.action_done_event.is_set():
            if self.cancel_requested.is_set():
                # Don't cancel if we're already in a return phase
                if 'after_cancel' not in self.current_phase:
                    return False
            time.sleep(0.1)
        
        # Check final status
        status = self.goal_handle.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        elif status == GoalStatus.STATUS_CANCELED:
            return False
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            return False

    def cancel_navigation(self):
        """Cancel the current navigation goal"""
        if self.goal_handle:
            self.get_logger().info("Sending cancellation request to navigation...")
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info("Navigation goal canceled")

    def result_callback(self, future):
        """Callback when navigation action completes"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation goal was canceled')
        else:
            self.get_logger().warn(f'Navigation goal failed with status: {status}')
        self.action_done_event.set()

    def feedback_callback(self, feedback_msg):
        """Callback for navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} meters',
            throttle_duration_sec=3
        )


def main(args=None):
    rclpy.init(args=args)
    robot = Task4DeliveryRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()