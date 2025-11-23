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

class Task5MultiTableDeliveryRobot(Node):
    def __init__(self):
        super().__init__('task5_multi_table_delivery_robot')
        
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

        # --- Start the main task loop ---
        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def task_loop(self):
        """Main task execution loop"""
        while rclpy.ok():
            try:
                # Get multiple table numbers from user
                table_input = input("\n=== TASK 5: Multi-Table Food Delivery ===\nEnter table numbers separated by spaces (e.g., 1 2 3) or 'q' to quit: ").strip()
                
                if table_input.lower() == 'q':
                    self.get_logger().info("Exiting delivery system...")
                    break
                
                # Parse table numbers
                table_numbers = []
                for num in table_input.split():
                    if num.isdigit():
                        table_num = int(num)
                        if table_num in [1, 2, 3]:
                            table_numbers.append(table_num)
                        else:
                            self.get_logger().warn(f"Invalid table number: {table_num}. Must be 1, 2, or 3.")
                    else:
                        self.get_logger().warn(f"Invalid input: {num}. Please enter numbers only.")
                
                if not table_numbers:
                    self.get_logger().warn("No valid table numbers provided. Please try again.")
                    continue
                
                # Remove duplicates and sort
                table_numbers = sorted(list(set(table_numbers)))
                
                # Execute multi-table delivery sequence
                self.execute_multi_table_delivery(table_numbers)
                
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in task loop: {e}")

    def execute_multi_table_delivery(self, table_numbers):
        """Execute the complete multi-table delivery sequence"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting multi-table delivery")
        self.get_logger().info(f"Order: {len(table_numbers)} tables - {table_numbers}")
        self.get_logger().info(f"{'='*60}")
        
        # Step 1: Go to Kitchen
        self.get_logger().info("Step 1: Moving to KITCHEN to collect orders...")
        if not self.navigate_to('kitchen'):
            self.get_logger().error("❌ Failed to reach kitchen. Aborting delivery.")
            self.navigate_to('home')
            return
        
        self.get_logger().info("✓ Reached kitchen successfully!")
        self.get_logger().info(f"🍔 Collected {len(table_numbers)} orders for tables: {table_numbers}")
        time.sleep(2)  # Brief pause for order collection
        
        # Step 2: Deliver to Multiple Tables
        delivered_tables = []
        failed_tables = []
        
        for idx, table_number in enumerate(table_numbers, 1):
            self.get_logger().info(f"\nStep 2.{idx}: Moving to TABLE {table_number} ({idx}/{len(table_numbers)})...")
            
            if not self.navigate_to(table_number):
                self.get_logger().error(f"❌ Failed to reach Table {table_number}.")
                failed_tables.append(table_number)
                continue
            
            self.get_logger().info(f"✓ Reached Table {table_number} successfully!")
            self.get_logger().info(f"🎉 Food delivered to Table {table_number}!")
            delivered_tables.append(table_number)
            time.sleep(1)  # Brief pause for delivery
            
            # Show progress
            remaining = len(table_numbers) - idx
            if remaining > 0:
                self.get_logger().info(f"📊 Progress: {idx}/{len(table_numbers)} completed, {remaining} remaining")
        
        # Step 3: Return Home
        self.get_logger().info(f"\nStep 3: All deliveries complete! Returning to HOME position...")
        if not self.navigate_to('home'):
            self.get_logger().error("❌ Failed to reach home position.")
            return
        
        # Delivery Summary
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("✓ Delivery mission complete! Robot returned home.")
        self.get_logger().info(f"📈 Delivery Summary:")
        self.get_logger().info(f"   - Total orders: {len(table_numbers)}")
        self.get_logger().info(f"   - Successfully delivered: {len(delivered_tables)} tables {delivered_tables}")
        if failed_tables:
            self.get_logger().warn(f"   - Failed deliveries: {len(failed_tables)} tables {failed_tables}")
        else:
            self.get_logger().info(f"   - Failed deliveries: 0 🎊")
        self.get_logger().info(f"   - Success rate: {len(delivered_tables)/len(table_numbers)*100:.1f}%")
        self.get_logger().info(f"{'='*60}\n")

    def navigate_to(self, pose_key):
        """
        Navigate to a specific pose
        Returns True if successful, False otherwise
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
        location_name = f"Table {pose_key}" if isinstance(pose_key, int) else pose_key.upper()
        self.get_logger().info(f"🚗 Navigating to {location_name}: [{pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}]")
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

        # Wait for navigation to complete
        while not self.action_done_event.is_set():
            time.sleep(0.1)
        
        # Check final status
        status = self.goal_handle.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            return False

    def result_callback(self, future):
        """Callback when navigation action completes"""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded!')
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
    robot = Task5MultiTableDeliveryRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()