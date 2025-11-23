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

class Task7MultiTableDynamicCancelRobot(Node):
    def __init__(self):
        super().__init__('task7_multi_table_dynamic_cancel_robot')
        
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
        
        # Order management
        self.pending_tables = []
        self.canceled_tables = []
        self.delivered_tables = []
        self.cancellation_lock = threading.Lock()
        self.cancel_input_active = False

        # --- Start the main task loop ---
        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def task_loop(self):
        """Main task execution loop"""
        while rclpy.ok():
            try:
                # Get multiple table numbers from user
                table_input = input("\n=== TASK 7: Multi-Table Delivery with Dynamic Cancellation ===\nEnter table numbers separated by spaces (e.g., 1 2 3) or 'q' to quit: ").strip()
                
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
                
                # Execute multi-table delivery sequence with dynamic cancellation
                self.execute_multi_table_with_dynamic_cancel(table_numbers)
                
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in task loop: {e}")

    def execute_multi_table_with_dynamic_cancel(self, table_numbers):
        """Execute the complete multi-table delivery with dynamic cancellation support"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting multi-table delivery with dynamic cancellation")
        self.get_logger().info(f"Order: {len(table_numbers)} tables - {table_numbers}")
        self.get_logger().info(f"{'='*60}")
        
        # Reset order tracking
        with self.cancellation_lock:
            self.pending_tables = table_numbers.copy()
            self.canceled_tables = []
            self.delivered_tables = []
        
        # Step 1: Go to Kitchen
        self.get_logger().info("Step 1: Moving to KITCHEN to collect orders...")
        if not self.navigate_to('kitchen'):
            self.get_logger().error("❌ Failed to reach kitchen. Aborting delivery.")
            self.navigate_to('home')
            return
        
        self.get_logger().info("✓ Reached kitchen successfully!")
        self.get_logger().info(f"🍔 Collected {len(self.pending_tables)} orders for tables: {self.pending_tables}")
        time.sleep(2)  # Brief pause for order collection
        
        # Start cancellation listener thread
        self.cancel_input_active = True
        cancel_listener_thread = threading.Thread(target=self.cancellation_input_listener, daemon=True)
        cancel_listener_thread.start()
        
        self.get_logger().info("💡 You can cancel orders at any time by typing table numbers (e.g., '2' or '1 3')")
        
        # Step 2: Deliver to Tables (with dynamic cancellation support)
        idx = 1
        while self.pending_tables:
            with self.cancellation_lock:
                if not self.pending_tables:
                    break
                table_number = self.pending_tables[0]
            
            self.get_logger().info(f"\nStep 2.{idx}: Moving to TABLE {table_number} ({idx}/{len(table_numbers)})...")
            
            if not self.navigate_to(table_number):
                self.get_logger().error(f"❌ Failed to reach Table {table_number}.")
                with self.cancellation_lock:
                    if table_number in self.pending_tables:
                        self.pending_tables.remove(table_number)
                        self.canceled_tables.append(table_number)
                idx += 1
                continue
            
            # Check if table was canceled during navigation
            with self.cancellation_lock:
                if table_number not in self.pending_tables:
                    self.get_logger().warn(f"⚠ Table {table_number} was canceled during navigation. Skipping delivery.")
                    idx += 1
                    continue
            
            self.get_logger().info(f"✓ Reached Table {table_number} successfully!")
            self.get_logger().info(f"🎉 Food delivered to Table {table_number}!")
            
            with self.cancellation_lock:
                self.pending_tables.remove(table_number)
                self.delivered_tables.append(table_number)
            
            time.sleep(1)  # Brief pause for delivery
            
            # Show progress
            remaining = len(self.pending_tables)
            if remaining > 0:
                self.get_logger().info(f"📊 Progress: Delivered to {len(self.delivered_tables)} tables, {remaining} remaining")
            
            idx += 1
        
        # Stop cancellation listener
        self.cancel_input_active = False
        
        # Step 3: Return to Kitchen (Always, regardless of cancellations)
        self.get_logger().info(f"\nStep 3: Returning to KITCHEN before going home...")
        
        if self.canceled_tables:
            self.get_logger().info(f"📋 Returning canceled orders for tables {self.canceled_tables} to kitchen")
        else:
            self.get_logger().info(f"📋 Returning to kitchen after completing all deliveries")
        
        if not self.navigate_to('kitchen'):
            self.get_logger().error("❌ Failed to return to kitchen")
        else:
            self.get_logger().info("✓ Returned to kitchen successfully!")
        
        # Step 4: Return Home
        self.get_logger().info(f"\nStep 4: Returning to HOME position...")
        if not self.navigate_to('home'):
            self.get_logger().error("❌ Failed to reach home position.")
            return
        
        # Delivery Summary
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("✓ Delivery mission complete! Robot returned home.")
        self.get_logger().info(f"📈 Delivery Summary:")
        self.get_logger().info(f"   - Total orders: {len(table_numbers)}")
        self.get_logger().info(f"   - Successfully delivered: {len(self.delivered_tables)} tables {self.delivered_tables}")
        self.get_logger().info(f"   - Canceled orders: {len(self.canceled_tables)} tables {self.canceled_tables}")
        self.get_logger().info(f"   - Success rate: {len(self.delivered_tables)/len(table_numbers)*100:.1f}%")
        self.get_logger().info(f"   - ✓ Returned to kitchen before home")
        self.get_logger().info(f"{'='*60}\n")

    def cancellation_input_listener(self):
        """Background thread to listen for order cancellations during delivery"""
        self.get_logger().info("📞 Cancellation listener active. Type table numbers to cancel (e.g., '2' or '1 3')")
        
        while self.cancel_input_active and rclpy.ok():
            try:
                cancel_input = input("Cancel table(s): ").strip()
                
                if not cancel_input:
                    continue
                
                # Parse cancellation input
                tables_to_cancel = []
                for num in cancel_input.split():
                    if num.isdigit():
                        table_num = int(num)
                        if table_num in [1, 2, 3]:
                            tables_to_cancel.append(table_num)
                
                if tables_to_cancel:
                    with self.cancellation_lock:
                        actually_canceled = []
                        for table_num in tables_to_cancel:
                            if table_num in self.pending_tables:
                                self.pending_tables.remove(table_num)
                                self.canceled_tables.append(table_num)
                                actually_canceled.append(table_num)
                                self.get_logger().warn(f"🚫 Order for Table {table_num} has been CANCELED!")
                            elif table_num in self.delivered_tables:
                                self.get_logger().warn(f"⚠ Table {table_num} already delivered. Cannot cancel.")
                            elif table_num in self.canceled_tables:
                                self.get_logger().warn(f"⚠ Table {table_num} already canceled.")
                            else:
                                self.get_logger().warn(f"⚠ Table {table_num} not in current order.")
                        
                        if actually_canceled:
                            self.get_logger().info(f"📋 Updated pending tables: {self.pending_tables}")
                
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"Error in cancellation listener: {e}")

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
    robot = Task7MultiTableDynamicCancelRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()