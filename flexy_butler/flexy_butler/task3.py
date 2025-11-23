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

class Task3DeliveryRobot(Node):
    def __init__(self):
        super().__init__('task3_delivery_robot')
        
        # --- Predefined Positions (DO NOT CHANGE) ---
        self.poses = {
            'home': [-0.15, -0.26, 0.0],
            'kitchen': [4.20, -5.26, 1.57],
            1: [-0.47, -3.16, 0.0],
            2: [2.94, 2.74, -1.57],
            3: [5.58, -0.26, 3.14]
        }

        # --- Timeout Configuration (in seconds) ---
        self.KITCHEN_TIMEOUT = 30  # 30 seconds for kitchen confirmation
        self.TABLE_TIMEOUT = 20    # 20 seconds for table confirmation

        # --- Action Client for Navigation ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- State Variables ---
        self.goal_handle = None
        self.result_future = None
        self.action_done_event = threading.Event()
        self.food_picked_up = False  # Track if food was picked up from kitchen

        # --- Start the main task loop ---
        self.task_thread = threading.Thread(target=self.task_loop, daemon=True)
        self.task_thread.start()

    def task_loop(self):
        """Main task execution loop"""
        while rclpy.ok():
            try:
                # Get table number from user
                table_input = input("\n=== TASK 3: Food Delivery with Kitchen Return ===\nEnter table number (1, 2, or 3) or 'q' to quit: ").strip()
                
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
                
                # Execute delivery sequence with kitchen return logic
                self.execute_delivery_with_kitchen_return(table_number)
                
            except KeyboardInterrupt:
                self.get_logger().info("Interrupted by user.")
                break
            except Exception as e:
                self.get_logger().error(f"Error in task loop: {e}")

    def execute_delivery_with_kitchen_return(self, table_number):
        """
        Execute the complete delivery sequence with kitchen return logic:
        - Kitchen timeout → Return home directly
        - Table timeout → Return to kitchen first, then home
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Starting delivery to Table {table_number}")
        self.get_logger().info(f"{'='*60}")
        
        # Reset food pickup status
        self.food_picked_up = False
        
        # SCENARIO A: Go to Kitchen and wait for confirmation
        self.get_logger().info("Step 1: Moving to KITCHEN to pick up order...")
        if not self.navigate_to('kitchen'):
            self.get_logger().error("❌ Failed to reach kitchen. Aborting delivery.")
            self.navigate_to('home')
            return
        
        self.get_logger().info("✓ Reached kitchen successfully!")
        
        # Wait for kitchen confirmation with timeout
        self.get_logger().info(f"⏳ Waiting for kitchen confirmation (Timeout: {self.KITCHEN_TIMEOUT}s)...")
        kitchen_confirmed = self.wait_for_confirmation("KITCHEN", self.KITCHEN_TIMEOUT)
        
        if not kitchen_confirmed:
            # SCENARIO A: No confirmation from kitchen → Return home directly
            self.get_logger().warn("⚠ Kitchen confirmation timeout!")
            self.get_logger().info("📋 SCENARIO A: No food picked up → Returning HOME directly")
            self.navigate_to('home')
            self.get_logger().info("✓ Returned home after kitchen timeout.")
            return
        
        # Food was picked up from kitchen
        self.food_picked_up = True
        self.get_logger().info("✓ Kitchen confirmed food pickup!")
        self.get_logger().info("🍔 Food loaded on robot")
        time.sleep(1)  # Brief pause for pickup simulation
        
        # Step 2: Go to Table
        self.get_logger().info(f"Step 2: Moving to TABLE {table_number} for delivery...")
        if not self.navigate_to(table_number):
            # Failed to reach table with food
            self.get_logger().error(f"❌ Failed to reach Table {table_number}.")
            self.get_logger().info("📋 Food still on robot → Returning to KITCHEN first")
            self.navigate_to('kitchen')
            self.get_logger().info("✓ Returned food to kitchen")
            self.navigate_to('home')
            self.get_logger().info("✓ Returned home after navigation failure")
            return
        
        self.get_logger().info(f"✓ Reached Table {table_number} successfully!")
        
        # Step 3: Wait for table confirmation with timeout
        self.get_logger().info(f"⏳ Waiting for customer confirmation at Table {table_number} (Timeout: {self.TABLE_TIMEOUT}s)...")
        table_confirmed = self.wait_for_confirmation(f"TABLE {table_number}", self.TABLE_TIMEOUT)
        
        if not table_confirmed:
            # SCENARIO B: No confirmation from table → Return to kitchen first, then home
            self.get_logger().warn(f"⚠ Customer at Table {table_number} did not confirm!")
            self.get_logger().info("📋 SCENARIO B: Undelivered food → Returning to KITCHEN first")
            
            # Return food to kitchen
            if not self.navigate_to('kitchen'):
                self.get_logger().error("❌ Failed to return to kitchen")
            else:
                self.get_logger().info("✓ Returned undelivered food to kitchen")
            
            # Then return home
            self.navigate_to('home')
            self.get_logger().info("✓ Returned home after returning food to kitchen")
            return
        
        # Successful delivery
        self.get_logger().info(f"✓ Customer at Table {table_number} confirmed order received!")
        self.get_logger().info("🎉 Food successfully delivered!")
        time.sleep(1)  # Brief pause for delivery simulation
        
        # Step 4: Return Home (successful completion)
        self.get_logger().info("Step 3: Returning to HOME position...")
        if not self.navigate_to('home'):
            self.get_logger().error("❌ Failed to reach home position.")
            return
        
        self.get_logger().info("✓ Delivery complete! Robot returned home.")
        self.get_logger().info(f"{'='*60}\n")

    def wait_for_confirmation(self, location_name, timeout_seconds):
        """
        Wait for user confirmation with timeout
        Returns True if confirmed within timeout, False otherwise
        """
        confirmation_received = [False]
        timeout_flag = [False]
        
        def get_confirmation():
            """Thread function to get user input"""
            try:
                response = input(f"{location_name}: Confirm? (y/n): ").strip().lower()
                if response == 'y':
                    confirmation_received[0] = True
            except EOFError:
                pass
        
        def timeout_timer():
            """Thread function to track timeout"""
            time.sleep(timeout_seconds)
            timeout_flag[0] = True
        
        # Start confirmation thread
        confirm_thread = threading.Thread(target=get_confirmation, daemon=True)
        confirm_thread.start()
        
        # Start timeout timer
        timer_thread = threading.Thread(target=timeout_timer, daemon=True)
        timer_thread.start()
        
        # Wait for either confirmation or timeout
        start_time = time.time()
        while not confirmation_received[0] and not timeout_flag[0]:
            elapsed = int(time.time() - start_time)
            remaining = timeout_seconds - elapsed
            
            if remaining > 0 and elapsed % 5 == 0 and elapsed > 0:  # Log every 5 seconds
                self.get_logger().info(f"⏰ Waiting... {remaining}s remaining")
            
            time.sleep(0.5)
        
        return confirmation_received[0]

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
    robot = Task3DeliveryRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()