#! /usr/bin/env python3
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
import math

# Helper function to create a quaternion from yaw
def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        
        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('home_pose', [-0.15, -0.26, 0.0]),
                ('kitchen_pose', [4.20, -5.26, 1.57]),
                ('table_poses.1', [-0.47, -3.16, 0.0]),
                ('table_poses.2', [2.94, 2.74, -1.57]),
                ('table_poses.3', [5.58, -0.26, 3.14])
            ])
        
        # Store poses from parameters
        self.poses = {
            'home': self.get_parameter('home_pose').value,
            'kitchen': self.get_parameter('kitchen_pose').value,
            1: self.get_parameter('table_poses.1').value,
            2: self.get_parameter('table_poses.2').value,
            3: self.get_parameter('table_poses.3').value,
        }

        # --- Action Client ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # --- State & Threading ---
        self.goal_handle = None
        self.result_future = None
        self.action_done_event = threading.Event()

        # --- Start the main loop in a separate thread ---
        self.logic_thread = threading.Thread(target=self.main_loop, daemon=True)
        self.logic_thread.start()

    def main_loop(self):
        """The main control loop for the robot's logic."""
        while rclpy.ok():
            try:
                # --- Get Order ---
                if not self.prompt_user_bool("\n### USER ###\nDo you want to place an order? (y/n): "):
                    time.sleep(1)
                    continue

                order_str = self.prompt_user_input('\n### USER ###\nEnter the Table Numbers (space-separated): ')
                if not order_str: continue
                
                order = [int(t) for t in order_str.split() if t.isdigit() and int(t) in self.poses]
                if not order:
                    self.get_logger().warn("No valid tables entered. Please try again.")
                    continue
                
                self.get_logger().info(f"Order placed for Tables: {order}")

                # --- Go to Kitchen ---
                self.get_logger().info("Heading to the kitchen...")
                status = self.go_to_pose('kitchen', "Cancel while going to kitchen? (y/n): ")
                
                if status == 'CANCELED':
                    self.get_logger().info("Order canceled. Returning home.")
                    self.go_to_pose('home')
                    continue
                if status != 'SUCCEEDED':
                    self.get_logger().error("Failed to reach kitchen. Returning home.")
                    self.go_to_pose('home')
                    continue

                # --- Kitchen Confirmation ---
                if not self.prompt_user_bool("Kitchen: Confirm food pickup? (y/n): ", timeout=15):
                    self.get_logger().warn("No confirmation from kitchen. Returning home.")
                    self.go_to_pose('home')
                    continue
                
                # --- Pre-delivery Cancellation ---
                final_order = order.copy()
                if self.prompt_user_bool("Cancel any table orders now? (y/n): "):
                    cancel_str = self.prompt_user_input("Enter table numbers to cancel: ")
                    tables_to_cancel = [int(t) for t in cancel_str.split() if t.isdigit()]
                    final_order = [t for t in order if t not in tables_to_cancel]

                if not final_order:
                    self.get_logger().info("All tables canceled. Returning food to kitchen.")
                    self.go_to_pose('kitchen')
                    self.go_to_pose('home')
                    continue

                # --- Deliver to Tables ---
                unconfirmed_tables = []
                for table_num in final_order:
                    self.get_logger().info(f"Delivering to Table {table_num}...")
                    status = self.go_to_pose(table_num, f"Cancel order for Table {table_num}? (y/n): ")
                    
                    if status == 'CANCELED' or status != 'SUCCEEDED':
                        self.get_logger().error(f"Delivery to Table {table_num} failed or was canceled. Aborting all remaining deliveries.")
                        self.go_to_pose('kitchen')
                        # Mark remaining tables as unconfirmed to ensure a trip to the kitchen at the end.
                        unconfirmed_tables.append(table_num)
                        break

                    # --- Customer Confirmation ---
                    if self.prompt_user_bool(f"Table {table_num}: Confirm order received? (y/n): ", timeout=10):
                        self.get_logger().info(f"Order delivered and confirmed at Table {table_num}.")
                    else:
                        self.get_logger().warn(f"No confirmation at Table {table_num}. Will continue delivery and return this order later.")
                        unconfirmed_tables.append(table_num)
                
                # --- Final Return Sequence ---
                # If any orders were unconfirmed, go to the kitchen first.
                if unconfirmed_tables:
                    self.get_logger().info(f"Returning unconfirmed items for tables {unconfirmed_tables} to the kitchen.")
                    self.go_to_pose('kitchen')
                
                # Finally, always return home.
                self.get_logger().info("All tasks complete. Returning home.")
                self.go_to_pose('home')

            except Exception as e:
                self.get_logger().error(f"An error occurred in the main loop: {e}")

    def go_to_pose(self, pose_key, cancel_prompt=None):
        """Navigates to a pose, returns status: SUCCEEDED, CANCELED, or FAILED."""
        if pose_key not in self.poses:
            self.get_logger().error(f"Unknown pose key: {pose_key}")
            return 'FAILED'
        
        pose = self.poses[pose_key]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.orientation = quaternion_from_yaw(pose[2])
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal for '{pose_key}'...")
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return 'FAILED'

        self.action_done_event.clear()
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

        cancel_requested = threading.Event()
        if cancel_prompt:
            cancel_thread = threading.Thread(target=self.cancellation_listener, args=(cancel_prompt, cancel_requested))
            cancel_thread.daemon = True
            cancel_thread.start()

        while not self.action_done_event.is_set():
            if cancel_requested.is_set():
                self.get_logger().info("Cancellation requested by user.")
                self.cancel_goal()
                break
            time.sleep(0.1)
        
        status = self.goal_handle.status if self.goal_handle else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED'
        else:
            return 'FAILED'

    def get_result_callback(self, future):
        """Callback for when the action is done."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
        self.action_done_event.set()

    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters.', throttle_duration_sec=2)

    def cancel_goal(self):
        """Cancel the current navigation goal."""
        if self.goal_handle:
            self.get_logger().info('Canceling the goal...')
            self.goal_handle.cancel_goal_async()

    def cancellation_listener(self, prompt, event):
        """Thread target to listen for user cancellation."""
        self.prompt_user_bool(prompt, event_to_set=event)

    def prompt_user_bool(self, prompt, timeout=None, event_to_set=None):
        """Prompts user for a y/n response with an optional timeout."""
        response = self.prompt_user_input(prompt, timeout)
        if response.lower() == 'y':
            if event_to_set:
                event_to_set.set()
            return True
        return False
    
    def prompt_user_input(self, prompt, timeout=None):
        """Gets user input with a timeout."""
        result_container = ['']
        def ask():
            try:
                result_container[0] = input(prompt)
            except EOFError:
                pass
        
        thread = threading.Thread(target=ask, daemon=True)
        thread.start()
        thread.join(timeout)
        return result_container[0].strip()

def main(args=None):
    rclpy.init(args=args)
    robot_mover = MoveRobot()
    rclpy.spin(robot_mover)
    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()