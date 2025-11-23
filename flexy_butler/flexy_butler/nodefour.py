#! /usr/bin/env python3
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
import time
import threading

class MoveRobot(Node):
    def __init__(self):
        super().__init__('MoveRobot')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.canceled_tables = []
        self.goal_completed = False
        self.goal_success = False
        self.run_loop()

    def run_loop(self):
        self.canceled_during_kitchen = False
        self.canceled_during_table = []
        while rclpy.ok():
            input_str = input('\n### USER ####\nDo you want to order anything (Y/N): ')
            if input_str.lower() != 'y':
                continue

            order = list(map(int, input('\n### USER ####\nEnter the Table Numbers: ').split()))
            print(f'Order placed for Tables {order}')

            # Ask cancel while heading to kitchen
            cancel_flag = threading.Event()
            def cancel_listener():
                while not self.goal_completed and not cancel_flag.is_set():
                    c = input("Do you want to cancel the order while going to kitchen? (y/n): ").strip().lower()
                    if c == 'y':
                        cancel_flag.set()
            threading.Thread(target=cancel_listener, daemon=True).start()

            reached_kitchen = self.move_to_kitchen()
            if cancel_flag.is_set():
                print("Order canceled before reaching kitchen. Returning home.")
                self.move_to_home()
                continue

            if not reached_kitchen:
                print("Failed to reach kitchen.")
                self.move_to_home()
                continue

            # Confirmation at kitchen with timeout
            print("Waiting for kitchen confirmation for 15 seconds...")
            confirmed = self.await_confirmation("Kitchen: Confirm food pickup? (y/n): ", timeout=15)
            if not confirmed:
                print("No confirmation from kitchen. Returning home.")
                self.move_to_home()
                continue

            # Ask if any table orders are to be canceled before moving to tables
            if input("Cancel any table orders now? (y/n): ").lower() == 'y':
                self.canceled_tables = list(map(int, input("Enter table numbers to cancel: ").split()))
            else:
                self.canceled_tables = []

            final_order = [t for t in order if t not in self.canceled_tables]

            for table in final_order:
                cancel_table_flag = threading.Event()
                def cancel_table_listener():
                    while not self.goal_completed and not cancel_table_flag.is_set():
                        c = input(f"Do you want to cancel the order while going to Table {table}? (y/n): ").strip().lower()
                        if c == 'y':
                            cancel_table_flag.set()
                threading.Thread(target=cancel_table_listener, daemon=True).start()

                reached = self.move_to_table(table)
                if cancel_table_flag.is_set():
                    print(f"Order to table {table} canceled before reaching. Returning to kitchen and then home.")
                    self.move_to_kitchen()
                    self.move_to_home()
                    break

                if not self.goal_success:
                    print(f"Failed to reach Table {table}.")
                    break

                # Confirmation from customer at table
                confirmed = self.await_confirmation(f"Table {table}: Confirm order received? (y/n): ", timeout=10)
                if not confirmed:
                    print(f"No confirmation at table {table}. Returning to kitchen then home.")
                    self.move_to_kitchen()
                    self.move_to_home()
                    break
                else:
                    print(f"Order delivered to Table {table}.")

            print("Returning home.")
            self.move_to_home()

    def await_confirmation(self, prompt="Confirm? (y/n): ", timeout=15):
        result = {'confirmed': False}
        def ask():
            try:
                res = input(prompt).strip().lower()
                if res == 'y':
                    result['confirmed'] = True
            except:
                pass
        thread = threading.Thread(target=ask)
        thread.daemon = True
        thread.start()
        thread.join(timeout)
        return result['confirmed']

    def move_to_position(self, x, y):
        self.goal_completed = False
        self.goal_success = False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = self.create_quaternion_identity()
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._wait_for_result)
        while not self.goal_completed and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.goal_success

    def create_quaternion_identity(self):
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    def _wait_for_result(self, result_future):
        result = result_future.result()
        self.goal_completed = True
        self.goal_success = result.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info('Goal succeeded!' if self.goal_success else f'Goal failed with status: {result.status}')

    def feedback_callback(self, msg):
        pass

    def move_to_kitchen(self):
        self.get_logger().info("Moving to Kitchen")
        return self.move_to_position(4.20, -5.26)

    def move_to_table(self, table_id):
        self.get_logger().info(f"Moving to Table {table_id}")
        positions = {
            1: (-0.47, -3.16),
            2: (2.94, 2.74),
            3: (5.58, -0.26)
        }
        if table_id in positions:
            return self.move_to_position(*positions[table_id])
        else:
            self.get_logger().info(f"Unknown table: {table_id}")
            return False

    def move_to_home(self):
        self.get_logger().info("Moving to Home")
        return self.move_to_position(-0.15, -0.26)

def main(args=None):
    rclpy.init(args=args)
    robot = MoveRobot()
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
