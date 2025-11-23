#!/usr/bin/env python3
"""
Task 8 - Delivery Manager
This ROS2 node unifies Tasks 1-7 into a single delivery manager suitable for
a cafe/butler robot. It supports:
 - single and multi-table orders
 - kitchen and table confirmations with timeouts
 - skipping unconfirmed tables (multi-table)
 - returning unconfirmed/canceled orders to kitchen at the end
 - dynamic order cancellation (per-table)
 - mission-level cancellation with phase-aware behavior:
     * canceled while going to kitchen -> return home
     * canceled while going to table   -> return to kitchen, then home
 - interactive commands via the console (non-blocking)
Implementation notes:
 - Uses threading for interactive input (orders, cancellations, confirmations)
 - Uses nav2 NavigateToPose action
 - Preserves the predefined positions for home, kitchen, and tables
"""

import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
import math

def quaternion_from_yaw(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

class DeliveryManager(Node):
    def __init__(self):
        super().__init__('task8_delivery_manager')

        # Predefined positions (do not change)
        self.poses = {
            'home': [-0.15, -0.26, 0.0],
            'kitchen': [4.20, -5.26, 1.57],
            1: [-0.47, -3.16, 0.0],
            2: [2.94, 2.74, -1.57],
            3: [5.58, -0.26, 3.14]
        }

        # Timeouts (seconds) - configurable here
        self.KITCHEN_TIMEOUT = 30
        self.TABLE_TIMEOUT = 20

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Action state
        self.goal_handle = None
        self.result_future = None
        self.action_done_event = threading.Event()
        self.action_lock = threading.Lock()

        # Order management (thread-safe)
        self.state_lock = threading.Lock()
        self.original_order = []    # original list provided by user
        self.pending_tables = []    # tables still to visit
        self.delivered_tables = []  # tables delivered/confirmed
        self.unconfirmed_tables = []# tables visited but unconfirmed (to return to kitchen later)
        self.canceled_tables = []   # tables canceled by user (dynamic)
        self.mission_canceled = False   # mission-level cancel flag ('c')
        self.phase = 'idle'         # 'idle', 'going_to_kitchen', 'going_to_table', 'returning_to_kitchen', 'returning_home'

        # Input/listener threads
        self.order_thread = threading.Thread(target=self.order_input_loop, daemon=True)
        self.command_thread = threading.Thread(target=self.command_input_loop, daemon=True)

        # Start threads
        self.order_thread.start()
        self.command_thread.start()

        self.get_logger().info("Task8 Delivery Manager started. Waiting for orders...")

    #
    # Main interactive loops
    #
    def order_input_loop(self):
        """Main loop to accept new orders from the console."""
        while rclpy.ok():
            try:
                raw = input("\nEnter table numbers (e.g. '1' or '1 2 3') or 'q' to quit: ").strip()
            except EOFError:
                # In some launch environments input might be EOF - exit loop
                break

            if not raw:
                continue
            if raw.lower() == 'q':
                self.get_logger().info("Shutting down delivery manager (user requested quit).")
                rclpy.shutdown()
                break

            # parse numbers
            parts = [p for p in raw.split() if p.isdigit()]
            tables = []
            for p in parts:
                n = int(p)
                if n in self.poses and n not in tables:
                    tables.append(n)
                else:
                    self.get_logger().warn(f"Ignored invalid/duplicate table: {p}")

            if not tables:
                self.get_logger().warn("No valid tables found. Valid table ids: 1,2,3")
                continue

            # start a mission if idle
            with self.state_lock:
                if self.phase != 'idle':
                    self.get_logger().warn("A mission is already running. Please wait until it finishes or cancel it.")
                    continue
                # initialize structures
                self.original_order = tables.copy()
                self.pending_tables = tables.copy()
                self.delivered_tables = []
                self.unconfirmed_tables = []
                self.canceled_tables = []
                self.mission_canceled = False

            # launch mission in a new thread so input loop remains free
            mission_thread = threading.Thread(target=self.execute_mission, daemon=True)
            mission_thread.start()

    def command_input_loop(self):
        """
        Listen for runtime commands while a mission may be active.
        Commands:
          - 'c' : global cancel => behavior depends on phase
          - 'cancel N ...' or just 'N ...' : cancel specific table(s) (dynamic cancellation)
          - 'status' : print current mission status
        """
        while rclpy.ok():
            try:
                cmd = input().strip()
            except EOFError:
                break
            if not cmd:
                continue
            cmd_lower = cmd.lower()
            if cmd_lower == 'c':
                # global cancellation
                with self.state_lock:
                    if self.phase == 'idle':
                        self.get_logger().info("No active mission to cancel.")
                        continue
                    self.get_logger().warn("Global mission cancellation requested by user.")
                    self.mission_canceled = True
                # cancel current nav goal if any
                with self.action_lock:
                    if self.goal_handle:
                        self.cancel_navigation_goal()
                    else:
                        self.get_logger().info("No active goal to cancel.")
                continue

            if cmd_lower.startswith('cancel'):
                # format: cancel 2 3
                parts = cmd.split()
                nums = [int(p) for p in parts[1:] if p.isdigit()]
                self._cancel_tables(nums)
                continue

            # allow just typing table numbers to cancel
            parts = [p for p in cmd.split() if p.isdigit()]
            nums = [int(p) for p in parts]
            if nums:
                self._cancel_tables(nums)
                continue

            if cmd_lower == 'status':
                self.print_status()
                continue

            self.get_logger().info("Unknown command. Use: 'c' (cancel mission), 'cancel N [M...]' (cancel tables), 'status'")

    def _cancel_tables(self, tables):
        """Mark tables as canceled (dynamic cancellation)."""
        with self.state_lock:
            actually_canceled = []
            for t in tables:
                if t in self.pending_tables:
                    self.pending_tables.remove(t)
                    if t not in self.canceled_tables:
                        self.canceled_tables.append(t)
                    actually_canceled.append(t)
                elif t in self.delivered_tables:
                    self.get_logger().warn(f"Table {t} already delivered; cannot cancel.")
                elif t in self.canceled_tables:
                    self.get_logger().warn(f"Table {t} was already canceled.")
                else:
                    self.get_logger().warn(f"Table {t} not in current mission.")
            if actually_canceled:
                self.get_logger().info(f"Canceled tables: {actually_canceled}. Pending now: {self.pending_tables}")

    def print_status(self):
        with self.state_lock:
            self.get_logger().info(f"Phase: {self.phase}")
            self.get_logger().info(f"Original order: {self.original_order}")
            self.get_logger().info(f"Pending: {self.pending_tables}")
            self.get_logger().info(f"Delivered: {self.delivered_tables}")
            self.get_logger().info(f"Unconfirmed (to return): {self.unconfirmed_tables}")
            self.get_logger().info(f"Canceled: {self.canceled_tables}")
            self.get_logger().info(f"Mission canceled flag: {self.mission_canceled}")

    #
    # Mission Execution
    #
    def execute_mission(self):
        """
        Top-level mission logic that merges behavior from tasks 1..7:
         - go to kitchen, wait for kitchen confirmation (timeout)
         - if kitchen not confirmed -> return home (Task2,3)
         - if confirmed -> iterate pending tables:
              * navigate to table
              * if navigation canceled while en route -> handle Task4 behavior
              * at table, wait for confirmation (timeout)
                - if confirmed -> delivered
                - if not confirmed:
                    - single table mission -> return to kitchen then home (Task3)
                    - multi-table mission -> skip and mark unconfirmed; continue to next (Task6)
         - after finishing visiting all tables:
              - if any unconfirmed/canceled -> return to kitchen to return those orders
         - always return home at the end
        """
        with self.state_lock:
            self.phase = 'starting_mission'
            original = self.original_order.copy()
            pending = self.pending_tables.copy()
        self.get_logger().info(f"Mission started for tables: {original}")

        # Step 1: Go to kitchen
        with self.state_lock:
            if self.mission_canceled:
                self.get_logger().warn("Mission canceled before starting navigation to kitchen. Returning home (idle).")
                self.phase = 'returning_home'
                self.navigate_to('home')
                self.phase = 'idle'
                return
            self.phase = 'going_to_kitchen'

        self.get_logger().info("Heading to kitchen...")
        nav_ok = self.navigate_to('kitchen')

        # If cancel requested while moving to kitchen -> follow Task4: return home
        with self.state_lock:
            if self.mission_canceled and self.phase == 'going_to_kitchen':
                self.get_logger().warn("Mission canceled while going to kitchen -> returning home.")
                self.phase = 'returning_home'
                self.navigate_to('home')
                self.phase = 'idle'
                return

        if not nav_ok:
            self.get_logger().error("Failed to reach kitchen. Returning home.")
            self.phase = 'returning_home'
            self.navigate_to('home')
            self.phase = 'idle'
            return

        # Step 2: Wait for kitchen confirmation
        self.get_logger().info("Waiting for kitchen confirmation...")
        kitchen_confirmed = self.wait_for_confirmation("KITCHEN", self.KITCHEN_TIMEOUT)

        if not kitchen_confirmed:
            # Task2/3 behavior: no confirmation at kitchen -> return home directly
            self.get_logger().warn("Kitchen confirmation timeout. Returning home.")
            self.phase = 'returning_home'
            self.navigate_to('home')
            self.phase = 'idle'
            return

        self.get_logger().info("Kitchen confirmed. Food picked up.")
        time.sleep(1.0)

        # Step 3: Deliver to pending tables
        while True:
            with self.state_lock:
                if self.mission_canceled:
                    # Mission-level cancel while delivering -> behavior per Task4:
                    # If canceled while going to table, return to kitchen then home.
                    # Here we handle it by returning to kitchen then home.
                    self.get_logger().warn("Mission-level cancel detected during deliveries.")
                    # Cancel nav goal if in progress
                    with self.action_lock:
                        if self.goal_handle:
                            self.cancel_navigation_goal()
                    self.phase = 'returning_to_kitchen_after_cancel'
                    self.navigate_to('kitchen')
                    self.phase = 'returning_home_after_cancel'
                    self.navigate_to('home')
                    self.phase = 'idle'
                    return

                if not self.pending_tables:
                    break
                # pop next table in order
                table = self.pending_tables.pop(0)

                # skip if dynamically canceled before reaching it
                if table in self.canceled_tables:
                    self.get_logger().info(f"Table {table} was canceled before visiting; skipping.")
                    continue

                # prepare to go to table
                self.phase = 'going_to_table'
            self.get_logger().info(f"Heading to Table {table} ...")
            nav_ok = self.navigate_to(table)

            # if mission_canceled while going -> follows Task4
            with self.state_lock:
                if self.mission_canceled and self.phase == 'going_to_table':
                    self.get_logger().warn(f"Mission canceled while going to table {table}. Returning to kitchen then home.")
                    # ensure current goal canceled
                    with self.action_lock:
                        if self.goal_handle:
                            self.cancel_navigation_goal()
                    self.phase = 'returning_to_kitchen_after_cancel'
                    self.navigate_to('kitchen')
                    self.phase = 'returning_home_after_cancel'
                    self.navigate_to('home')
                    self.phase = 'idle'
                    return

            if not nav_ok:
                # navigation failure (could be canceled or failed). Treat as failed delivery.
                self.get_logger().error(f"Failed to reach Table {table}. Marking as unconfirmed and continuing.")
                with self.state_lock:
                    if table not in self.unconfirmed_tables:
                        self.unconfirmed_tables.append(table)
                continue

            # Arrived at table. Check if it was canceled during travel (edge-case)
            with self.state_lock:
                if table in self.canceled_tables:
                    self.get_logger().warn(f"Table {table} was canceled during travel; skipping delivery.")
                    continue

            # Wait for table confirmation
            self.get_logger().info(f"Waiting for confirmation at Table {table} ...")
            table_confirmed = self.wait_for_confirmation(f"TABLE {table}", self.TABLE_TIMEOUT)

            # Behavior depends on single-vs-multi order
            with self.state_lock:
                multi_mode = (len(original) > 1)

            if table_confirmed:
                self.get_logger().info(f"Table {table} confirmed. Delivery complete.")
                with self.state_lock:
                    self.delivered_tables.append(table)
                time.sleep(0.7)
            else:
                self.get_logger().warn(f"No confirmation at Table {table}.")
                if not multi_mode:
                    # single-table scenario -> Task3: return to kitchen then home
                    self.get_logger().info("Single-order mission and table not confirmed -> returning to kitchen then home.")
                    with self.state_lock:
                        if table not in self.unconfirmed_tables:
                            self.unconfirmed_tables.append(table)
                    self.phase = 'returning_to_kitchen_after_table_timeout'
                    self.navigate_to('kitchen')
                    self.phase = 'returning_home'
                    self.navigate_to('home')
                    self.phase = 'idle'
                    return
                else:
                    # multi-table: skip and continue; mark for return to kitchen at the end
                    self.get_logger().info(f"Multi-order mission: skipping Table {table} and continuing to next tables.")
                    with self.state_lock:
                        if table not in self.unconfirmed_tables:
                            self.unconfirmed_tables.append(table)
                    continue

        # Done visiting all tables
        self.get_logger().info("Visited all pending tables.")
        # Step 4: If there are unconfirmed or canceled orders, go to kitchen to return them
        with self.state_lock:
            to_return = sorted(list(set(self.unconfirmed_tables + self.canceled_tables)))
        if to_return:
            self.get_logger().info(f"Returning unconfirmed/canceled orders to kitchen for tables: {to_return}")
        else:
            self.get_logger().info("All orders handled. Returning to kitchen (standard procedure).")

        self.phase = 'returning_to_kitchen'
        self.navigate_to('kitchen')

        # (Optional) simulate handover at kitchen
        if to_return:
            self.get_logger().info("Returned items handed back to kitchen staff.")
            time.sleep(1.0)

        # Step 5: Return home
        self.phase = 'returning_home'
        self.navigate_to('home')

        # Summarize mission
        with self.state_lock:
            self.get_logger().info("Mission Summary:")
            self.get_logger().info(f"  Original Order: {original}")
            self.get_logger().info(f"  Delivered: {self.delivered_tables}")
            self.get_logger().info(f"  Unconfirmed (returned): {self.unconfirmed_tables}")
            self.get_logger().info(f"  Canceled: {self.canceled_tables}")
            self.phase = 'idle'

        self.get_logger().info("Mission complete. Ready for next order.")

    #
    # Navigation helpers
    #
    def navigate_to(self, pose_key):
        """
        Sends a NavigateToPose goal and waits for completion.
        Returns True on success, False otherwise.
        This method handles cancel requests initiated externally by command_input_loop.
        """
        if pose_key not in self.poses:
            self.get_logger().error(f"Unknown pose_key: {pose_key}")
            return False

        pose = self.poses[pose_key]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.orientation = quaternion_from_yaw(pose[2])

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Wait for server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return False

        # send goal
        with self.action_lock:
            send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback_callback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()

            if not self.goal_handle.accepted:
                self.get_logger().error("Goal was rejected by server.")
                self.goal_handle = None
                return False

            # prepare to wait for result
            self.action_done_event.clear()
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self._nav_result_callback)

        # wait for completion or external cancellation requests
        while not self.action_done_event.is_set():
            # if mission-level cancel is set, cancel goal here (caller decides behavior)
            with self.state_lock:
                if self.mission_canceled:
                    # send cancel
                    with self.action_lock:
                        if self.goal_handle:
                            self.get_logger().info("Cancelling current goal (mission cancel).")
                            self.cancel_navigation_goal()
                    # break and return False to caller to handle next steps
                    break
            time.sleep(0.05)

        # get status
        with self.action_lock:
            if not self.goal_handle:
                return False
            status = self.goal_handle.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False

    def cancel_navigation_goal(self):
        """Cancel the current navigation goal (non-blocking)."""
        if not self.goal_handle:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        # block briefly until cancel sent
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        self.get_logger().info("Cancel request sent to navigation action server.")

    def _nav_result_callback(self, future):
        """Internal callback when navigation action finishes."""
        res = future.result()
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation goal succeeded.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Navigation goal was canceled.")
        else:
            self.get_logger().warn(f"Navigation goal finished with status: {status}")
        # clear current goal handle
        with self.action_lock:
            self.goal_handle = None
        self.action_done_event.set()

    def nav_feedback_callback(self, feedback_msg):
        """Show periodic distance feedback (throttled by logger)."""
        try:
            distance = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f"Distance remaining: {distance:.2f} m")
        except Exception:
            pass

    #
    # Confirmation helper
    #
    def wait_for_confirmation(self, location_name, timeout_seconds):
        """
        Wait for a 'y' confirmation input from console for up to timeout_seconds.
        Non-blocking outside; runs in same thread as mission but will wait on input thread which may conflict.
        Implementation uses a separate input thread that reads a single line; however to avoid multiple simultaneous
        console reads we implement a prompt that must be answered in the main console.
        Returns True if user confirmed ('y'), False otherwise (timeout or other).
        """
        self.get_logger().info(f"{location_name}: Please confirm by typing 'y' (you have {timeout_seconds}s).")
        confirmed = [False]
        input_done = [False]

        def ask():
            try:
                resp = input(f"{location_name}: Confirm? (y/n): ").strip().lower()
                if resp == 'y':
                    confirmed[0] = True
            except EOFError:
                pass
            input_done[0] = True

        t = threading.Thread(target=ask, daemon=True)
        t.start()

        start = time.time()
        while time.time() - start < timeout_seconds:
            if input_done[0]:
                break
            time.sleep(0.2)

        # If not answered, we consider it timeout; attempt to consume any leftover input to avoid interference
        if not input_done[0]:
            self.get_logger().info(f"{location_name}: confirmation timed out after {timeout_seconds}s.")
            return False

        return confirmed[0]

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (KeyboardInterrupt).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()