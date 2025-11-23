#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from nav2_msgs.srv import ClearEntireCostmap
import math
import time
from threading import Lock

class StuckRecoveryMonitor(Node):
    def __init__(self):
        super().__init__('stuck_recovery_monitor')
        
        # Parameters
        self.declare_parameter('stuck_timeout', 15.0)  # seconds
        self.declare_parameter('min_movement_threshold', 0.05)  # meters
        self.declare_parameter('recovery_linear_vel', 0.08)  # m/s
        self.declare_parameter('recovery_angular_vel', 0.5)  # rad/s
        self.declare_parameter('max_recovery_attempts', 3)
        
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.min_movement = self.get_parameter('min_movement_threshold').value
        self.recovery_linear_vel = self.get_parameter('recovery_linear_vel').value
        self.recovery_angular_vel = self.get_parameter('recovery_angular_vel').value
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value
        
        # State variables
        self.last_pose = None
        self.last_movement_time = time.time()
        self.recovery_attempts = 0
        self.in_recovery = False
        self.lock = Lock()
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Service clients for clearing costmaps - using correct service names
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Alternative service clients (fallback)
        self.clear_local_costmap_alt = self.create_client(
            Empty, 
            '/local_costmap/clear_around_global_robot'
        )
        self.clear_global_costmap_alt = self.create_client(
            Empty, 
            '/global_costmap/clear_around_global_robot'
        )
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_stuck_state)
        
        # Wait for services at startup
        self.get_logger().info("Waiting for costmap clearing services...")
        self.wait_for_services()
        
        self.get_logger().info("Stuck Recovery Monitor initialized and ready")
    
    def wait_for_services(self):
        """Wait for costmap clearing services to become available"""
        services_ready = False
        timeout = 10.0  # 10 second timeout
        
        start_time = time.time()
        while not services_ready and (time.time() - start_time) < timeout:
            local_ready = (self.clear_local_costmap.wait_for_service(timeout_sec=1.0) or 
                          self.clear_local_costmap_alt.wait_for_service(timeout_sec=1.0))
            global_ready = (self.clear_global_costmap.wait_for_service(timeout_sec=1.0) or 
                           self.clear_global_costmap_alt.wait_for_service(timeout_sec=1.0))
            
            services_ready = local_ready and global_ready
            
            if not services_ready:
                self.get_logger().warn("Waiting for costmap services to become available...")
                time.sleep(1.0)
        
        if services_ready:
            self.get_logger().info("Costmap clearing services are available")
        else:
            self.get_logger().error("Timeout waiting for costmap services. Some recovery features may not work.")
    
    def odom_callback(self, msg):
        with self.lock:
            current_pose = msg.pose.pose.position
            current_time = time.time()
            
            if self.last_pose is not None:
                # Calculate movement distance
                dx = current_pose.x - self.last_pose.x
                dy = current_pose.y - self.last_pose.y
                distance_moved = math.sqrt(dx*dx + dy*dy)
                
                # Check if robot has moved significantly
                if distance_moved > self.min_movement:
                    self.last_movement_time = current_time
                    if self.in_recovery:
                        self.get_logger().info("Robot is moving again, stopping recovery")
                        self.in_recovery = False
                        self.recovery_attempts = 0
            
            self.last_pose = current_pose
    
    def monitor_stuck_state(self):
        if self.in_recovery:
            return
            
        current_time = time.time()
        time_since_movement = current_time - self.last_movement_time
        
        if time_since_movement > self.stuck_timeout and self.recovery_attempts < self.max_recovery_attempts:
            self.get_logger().warn(f"Robot appears stuck for {time_since_movement:.1f}s. Starting recovery #{self.recovery_attempts + 1}")
            self.start_recovery()
    
    def start_recovery(self):
        with self.lock:
            self.in_recovery = True
            self.recovery_attempts += 1
        
        # Clear costmaps first
        self.clear_costmaps()
        
        # Brief wait after clearing costmaps
        time.sleep(0.5)
        
        # Perform recovery maneuver based on attempt number
        if self.recovery_attempts == 1:
            self.recovery_spin()
        elif self.recovery_attempts == 2:
            self.recovery_backup_and_spin()
        else:
            self.recovery_aggressive_maneuver()
    
    def clear_costmaps(self):
        """Clear both local and global costmaps using multiple methods"""
        local_cleared = False
        global_cleared = False
        
        try:
            # Try primary method first (ClearEntireCostmap service)
            if self.clear_local_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_local_costmap.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None:
                    self.get_logger().info("Cleared local costmap (primary method)")
                    local_cleared = True
                else:
                    self.get_logger().warn("Primary local costmap clear failed")
            
            # Try alternative method if primary failed
            if not local_cleared and self.clear_local_costmap_alt.service_is_ready():
                req = Empty.Request()
                future = self.clear_local_costmap_alt.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None:
                    self.get_logger().info("Cleared local costmap (alternative method)")
                    local_cleared = True
            
            # Clear global costmap
            if self.clear_global_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_global_costmap.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None:
                    self.get_logger().info("Cleared global costmap (primary method)")
                    global_cleared = True
                else:
                    self.get_logger().warn("Primary global costmap clear failed")
            
            # Try alternative method if primary failed
            if not global_cleared and self.clear_global_costmap_alt.service_is_ready():
                req = Empty.Request()
                future = self.clear_global_costmap_alt.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None:
                    self.get_logger().info("Cleared global costmap (alternative method)")
                    global_cleared = True
            
            if not local_cleared:
                self.get_logger().error("Failed to clear local costmap")
            if not global_cleared:
                self.get_logger().error("Failed to clear global costmap")
                
        except Exception as e:
            self.get_logger().error(f"Exception while clearing costmaps: {e}")
    
    def recovery_spin(self):
        """Simple spin recovery"""
        self.get_logger().info("Executing spin recovery")
        twist = Twist()
        twist.angular.z = self.recovery_angular_vel
        
        # Spin for 3 seconds with small steps
        steps = 30  # 0.1 second steps
        for i in range(steps):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Wait and check if recovery worked
        self.create_timer(2.0, self.check_recovery_success)
    
    def recovery_backup_and_spin(self):
        """Backup then spin recovery"""
        self.get_logger().info("Executing backup and spin recovery")
        twist = Twist()
        
        # Backup for 2 seconds
        twist.linear.x = -self.recovery_linear_vel
        for i in range(20):  # 0.1 second steps
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
        
        # Spin for 4 seconds
        twist.angular.z = self.recovery_angular_vel
        for i in range(40):  # 0.1 second steps
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.create_timer(2.0, self.check_recovery_success)
    
    def recovery_aggressive_maneuver(self):
        """More aggressive recovery with multiple maneuvers"""
        self.get_logger().info("Executing aggressive recovery maneuver")
        twist = Twist()
        
        # Sequence: backup, spin left, forward, spin right, backup
        maneuvers = [
            (-self.recovery_linear_vel, 0.0, 2.0),  # backup
            (0.0, self.recovery_angular_vel, 2.0),   # spin left
            (self.recovery_linear_vel * 0.5, 0.0, 1.0),  # forward slow
            (0.0, -self.recovery_angular_vel, 2.0), # spin right
            (-self.recovery_linear_vel * 0.5, 0.0, 1.0), # backup slow
        ]
        
        for linear_vel, angular_vel, duration in maneuvers:
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            
            steps = int(duration * 10)  # 0.1 second steps
            for i in range(steps):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # Brief pause between maneuvers
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)
        
        self.create_timer(3.0, self.check_recovery_success)
    
    def check_recovery_success(self):
        """Check if recovery was successful and reset if needed"""
        current_time = time.time()
        time_since_movement = current_time - self.last_movement_time
        
        if time_since_movement < 5.0:  # Robot moved within last 5 seconds
            self.get_logger().info("Recovery successful!")
            with self.lock:
                self.in_recovery = False
                self.recovery_attempts = 0
        else:
            self.get_logger().warn("Recovery may not have been successful")
            with self.lock:
                self.in_recovery = False
            
            # If we've exhausted all attempts, reset counter after a longer wait
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.get_logger().error("All recovery attempts exhausted. Waiting before reset...")
                self.create_timer(30.0, self.reset_recovery_attempts)
    
    def reset_recovery_attempts(self):
        """Reset recovery attempts after extended wait"""
        with self.lock:
            self.recovery_attempts = 0
        self.get_logger().info("Recovery attempts reset. Monitoring resumed.")

def main(args=None):
    rclpy.init(args=args)
    node = StuckRecoveryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()