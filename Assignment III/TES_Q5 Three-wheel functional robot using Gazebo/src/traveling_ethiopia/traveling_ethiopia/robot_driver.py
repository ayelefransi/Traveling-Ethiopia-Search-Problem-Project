"""
Robot driver that navigates to waypoints published by the path planner.
Subscribes to waypoints and uses simple proportional control to reach them.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PoseStamped, 'current_waypoint', self.waypoint_callback, 10
        )
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Navigation state
        self.target_x = None
        self.target_y = None
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.position_tolerance = 0.3
        self.angle_tolerance = 0.1
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Robot Driver initialized')
    
    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def waypoint_callback(self, msg):
        """Receive new waypoint."""
        waypoint = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints.append(waypoint)
        self.get_logger().info(f'Received waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})')
        
        # Set first waypoint as target if not already navigating
        if self.target_x is None and len(self.waypoints) > 0:
            self.set_next_waypoint()
    
    def set_next_waypoint(self):
        """Set the next waypoint as the current target."""
        if self.current_waypoint_index < len(self.waypoints):
            self.target_x, self.target_y = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(
                f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({self.target_x:.2f}, {self.target_y:.2f})'
            )
        else:
            self.target_x = None
            self.target_y = None
            self.get_logger().info('All waypoints reached!')
    
    def control_loop(self):
        """Main control loop for navigation."""
        if self.target_x is None or self.target_y is None:
            # No target, stop the robot
            self.stop_robot()
            return
        
        # Calculate distance and angle to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        # Create velocity command
        cmd = Twist()
        
        # Check if waypoint is reached
        if distance < self.position_tolerance:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
            self.current_waypoint_index += 1
            self.set_next_waypoint()
            self.stop_robot()
            return
        
        # Proportional control
        if abs(angle_error) > self.angle_tolerance:
            # Rotate towards target
            cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            cmd.linear.x = 0.0
        else:
            # Move forward
            cmd.linear.x = min(self.linear_speed, distance)
            cmd.angular.z = 0.5 * angle_error  # Small correction
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)
    robot_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
