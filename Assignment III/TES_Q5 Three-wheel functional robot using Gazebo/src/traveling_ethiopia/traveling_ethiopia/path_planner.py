"""
Path planner using Breadth-First Search (BFS) - an uninformed search strategy.
Finds the shortest path between two cities in the Ethiopian graph.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from collections import deque
from .graph_data import CITY_GRAPH, CITY_COORDINATES, get_neighbors, get_position


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Parameters
        self.declare_parameter('start_city', 'Addis_Ababa')
        self.declare_parameter('goal_city', 'Harar')
        
        self.start_city = self.get_parameter('start_city').value
        self.goal_city = self.get_parameter('goal_city').value
        
        # Publishers
        self.path_pub = self.create_publisher(String, 'planned_path', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, 'current_waypoint', 10)
        
        self.get_logger().info(f'Path Planner initialized')
        self.get_logger().info(f'Start: {self.start_city}, Goal: {self.goal_city}')
        
        # Plan path
        self.plan_path()
    
    def bfs_search(self, start, goal):
        """
        Breadth-First Search algorithm.
        Returns the path from start to goal as a list of cities.
        """
        if start not in CITY_GRAPH or goal not in CITY_GRAPH:
            self.get_logger().error(f'Invalid cities: {start} or {goal}')
            return None
        
        # Queue for BFS: (current_city, path_to_current)
        queue = deque([(start, [start])])
        visited = {start}
        
        self.get_logger().info('Starting BFS search...')
        nodes_explored = 0
        
        while queue:
            current_city, path = queue.popleft()
            nodes_explored += 1
            
            self.get_logger().info(f'Exploring: {current_city} (nodes explored: {nodes_explored})')
            
            # Goal test
            if current_city == goal:
                self.get_logger().info(f'Goal found! Path length: {len(path)}')
                self.get_logger().info(f'Total nodes explored: {nodes_explored}')
                return path
            
            # Expand neighbors
            for neighbor in get_neighbors(current_city):
                if neighbor not in visited:
                    visited.add(neighbor)
                    new_path = path + [neighbor]
                    queue.append((neighbor, new_path))
        
        self.get_logger().error(f'No path found from {start} to {goal}')
        return None
    
    def plan_path(self):
        """Plan path using BFS and publish results."""
        path = self.bfs_search(self.start_city, self.goal_city)
        
        if path:
            # Publish path as string
            path_msg = String()
            path_msg.data = ' -> '.join(path)
            self.path_pub.publish(path_msg)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info('PATH FOUND:')
            self.get_logger().info(path_msg.data)
            self.get_logger().info('=' * 60)
            
            # Publish waypoints
            self.get_logger().info('Publishing waypoints...')
            for i, city in enumerate(path):
                x, y = get_position(city)
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'map'
                waypoint.header.stamp = self.get_clock().now().to_msg()
                waypoint.pose.position.x = x
                waypoint.pose.position.y = y
                waypoint.pose.position.z = 0.0
                
                self.waypoint_pub.publish(waypoint)
                self.get_logger().info(f'Waypoint {i+1}/{len(path)}: {city} at ({x:.1f}, {y:.1f})')
        else:
            self.get_logger().error('Path planning failed!')


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()
    
    # Keep node alive briefly to publish messages
    rclpy.spin_once(planner, timeout_sec=1.0)
    
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
