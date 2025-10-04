#!/usr/bin/env python3

"""
Example Trajectory Generator Node

This node subscribes to waypoints from the waypoint_collector
and demonstrates how to process them for trajectory generation.

This is a template/example that you can modify for your specific needs.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Waypoint subscription
        self.waypoint_sub = self.create_subscription(
            Path,
            '/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Command velocity publisher (example)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Storage
        self.current_waypoints = []
        self.current_waypoint_index = 0
        
        self.get_logger().info('Trajectory Generator started')
        self.get_logger().info('Waiting for waypoints on /waypoints topic...')
    
    def waypoints_callback(self, msg: Path):
        """
        Callback when new waypoints are received
        """
        # Extract waypoints from Path message
        self.current_waypoints = []
        
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            self.current_waypoints.append((x, y, z))
        
        self.get_logger().info(
            f'Received {len(self.current_waypoints)} waypoints'
        )
        
        # Print waypoints
        for i, (x, y, z) in enumerate(self.current_waypoints):
            self.get_logger().info(
                f'  Waypoint {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})'
            )
        
        # TODO: Add your trajectory generation logic here
        # Example: Generate smooth trajectory between waypoints
        # self.generate_trajectory()
    
    def generate_trajectory(self):
        """
        Example method to generate trajectory from waypoints
        
        This is where you would implement:
        - Bezier curves
        - Spline interpolation
        - Minimum jerk trajectories
        - Etc.
        """
        if len(self.current_waypoints) < 2:
            self.get_logger().warn('Need at least 2 waypoints to generate trajectory')
            return
        
        self.get_logger().info('Generating trajectory...')
        
        # Example: Calculate distances between waypoints
        total_distance = 0.0
        for i in range(len(self.current_waypoints) - 1):
            x1, y1, _ = self.current_waypoints[i]
            x2, y2, _ = self.current_waypoints[i + 1]
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            total_distance += distance
            self.get_logger().info(
                f'Segment {i+1}: {distance:.3f} meters'
            )
        
        self.get_logger().info(f'Total path length: {total_distance:.3f} meters')
        
        # TODO: Implement your trajectory generation algorithm here


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
