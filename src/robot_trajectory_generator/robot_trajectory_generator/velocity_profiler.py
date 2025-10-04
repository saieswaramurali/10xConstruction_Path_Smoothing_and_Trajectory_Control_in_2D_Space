#!/usr/bin/env python3

"""
Velocity Profiler Node

This node subscribes to smooth trajectory and adds velocity profile based on:
- Curvature (slower on curves, faster on straight lines)
- Maximum velocity and acceleration limits

Publishes trajectory with velocity information for the controller.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import math


class VelocityProfiler(Node):
    def __init__(self):
        super().__init__('velocity_profiler')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.35)  # m/s (1.75x - balanced speed)
        self.declare_parameter('min_linear_vel', 0.08)  # m/s (slower on sharp curves)
        self.declare_parameter('max_angular_vel', 1.5)  # rad/s (moderate turning)
        self.declare_parameter('curvature_threshold', 0.3)  # 1/m (more sensitive to curves)
        self.declare_parameter('lookahead_points', 5)  # Points to look ahead for curvature
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.curvature_threshold = self.get_parameter('curvature_threshold').value
        self.lookahead_points = self.get_parameter('lookahead_points').value
        
        # Subscriber to smooth trajectory
        self.trajectory_sub = self.create_subscription(
            Path,
            '/smooth_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publisher for trajectory with velocity
        self.trajectory_with_vel_pub = self.create_publisher(
            Path,
            '/trajectory_with_velocity',
            10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Velocity Profiler Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Parameters:')
        self.get_logger().info(f'  Max Linear Velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Min Linear Velocity: {self.min_linear_vel} m/s')
        self.get_logger().info(f'  Max Angular Velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Curvature Threshold: {self.curvature_threshold} 1/m')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribed to: /smooth_trajectory')
        self.get_logger().info('Publishing to: /trajectory_with_velocity')
        self.get_logger().info('='*60)
    
    def calculate_curvature(self, points, index):
        """
        Calculate curvature at a point using neighboring points
        
        Curvature = |dθ/ds| where θ is heading angle, s is arc length
        Higher curvature = tighter turn = slower speed
        
        Args:
            points: List of (x, y) tuples
            index: Current point index
        
        Returns:
            Curvature value (1/m)
        """
        n = len(points)
        if n < 3 or index < 1 or index >= n - 1:
            return 0.0
        
        # Get neighboring points
        p_prev = points[max(0, index - self.lookahead_points)]
        p_curr = points[index]
        p_next = points[min(n - 1, index + self.lookahead_points)]
        
        # Calculate vectors
        v1 = np.array([p_curr[0] - p_prev[0], p_curr[1] - p_prev[1]])
        v2 = np.array([p_next[0] - p_curr[0], p_next[1] - p_curr[1]])
        
        # Calculate lengths
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
        
        # Normalize vectors
        v1 = v1 / len1
        v2 = v2 / len2
        
        # Calculate angle between vectors (change in heading)
        dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_change = np.arccos(dot_product)
        
        # Arc length approximation
        arc_length = (len1 + len2) / 2.0
        
        # Curvature = angle change / arc length
        if arc_length > 1e-6:
            curvature = angle_change / arc_length
        else:
            curvature = 0.0
        
        return curvature
    
    def velocity_from_curvature(self, curvature):
        """
        Calculate desired velocity based on curvature
        
        Strategy:
        - Low curvature (straight) → High velocity
        - High curvature (sharp turn) → Low velocity
        
        Args:
            curvature: Curvature value (1/m)
        
        Returns:
            Desired linear velocity (m/s)
        """
        # Normalize curvature to [0, 1]
        normalized_curvature = min(curvature / self.curvature_threshold, 1.0)
        
        # Linear interpolation: high curvature → low speed
        velocity = self.max_linear_vel - (self.max_linear_vel - self.min_linear_vel) * normalized_curvature
        
        # Ensure within bounds
        velocity = max(self.min_linear_vel, min(self.max_linear_vel, velocity))
        
        return velocity
    
    def add_velocity_profile(self, path_msg):
        """
        Add velocity profile to trajectory based on curvature
        
        Args:
            path_msg: Input Path message
        
        Returns:
            Path message with velocity encoded in orientation.z
        """
        if len(path_msg.poses) < 3:
            self.get_logger().warn('Not enough points for velocity profiling')
            return path_msg
        
        # Extract points
        points = [(pose.pose.position.x, pose.pose.position.y) 
                  for pose in path_msg.poses]
        
        # Create new path with velocity
        new_path = Path()
        new_path.header = path_msg.header
        new_path.header.stamp = self.get_clock().now().to_msg()
        
        curvatures = []
        velocities = []
        
        for i, pose_stamped in enumerate(path_msg.poses):
            # Calculate curvature at this point
            curvature = self.calculate_curvature(points, i)
            curvatures.append(curvature)
            
            # Get velocity from curvature
            velocity = self.velocity_from_curvature(curvature)
            velocities.append(velocity)
            
            # Create new pose with velocity encoded
            new_pose = PoseStamped()
            new_pose.header = pose_stamped.header
            new_pose.pose.position = pose_stamped.pose.position
            new_pose.pose.orientation = pose_stamped.pose.orientation
            
            # Encode velocity in orientation.z (we'll use this in controller)
            # Note: This is a hack for simple communication
            # Better approach: use custom message type
            new_pose.pose.orientation.z = velocity
            
            new_path.poses.append(new_pose)
        
        # Log statistics
        avg_curvature = np.mean(curvatures)
        max_curvature = np.max(curvatures)
        avg_velocity = np.mean(velocities)
        min_velocity = np.min(velocities)
        max_velocity = np.max(velocities)
        
        self.get_logger().info('Velocity Profile Generated:')
        self.get_logger().info(f'  Avg Curvature: {avg_curvature:.3f} 1/m')
        self.get_logger().info(f'  Max Curvature: {max_curvature:.3f} 1/m')
        self.get_logger().info(f'  Velocity Range: {min_velocity:.3f} - {max_velocity:.3f} m/s')
        self.get_logger().info(f'  Avg Velocity: {avg_velocity:.3f} m/s')
        
        return new_path
    
    def trajectory_callback(self, msg):
        """Callback for smooth trajectory"""
        if len(msg.poses) < 3:
            self.get_logger().warn('Trajectory too short for velocity profiling')
            return
        
        self.get_logger().info(f'Received trajectory with {len(msg.poses)} points')
        
        # Add velocity profile
        trajectory_with_vel = self.add_velocity_profile(msg)
        
        # Publish
        self.trajectory_with_vel_pub.publish(trajectory_with_vel)
        
        self.get_logger().info('Published trajectory with velocity profile')


def main(args=None):
    rclpy.init(args=args)
    
    node = VelocityProfiler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
