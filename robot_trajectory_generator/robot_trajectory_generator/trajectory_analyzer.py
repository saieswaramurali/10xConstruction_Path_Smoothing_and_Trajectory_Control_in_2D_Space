#!/usr/bin/env python3

"""
Trajectory Analyzer Node

Description:
    This node analyzes the time-parameterized trajectory and displays detailed
    information including timestamps, velocities, positions, and path metrics.
    It subscribes to the trajectory with velocity information and computes:
    - Time stamps for each trajectory point (t0, t1, ..., tn)
    - Position coordinates (x, y) at each time
    - Velocity profile along the path
    - Total path length and estimated completion time
    
    This fulfills the assignment requirement for time-parameterized trajectory:
    trajectory = [(x0, y0, t0), (x1, y1, t1), ..., (xn, yn, tn)]

Nodes:
    - trajectory_analyzer: Analyzes and displays trajectory information

Topics:
    Subscribes to:
        - /trajectory_with_velocity (nav_msgs/Path): Trajectory with velocities
    
Usage:
    ros2 run robot_trajectory_generator trajectory_analyzer
    
    # The node will display:
    # - Summary: total points, path length, estimated time
    # - First 10 points with (x, y, t, v) format
    # - Last 5 points
    # - Statistics: avg velocity, max/min velocities

Author: Robot Trajectory Generator Team
Date: 2025
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class TrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('trajectory_analyzer')
        
        # Subscriber to trajectory with velocity
        self.trajectory_sub = self.create_subscription(
            Path,
            '/trajectory_with_velocity',
            self.trajectory_callback,
            10
        )
        
        self.get_logger().info('Trajectory Analyzer Node Started')
        self.get_logger().info('Waiting for trajectory on /trajectory_with_velocity...')
        self.get_logger().info('This will display time-parameterized trajectory: [(x, y, t), ...]')
        
    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def trajectory_callback(self, msg):
        """
        Analyze trajectory and display time-parameterized information
        
        Computes time stamps by integrating distance/velocity along path
        """
        if len(msg.poses) < 2:
            self.get_logger().warn('Trajectory has less than 2 points')
            return
        
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('TIME-PARAMETERIZED TRAJECTORY ANALYSIS')
        self.get_logger().info('='*80)
        
        # Extract trajectory data
        trajectory_data = []
        cumulative_time = 0.0
        total_distance = 0.0
        
        # First point starts at t=0
        first_pose = msg.poses[0]
        x0 = first_pose.pose.position.x
        y0 = first_pose.pose.position.y
        trajectory_data.append({
            'x': x0,
            'y': y0,
            't': 0.0,
            'v': 0.0,  # Start from rest
            'distance': 0.0
        })
        
        # Calculate time stamps for subsequent points
        for i in range(1, len(msg.poses)):
            prev_pose = msg.poses[i-1]
            curr_pose = msg.poses[i]
            
            # Extract position
            x = curr_pose.pose.position.x
            y = curr_pose.pose.position.y
            
            # Calculate segment distance
            segment_distance = self.calculate_distance(prev_pose, curr_pose)
            total_distance += segment_distance
            
            # Extract velocity from orientation (stored in quaternion z component)
            # This is how velocity_profiler stores velocity information
            velocity = abs(curr_pose.pose.orientation.z)
            
            # If velocity is too small, use a minimum to avoid division by zero
            if velocity < 0.01:
                velocity = 0.05
            
            # Calculate time for this segment: dt = distance / velocity
            dt = segment_distance / velocity
            cumulative_time += dt
            
            trajectory_data.append({
                'x': x,
                'y': y,
                't': cumulative_time,
                'v': velocity,
                'distance': total_distance
            })
        
        # Display summary statistics
        self.get_logger().info(f'\n[SUMMARY]')
        self.get_logger().info(f'  Total Points: {len(trajectory_data)}')
        self.get_logger().info(f'  Path Length: {total_distance:.3f} meters')
        self.get_logger().info(f'  Estimated Time: {cumulative_time:.3f} seconds')
        
        # Calculate velocity statistics
        velocities = [d['v'] for d in trajectory_data[1:]]  # Skip first point (v=0)
        if velocities:
            avg_vel = np.mean(velocities)
            max_vel = np.max(velocities)
            min_vel = np.min(velocities)
            
            self.get_logger().info(f'  Average Velocity: {avg_vel:.3f} m/s')
            self.get_logger().info(f'  Max Velocity: {max_vel:.3f} m/s')
            self.get_logger().info(f'  Min Velocity: {min_vel:.3f} m/s')
        
        # Display first 10 points in detail
        self.get_logger().info(f'\n[TRAJECTORY POINTS - First 10]')
        self.get_logger().info('  Format: Point | (x, y, t) | velocity')
        self.get_logger().info('  ' + '-'*70)
        
        for i, point in enumerate(trajectory_data[:10]):
            self.get_logger().info(
                f'  Point {i:3d} | '
                f'({point["x"]:7.3f}, {point["y"]:7.3f}, {point["t"]:7.3f}s) | '
                f'v={point["v"]:5.3f} m/s'
            )
        
        if len(trajectory_data) > 15:
            self.get_logger().info(f'  ... ({len(trajectory_data)-15} points omitted) ...')
            
            # Display last 5 points
            self.get_logger().info(f'\n[TRAJECTORY POINTS - Last 5]')
            for i in range(len(trajectory_data)-5, len(trajectory_data)):
                point = trajectory_data[i]
                self.get_logger().info(
                    f'  Point {i:3d} | '
                    f'({point["x"]:7.3f}, {point["y"]:7.3f}, {point["t"]:7.3f}s) | '
                    f'v={point["v"]:5.3f} m/s'
                )
        
        # Display assignment deliverable format
        self.get_logger().info(f'\n[ASSIGNMENT DELIVERABLE FORMAT]')
        self.get_logger().info('  Trajectory as [(x, y, t), ...]:')
        self.get_logger().info('  ' + '-'*70)
        
        # Show first 5 in assignment format
        for i in range(min(5, len(trajectory_data))):
            point = trajectory_data[i]
            self.get_logger().info(
                f'  ({point["x"]:.3f}, {point["y"]:.3f}, {point["t"]:.3f})'
            )
        
        if len(trajectory_data) > 10:
            self.get_logger().info(f'  ... and {len(trajectory_data)-10} more points ...')
            
            # Show last 5 in assignment format
            for i in range(len(trajectory_data)-5, len(trajectory_data)):
                point = trajectory_data[i]
                self.get_logger().info(
                    f'  ({point["x"]:.3f}, {point["y"]:.3f}, {point["t"]:.3f})'
                )
        
        self.get_logger().info('='*80 + '\n')
        
        # Export to file for easy viewing
        self.export_to_file(trajectory_data)
    
    def export_to_file(self, trajectory_data):
        """Export trajectory to a text file for analysis"""
        try:
            filename = '/tmp/trajectory_time_parameterized.txt'
            with open(filename, 'w') as f:
                f.write("TIME-PARAMETERIZED TRAJECTORY\n")
                f.write("="*80 + "\n\n")
                f.write("Format: x (m), y (m), t (s), v (m/s), cumulative_distance (m)\n")
                f.write("-"*80 + "\n\n")
                
                for i, point in enumerate(trajectory_data):
                    f.write(f"{i:4d}  {point['x']:8.4f}  {point['y']:8.4f}  "
                           f"{point['t']:8.4f}  {point['v']:6.4f}  {point['distance']:8.4f}\n")
                
                f.write("\n" + "="*80 + "\n")
                f.write(f"Total points: {len(trajectory_data)}\n")
                f.write(f"Total distance: {trajectory_data[-1]['distance']:.3f} m\n")
                f.write(f"Total time: {trajectory_data[-1]['t']:.3f} s\n")
            
            self.get_logger().info(f'[FILE EXPORT] Trajectory saved to: {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to export trajectory: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
