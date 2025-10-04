#!/usr/bin/env python3

"""
Catmull-Rom Trajectory Generator

This node subscribes to waypoints from the waypoint_collector and generates
smooth trajectories using the Catmull-Rom spline algorithm.

Based on the analysis from TrajAlgo.ipynb which showed Catmull-Rom performs
best for sharp turns and varied curvature paths.

Topics:
- Subscribe: /waypoints (nav_msgs/Path) - from waypoint_collector
- Publish: /smooth_trajectory (nav_msgs/Path) - generated smooth path
- Publish: /trajectory_markers (visualization_msgs/MarkerArray) - visualization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.interpolate import CubicSpline


class CatmullRomTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('catmull_rom_trajectory_generator')
        
        # Parameters
        self.declare_parameter('num_points', 200)
        self.declare_parameter('alpha', 0.5)  # Centripetal parameterization
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('auto_generate', True)  # Auto-generate on new waypoints
        
        self.num_points = self.get_parameter('num_points').value
        self.alpha = self.get_parameter('alpha').value
        self.frame_id = self.get_parameter('frame_id').value
        self.auto_generate = self.get_parameter('auto_generate').value
        
        # Storage
        self.current_waypoints = []
        self.smooth_trajectory = None
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Path,
            '/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(Path, '/smooth_trajectory', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.5, self.publish_trajectory)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Catmull-Rom Trajectory Generator Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Algorithm: Catmull-Rom Spline (Centripetal)')
        self.get_logger().info(f'Parameters:')
        self.get_logger().info(f'  - num_points: {self.num_points}')
        self.get_logger().info(f'  - alpha: {self.alpha} (centripetal)')
        self.get_logger().info(f'  - frame_id: {self.frame_id}')
        self.get_logger().info(f'  - auto_generate: {self.auto_generate}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for waypoints on /waypoints topic...')
    
    def waypoints_callback(self, msg: Path):
        """Callback when new waypoints are received"""
        # Extract waypoints
        self.current_waypoints = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.current_waypoints.append((x, y))
        
        num_wp = len(self.current_waypoints)
        self.get_logger().info(f'Received {num_wp} waypoints')
        
        if num_wp < 2:
            self.get_logger().warn('Need at least 2 waypoints to generate trajectory')
            return
        
        if self.auto_generate:
            self.generate_trajectory()
    
    def catmull_rom_interpolate(self, waypoints, num_points, alpha):
        """
        Catmull-Rom Spline with Centripetal Parameterization
        
        Properties:
        - Passes through ALL waypoints exactly
        - No overshooting on sharp turns
        - Centripetal (alpha=0.5) best for varied curvature
        - C1 continuous (smooth velocity)
        """
        waypoints = np.array(waypoints)
        
        if len(waypoints) < 2:
            return waypoints
        
        # Remove duplicates
        mask = np.ones(len(waypoints), dtype=bool)
        mask[1:] = ~np.all(waypoints[1:] == waypoints[:-1], axis=1)
        waypoints = waypoints[mask]
        
        if len(waypoints) < 2:
            return waypoints
        
        def catmull_rom_segment(p0, p1, p2, p3, num_seg_points, alpha):
            """Generate Catmull-Rom curve segment"""
            def tj(ti, pi, pj):
                xi, yi = pi
                xj, yj = pj
                return ti + ((xj - xi)**2 + (yj - yi)**2)**(alpha/2)
            
            t0 = 0
            t1 = tj(t0, p0, p1)
            t2 = tj(t1, p1, p2)
            t3 = tj(t2, p2, p3)
            
            if t1 == t0 or t2 == t1 or t3 == t2:
                # Degenerate case - return linear interpolation
                t = np.linspace(0, 1, num_seg_points)
                return p1 + t[:, None] * (p2 - p1)
            
            t = np.linspace(t1, t2, num_seg_points)
            
            A1 = (t1 - t)[:, None] / (t1 - t0) * p0 + (t - t0)[:, None] / (t1 - t0) * p1
            A2 = (t2 - t)[:, None] / (t2 - t1) * p1 + (t - t1)[:, None] / (t2 - t1) * p2
            A3 = (t3 - t)[:, None] / (t3 - t2) * p2 + (t - t2)[:, None] / (t3 - t2) * p3
            
            B1 = (t2 - t)[:, None] / (t2 - t0) * A1 + (t - t0)[:, None] / (t2 - t0) * A2
            B2 = (t3 - t)[:, None] / (t3 - t1) * A2 + (t - t1)[:, None] / (t3 - t1) * A3
            
            C = (t2 - t)[:, None] / (t2 - t1) * B1 + (t - t1)[:, None] / (t2 - t1) * B2
            
            return C
        
        # Add virtual points at start and end for boundary conditions
        p_start = 2 * waypoints[0] - waypoints[1]
        p_end = 2 * waypoints[-1] - waypoints[-2]
        extended_waypoints = np.vstack([p_start, waypoints, p_end])
        
        smooth_path = []
        num_segments = len(waypoints) - 1
        points_per_segment = max(num_points // num_segments, 2)
        
        for i in range(len(waypoints) - 1):
            segment = catmull_rom_segment(
                extended_waypoints[i],
                extended_waypoints[i + 1],
                extended_waypoints[i + 2],
                extended_waypoints[i + 3],
                points_per_segment,
                alpha
            )
            smooth_path.append(segment)
        
        smooth_path = np.vstack(smooth_path)
        return smooth_path
    
    def generate_trajectory(self):
        """Generate smooth trajectory using Catmull-Rom spline"""
        if len(self.current_waypoints) < 2:
            self.get_logger().warn('Need at least 2 waypoints')
            return
        
        try:
            self.get_logger().info('Generating Catmull-Rom trajectory...')
            
            # Apply Catmull-Rom interpolation
            self.smooth_trajectory = self.catmull_rom_interpolate(
                self.current_waypoints,
                self.num_points,
                self.alpha
            )
            
            # Calculate path metrics
            path_length = np.sum(np.sqrt(np.sum(np.diff(self.smooth_trajectory, axis=0)**2, axis=1)))
            
            self.get_logger().info(f'[OK] Trajectory generated!')
            self.get_logger().info(f'  - Input waypoints: {len(self.current_waypoints)}')
            self.get_logger().info(f'  - Output points: {len(self.smooth_trajectory)}')
            self.get_logger().info(f'  - Path length: {path_length:.2f} meters')
            
            # Publish immediately
            self.publish_trajectory()
            
        except Exception as e:
            self.get_logger().error(f'Failed to generate trajectory: {str(e)}')
    
    def publish_trajectory(self):
        """Publish the smooth trajectory"""
        if self.smooth_trajectory is None or len(self.smooth_trajectory) == 0:
            return
        
        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in self.smooth_trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.trajectory_pub.publish(path_msg)
        
        # Publish visualization markers
        self.publish_markers()
    
    def publish_markers(self):
        """Create visualization markers for the trajectory"""
        if self.smooth_trajectory is None:
            return
        
        marker_array = MarkerArray()
        
        # Line strip for smooth trajectory
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "smooth_trajectory"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        line_marker.scale.x = 0.08  # Line width
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0  # Magenta for Catmull-Rom
        line_marker.color.a = 0.9
        
        for point in self.smooth_trajectory:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.05  # Slightly elevated
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Points along trajectory
        points_marker = Marker()
        points_marker.header = line_marker.header
        points_marker.ns = "trajectory_points"
        points_marker.id = 1
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        
        points_marker.scale.x = 0.05
        points_marker.scale.y = 0.05
        points_marker.color.r = 1.0
        points_marker.color.g = 0.5
        points_marker.color.b = 0.0
        points_marker.color.a = 0.6
        
        # Sample every 10th point
        for i in range(0, len(self.smooth_trajectory), 10):
            p = Point()
            p.x = float(self.smooth_trajectory[i][0])
            p.y = float(self.smooth_trajectory[i][1])
            p.z = 0.05
            points_marker.points.append(p)
        
        marker_array.markers.append(points_marker)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = CatmullRomTrajectoryGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
