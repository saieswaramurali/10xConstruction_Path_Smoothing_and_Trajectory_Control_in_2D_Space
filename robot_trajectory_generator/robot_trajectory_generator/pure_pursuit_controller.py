#!/usr/bin/env python3

"""
Pure Pursuit Controller Node

This node implements the Pure Pursuit algorithm to follow a trajectory.
It subscribes to trajectory with velocity and robot odometry, then publishes
velocity commands to /cmd_vel to make the robot follow the path.

Pure Pursuit Algorithm:
1. Find the closest point on path to robot
2. Find lookahead point at distance L from robot
3. Calculate curvature to reach lookahead point
4. Convert to linear and angular velocities
5. Publish to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quaternion
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Declare parameters
        self.declare_parameter('lookahead_distance', 0.5)  # meters (balanced for accuracy)
        self.declare_parameter('min_lookahead', 0.25)  # meters (tighter tracking)
        self.declare_parameter('max_lookahead', 0.8)  # meters (not too far ahead)
        self.declare_parameter('goal_tolerance', 0.12)  # meters
        self.declare_parameter('max_linear_vel', 0.35)  # m/s (1.75x - balanced!)
        self.declare_parameter('max_angular_vel', 1.5)  # rad/s (controlled turning)
        self.declare_parameter('use_dynamic_lookahead', True)  # Scale with velocity
        
        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.use_dynamic_lookahead = self.get_parameter('use_dynamic_lookahead').value
        
        # State variables
        self.current_trajectory = None
        self.current_pose = None
        self.goal_reached = False
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            Path,
            '/trajectory_with_velocity',
            self.trajectory_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Pure Pursuit Controller Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Parameters:')
        self.get_logger().info(f'  Lookahead Distance: {self.lookahead_distance} m')
        self.get_logger().info(f'  Lookahead Range: [{self.min_lookahead}, {self.max_lookahead}] m')
        self.get_logger().info(f'  Goal Tolerance: {self.goal_tolerance} m')
        self.get_logger().info(f'  Max Linear Vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max Angular Vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Dynamic Lookahead: {self.use_dynamic_lookahead}')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribed to: /trajectory_with_velocity, /odom')
        self.get_logger().info('Publishing to: /cmd_vel')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for trajectory and odometry...')
    
    def trajectory_callback(self, msg):
        """Callback for trajectory with velocity"""
        if len(msg.poses) < 2:
            self.get_logger().warn('Received empty trajectory')
            return
        
        self.current_trajectory = msg
        self.goal_reached = False
        
        self.get_logger().info(f'Received new trajectory with {len(msg.poses)} points')
        self.get_logger().info('Starting path following...')
    
    def odom_callback(self, msg):
        """Callback for odometry"""
        self.current_pose = msg.pose.pose
    
    def get_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def find_closest_point(self, trajectory, robot_x, robot_y):
        """
        Find the closest point on trajectory to robot
        
        Returns:
            Index of closest point
        """
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose_stamped in enumerate(trajectory.poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            dist = self.get_distance(robot_x, robot_y, x, y)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx
    
    def find_lookahead_point(self, trajectory, robot_x, robot_y, start_idx, lookahead):
        """
        Find the lookahead point on trajectory
        
        Starting from closest point, find first point that is
        approximately lookahead distance away from robot.
        
        Returns:
            Tuple of (x, y, velocity, index) or None if not found
        """
        for i in range(start_idx, len(trajectory.poses)):
            x = trajectory.poses[i].pose.position.x
            y = trajectory.poses[i].pose.position.y
            velocity = trajectory.poses[i].pose.orientation.z  # Encoded velocity
            
            dist = self.get_distance(robot_x, robot_y, x, y)
            
            if dist >= lookahead:
                return (x, y, velocity, i)
        
        # If no point found, return last point (goal)
        last_pose = trajectory.poses[-1]
        return (
            last_pose.pose.position.x,
            last_pose.pose.position.y,
            last_pose.pose.orientation.z,
            len(trajectory.poses) - 1
        )
    
    def pure_pursuit_control(self, robot_x, robot_y, robot_yaw, lookahead_x, lookahead_y, target_vel):
        """
        Pure Pursuit algorithm to calculate control commands
        
        Args:
            robot_x, robot_y: Robot position
            robot_yaw: Robot heading (radians)
            lookahead_x, lookahead_y: Lookahead point position
            target_vel: Target linear velocity from velocity profile
        
        Returns:
            Tuple of (linear_vel, angular_vel)
        """
        # Transform lookahead point to robot frame
        dx = lookahead_x - robot_x
        dy = lookahead_y - robot_y
        
        # Rotate to robot frame
        dx_robot = dx * math.cos(-robot_yaw) - dy * math.sin(-robot_yaw)
        dy_robot = dx * math.sin(-robot_yaw) + dy * math.cos(-robot_yaw)
        
        # Calculate lookahead distance
        lookahead_dist = math.sqrt(dx_robot**2 + dy_robot**2)
        
        if lookahead_dist < 1e-6:
            return 0.0, 0.0
        
        # Calculate curvature (Pure Pursuit formula)
        # curvature = 2 * dy_robot / L^2
        curvature = (2.0 * dy_robot) / (lookahead_dist ** 2)
        
        # Calculate angular velocity
        # w = v * curvature
        linear_vel = min(target_vel, self.max_linear_vel)
        angular_vel = linear_vel * curvature
        
        # Limit angular velocity
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """Main control loop (called at 20 Hz)"""
        # Check if we have trajectory and odometry
        if self.current_trajectory is None or self.current_pose is None:
            return
        
        if self.goal_reached:
            return
        
        # Get robot position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Get robot orientation (yaw)
        orientation_q = self.current_pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, robot_yaw = euler_from_quaternion(quaternion)
        
        # Find closest point on trajectory
        closest_idx = self.find_closest_point(self.current_trajectory, robot_x, robot_y)
        
        # Check if goal is reached
        goal_pose = self.current_trajectory.poses[-1]
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        dist_to_goal = self.get_distance(robot_x, robot_y, goal_x, goal_y)
        
        if dist_to_goal < self.goal_tolerance:
            # Goal reached!
            self.get_logger().info('🎯 Goal Reached!')
            self.stop_robot()
            self.goal_reached = True
            return
        
        # Dynamic lookahead (optional: scale with velocity)
        lookahead = self.lookahead_distance
        if self.use_dynamic_lookahead:
            # Get current velocity from closest point
            if closest_idx < len(self.current_trajectory.poses):
                current_target_vel = self.current_trajectory.poses[closest_idx].pose.orientation.z
                # Scale lookahead: higher velocity → larger lookahead
                lookahead = self.min_lookahead + (current_target_vel / self.max_linear_vel) * \
                           (self.max_lookahead - self.min_lookahead)
        
        # Find lookahead point
        lookahead_result = self.find_lookahead_point(
            self.current_trajectory, robot_x, robot_y, closest_idx, lookahead
        )
        
        if lookahead_result is None:
            self.get_logger().warn('No lookahead point found')
            self.stop_robot()
            return
        
        lookahead_x, lookahead_y, target_vel, _ = lookahead_result
        
        # Calculate control commands using Pure Pursuit
        linear_vel, angular_vel = self.pure_pursuit_control(
            robot_x, robot_y, robot_yaw,
            lookahead_x, lookahead_y,
            target_vel
        )
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # Log occasionally (every 1 second = 20 control loops)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f'Distance to goal: {dist_to_goal:.2f}m | '
                f'Linear: {linear_vel:.2f}m/s | '
                f'Angular: {angular_vel:.2f}rad/s'
            )
    
    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    node = PurePursuitController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
