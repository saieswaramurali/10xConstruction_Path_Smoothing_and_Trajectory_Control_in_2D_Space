#!/usr/bin/env python3

"""
Complete Navigation System Launch File

Description:
    This launch file starts the complete autonomous navigation pipeline for
    trajectory generation and path following. It integrates four key components
    to enable smooth, velocity-aware robot navigation:
    
    1. Waypoint Collector - Collects waypoints from RViz clicks or test cases
    2. Catmull-Rom Generator - Generates smooth trajectories using Catmull-Rom
       spline interpolation with centripetal parameterization
    3. Velocity Profiler - Assigns velocities based on path curvature (faster
       on straight lines, slower on curves)
    4. Pure Pursuit Controller - Follows the trajectory and publishes velocity
       commands to /cmd_vel for robot motion
    
    This system implements a complete autonomous navigation solution with:
    - Smooth trajectory generation (no sharp corners)
    - Curvature-based velocity planning (safe cornering)
    - Path following control (accurate tracking)

Nodes Launched:
    - waypoint_collector: Collects clicked points and publishes waypoints
    - catmull_rom_generator: Generates smooth Catmull-Rom spline trajectories
    - velocity_profiler: Adds velocity profile based on path curvature
    - pure_pursuit_controller: Follows trajectory using Pure Pursuit algorithm

Parameters:
    - frame_id (str): Frame ID for all components (default: 'odom')
    - num_points (int): Number of points in trajectory (default: 200)
    - alpha (float): Catmull-Rom alpha (0.5=centripetal, recommended)
    - max_linear_vel (float): Maximum linear velocity in m/s (default: 0.22)
    - lookahead_distance (float): Pure Pursuit lookahead in meters (default: 0.5)

Topics:
    Subscribes to:
        - /clicked_point (geometry_msgs/PointStamped): Input waypoints
        - /odom (nav_msgs/Odometry): Robot odometry for control
    Publishes to:
        - /waypoints (nav_msgs/Path): Collected waypoints
        - /smooth_trajectory (nav_msgs/Path): Smooth trajectory
        - /trajectory_with_velocity (nav_msgs/Path): Trajectory with velocities
        - /cmd_vel (geometry_msgs/Twist): Velocity commands to robot

Usage:
    # Step 1: Launch the complete navigation system
    ros2 launch robot_trajectory_generator complete_navigation.launch.py
    
    # Step 2: In another terminal, publish test waypoints
    ros2 run robot_trajectory_generator test_case_publisher
    
    # Custom parameters example:
    ros2 launch robot_trajectory_generator complete_navigation.launch.py \
        max_linear_vel:=0.15 lookahead_distance:=0.6

Workflow:
    1. Start this launch file (brings up all 4 nodes)
    2. Run test_case_publisher to send waypoints
    3. Watch the robot follow the smooth trajectory in Gazebo/RViz
    4. Waypoints → Smooth Path → Velocities → Robot Motion

Visualization in RViz:
    - Green path: Original waypoints
    - Magenta curve: Smooth Catmull-Rom trajectory
    - Robot: Autonomously follows the magenta path

Dependencies:
    - ROS 2 Humble
    - nav_msgs, geometry_msgs
    - TurtleBot3 simulation (robot_bringup package)

Author: Robot Trajectory Generator Team
Date: 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete navigation system"""
    
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Frame ID for waypoints and trajectory'
    )
    
    num_points_arg = DeclareLaunchArgument(
        'num_points',
        default_value='200',
        description='Number of points in smooth trajectory'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.5',
        description='Catmull-Rom alpha parameter (0.5=centripetal)'
    )
    
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.22',
        description='Maximum linear velocity (m/s) - TurtleBot3 limit'
    )
    
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.5',
        description='Pure Pursuit lookahead distance (m)'
    )
    
    # Info message
    info_message = LogInfo(
        msg="\n" + "="*80 + "\n" +
            "  [LAUNCH] COMPLETE NAVIGATION SYSTEM LAUNCHED\n" +
            "="*80 + "\n" +
            "  Pipeline:\n" +
            "  1. Waypoint Collector → Collects waypoints from /clicked_point\n" +
            "  2. Catmull-Rom Generator → Generates smooth trajectory\n" +
            "  3. Velocity Profiler → Adds velocity based on curvature\n" +
            "  4. Pure Pursuit Controller → Follows path, publishes /cmd_vel\n" +
            "="*80 + "\n" +
            "  [INFO] To send waypoints, run in ANOTHER terminal:\n" +
            "     ros2 run robot_trajectory_generator test_case_publisher\n" +
            "="*80 + "\n" +
            "  Topics:\n" +
            "  • Input: /clicked_point (from test_case_publisher or RViz)\n" +
            "  • Waypoints: /waypoints (nav_msgs/Path)\n" +
            "  • Smooth Trajectory: /smooth_trajectory (nav_msgs/Path)\n" +
            "  • Trajectory + Velocity: /trajectory_with_velocity (nav_msgs/Path)\n" +
            "  • Output: /cmd_vel (geometry_msgs/Twist) → Robot moves!\n" +
            "="*80 + "\n" +
            "  Visualization in RViz:\n" +
            "  - Green path: Original waypoints\n" +
            "  - Magenta curve: Smooth trajectory\n" +
            "  - Robot follows the path autonomously!\n" +
            "="*80 + "\n"
    )
    
    # Node 1: Waypoint Collector
    waypoint_collector = Node(
        package='robot_trajectory_generator',
        executable='waypoint_collector',
        name='waypoint_collector',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen'
    )
    
    # Node 2: Catmull-Rom Trajectory Generator
    catmull_rom_generator = Node(
        package='robot_trajectory_generator',
        executable='catmull_rom_generator',
        name='catmull_rom_generator',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'num_points': LaunchConfiguration('num_points'),
            'alpha': LaunchConfiguration('alpha'),
        }],
        output='screen'
    )
    
    # Node 3: Velocity Profiler
    velocity_profiler = Node(
        package='robot_trajectory_generator',
        executable='velocity_profiler',
        name='velocity_profiler',
        parameters=[{
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'min_linear_vel': 0.05,
            'max_angular_vel': 2.0,
            'curvature_threshold': 0.5,
            'lookahead_points': 5,
        }],
        output='screen'
    )
    
    # Node 4: Pure Pursuit Controller
    pure_pursuit_controller = Node(
        package='robot_trajectory_generator',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[{
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'min_lookahead': 0.3,
            'max_lookahead': 1.0,
            'goal_tolerance': 0.1,
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': 2.0,
            'use_dynamic_lookahead': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        frame_id_arg,
        num_points_arg,
        alpha_arg,
        max_linear_vel_arg,
        lookahead_distance_arg,
        
        # Info
        info_message,
        
        # All nodes
        waypoint_collector,
        catmull_rom_generator,
        velocity_profiler,
        pure_pursuit_controller,
    ])
