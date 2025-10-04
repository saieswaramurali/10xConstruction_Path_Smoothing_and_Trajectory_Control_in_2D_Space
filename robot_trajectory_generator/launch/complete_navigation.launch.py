#!/usr/bin/env python3

"""
Complete Navigation System Launch File

This launches the complete trajectory following pipeline:
1. Waypoint Collector - collects waypoints
2. Catmull-Rom Generator - generates smooth trajectory
3. Velocity Profiler - adds velocity based on curvature
4. Pure Pursuit Controller - follows trajectory and publishes to /cmd_vel

Usage:
    ros2 launch robot_trajectory_generator complete_navigation.launch.py
    
Then in another terminal, publish waypoints using:
    ros2 run robot_trajectory_generator test_case_publisher
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
            "  🚀 COMPLETE NAVIGATION SYSTEM LAUNCHED 🚀\n" +
            "="*80 + "\n" +
            "  Pipeline:\n" +
            "  1. Waypoint Collector → Collects waypoints from /clicked_point\n" +
            "  2. Catmull-Rom Generator → Generates smooth trajectory\n" +
            "  3. Velocity Profiler → Adds velocity based on curvature\n" +
            "  4. Pure Pursuit Controller → Follows path, publishes /cmd_vel\n" +
            "="*80 + "\n" +
            "  📋 To send waypoints, run in ANOTHER terminal:\n" +
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
