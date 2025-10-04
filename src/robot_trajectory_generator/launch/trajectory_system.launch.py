#!/usr/bin/env python3

"""
Trajectory System Launch File

Description:
    This launch file starts the complete trajectory generation pipeline:
    1. Waypoint Collector - Collects waypoints from /clicked_point topic
    2. Catmull-Rom Generator - Generates smooth trajectories using Catmull-Rom
       spline interpolation with centripetal parameterization (alpha=0.5)
    
    This is the core trajectory generation system that takes discrete waypoints
    and produces smooth, continuous paths suitable for robot navigation.

Nodes Launched:
    - waypoint_collector: Collects clicked points from RViz or test cases
    - catmull_rom_generator: Generates smooth Catmull-Rom spline trajectories

Parameters:
    - frame_id (str): Frame ID for waypoints and trajectory (default: 'odom')
    - num_points (int): Number of interpolated points in trajectory (default: 200)
    - alpha (float): Catmull-Rom alpha parameter - 0.5 is centripetal (default: 0.5)
                     0.0=uniform, 0.5=centripetal (recommended), 1.0=chordal

Topics:
    Subscribes to:
        - /clicked_point (geometry_msgs/PointStamped): Input waypoints
        - /waypoints (nav_msgs/Path): Collected waypoints
    Publishes to:
        - /waypoints (nav_msgs/Path): Collected waypoints path
        - /smooth_trajectory (nav_msgs/Path): Smooth interpolated trajectory

Usage:
    ros2 launch robot_trajectory_generator trajectory_system.launch.py
    
    # With custom parameters:
    ros2 launch robot_trajectory_generator trajectory_system.launch.py \
        num_points:=300 alpha:=0.5
    
    # Then publish test waypoints in another terminal:
    ros2 run robot_trajectory_generator test_case_publisher

Visualization:
    In RViz, add Path displays for:
    - /waypoints (green): Original waypoints
    - /smooth_trajectory (magenta): Smooth trajectory

Author: Robot Trajectory Generator Team
Date: 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for complete trajectory generation system
    
    This launches:
    1. Waypoint Collector - Collects clicked points from RViz
    2. Catmull-Rom Generator - Generates smooth trajectories
    
    You can then use test_case_publisher to publish predefined test cases
    """
    
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
        description='Catmull-Rom alpha parameter (0.5=centripetal, recommended)'
    )
    
    # Waypoint collector node
    waypoint_collector_node = Node(
        package='robot_trajectory_generator',
        executable='waypoint_collector',
        name='waypoint_collector',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'max_waypoints': 100,
            'default_z': 0.0,
        }]
    )
    
    # Catmull-Rom trajectory generator node
    catmull_rom_generator_node = Node(
        package='robot_trajectory_generator',
        executable='catmull_rom_generator',
        name='catmull_rom_generator',
        output='screen',
        parameters=[{
            'num_points': LaunchConfiguration('num_points'),
            'alpha': LaunchConfiguration('alpha'),
            'frame_id': LaunchConfiguration('frame_id'),
            'auto_generate': True,
        }]
    )
    
    return LaunchDescription([
        frame_id_arg,
        num_points_arg,
        alpha_arg,
        waypoint_collector_node,
        catmull_rom_generator_node,
    ])
