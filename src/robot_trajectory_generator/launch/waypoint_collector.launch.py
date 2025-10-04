#!/usr/bin/env python3

"""
Waypoint Collector Launch File

Description:
    This launch file starts the waypoint collector node which listens to
    /clicked_point topic from RViz's "Publish Point" tool or from the
    test_case_publisher node. It collects waypoints and publishes them
    as a nav_msgs/Path message on the /waypoints topic.

Nodes Launched:
    - waypoint_collector: Collects clicked points and publishes waypoints

Parameters:
    - frame_id (str): Frame ID for waypoints (default: 'odom')
    - max_waypoints (int): Maximum number of waypoints to collect (default: 100)
    - default_z (float): Default z coordinate for waypoints (default: 0.0)

Topics:
    Subscribes to:
        - /clicked_point (geometry_msgs/PointStamped): Input waypoints
    Publishes to:
        - /waypoints (nav_msgs/Path): Collected waypoints as path

Usage:
    ros2 launch robot_trajectory_generator waypoint_collector.launch.py
    
    # With custom parameters:
    ros2 launch robot_trajectory_generator waypoint_collector.launch.py \
        frame_id:=map max_waypoints:=50

Author: Robot Trajectory Generator Team
Date: 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Frame ID for waypoints (odom or map)'
    )
    
    max_waypoints_arg = DeclareLaunchArgument(
        'max_waypoints',
        default_value='100',
        description='Maximum number of waypoints to collect'
    )
    
    default_z_arg = DeclareLaunchArgument(
        'default_z',
        default_value='0.0',
        description='Default z coordinate for waypoints'
    )
    
    # Waypoint collector node
    waypoint_collector_node = Node(
        package='robot_trajectory_generator',
        executable='waypoint_collector',
        name='waypoint_collector',
        output='screen',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'max_waypoints': LaunchConfiguration('max_waypoints'),
            'default_z': LaunchConfiguration('default_z'),
        }]
    )
    
    return LaunchDescription([
        frame_id_arg,
        max_waypoints_arg,
        default_z_arg,
        waypoint_collector_node,
    ])
