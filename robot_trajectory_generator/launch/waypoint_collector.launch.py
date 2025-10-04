#!/usr/bin/env python3

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
