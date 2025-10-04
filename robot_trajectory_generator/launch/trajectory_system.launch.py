#!/usr/bin/env python3

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
