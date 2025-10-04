#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable
    turtlebot3_model = LaunchConfiguration('model', default='burger')
    
    # Set the environment variable
    set_turtlebot3_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=turtlebot3_model
    )
    
    # Declare launch argument for model selection
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type [burger, waffle, waffle_pi]'
    )
    
    # Include empty_world.launch.py from turtlebot3_gazebo package
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ])
    )
    
    # Get the RViz config file path - using custom trajectory display config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'rviz',
        'display.rviz'
    ])
    
    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        declare_model_arg,
        set_turtlebot3_model_env,
        turtlebot3_world_launch,
        rviz_node,
    ])
