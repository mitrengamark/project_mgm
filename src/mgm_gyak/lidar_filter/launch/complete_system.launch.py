#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Complete launch file for LIDAR object detection system.
    
    Launches:
    - Gazebo with TurtleBot3
    - lidar_filter_node
    - RViz2 with custom config
    """
    
    # Paths
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_lidar_filter = FindPackageShare('lidar_filter')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
    )
    
    # LIDAR filter node
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_range': 0.1,
            'max_range': 10.0,
            'min_cluster_size': 3,
            'cluster_threshold': 0.2,
        }],
    )
    
    # RViz2 with config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('lidar_filter'),
        'config',
        'lidar_filter_rviz.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_launch,
        lidar_filter_node,
        rviz_node,
    ])
