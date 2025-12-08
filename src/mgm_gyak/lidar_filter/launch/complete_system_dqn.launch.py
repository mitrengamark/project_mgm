#!/usr/bin/env python3
"""
Komplett rendszer launch fájl - Teljes objektum detektálási rendszer indítása
TurtleBot3 dqn stage2 pályán.

Ez a launch fájl elindítja a teljes rendszert:
1. Gazebo szimulátort TurtleBot3-mal (TurtleBot3 dqn stage2 world)
2. LIDAR objektum detektáló node-ot
3. RViz2 vizualizációt egyedi konfigurációval

Használat:
    ros2 launch lidar_filter complete_system_dqn.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Teljes rendszer launch description generálása.

    Komponensek:
    1. Gazebo World - Stage 4 szimulációs környezet (5x5 pálya, 9 akadály)
    2. LIDAR Filter Node - Objektum detektálás
    3. RViz2 - 3D vizualizáció
    """

    # Csomagok elérési útjának feloldása
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_lidar_filter = FindPackageShare('lidar_filter')

    # Launch argumentumok - külső paraméterek a launch fájlhoz
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Gazebo World Launch - TurtleBot3 DQN Stage 4 (hivatalos launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_dqn_stage4.launch.py'
            ])
        ]),
    )

    # 2. LIDAR Filter Node - Objektum detektáló node
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Ha vannak paramétereid, ide beteheted:
            # 'min_range': 0.2,
            # 'max_range': 10.0,
            # stb.
        }]
    )

    # 3. RViz2 - Vizualizációs tool egyedi konfigurációval
    rviz_config_file = PathJoinSubstitution([
        pkg_lidar_filter,
        'config',
        'lidar_filter_rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # LaunchDescription összeállítása
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (Gazebo órához igazítva)'
        ),
        gazebo_launch,
        lidar_filter_node,
        rviz_node,
    ])
