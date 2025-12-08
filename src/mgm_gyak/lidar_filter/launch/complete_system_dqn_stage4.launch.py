#!/usr/bin/env python3
"""
Komplett rendszer launch fájl - Teljes objektum detektálási rendszer indítása
TurtleBot3 DQN Stage 4 pályán (ROS 2 Jazzy).

Ez a launch fájl elindítja a teljes rendszert:
1. Gazebo szimulátort TurtleBot3-mal (DQN Stage 4 map)
2. LIDAR objektum detektáló node-ot (lidar_filter_node)
3. RViz2 vizualizációt a meglévő RViz konfiggal

Használat (Jazzy):
    export TURTLEBOT3_MODEL=burger   # DQN példák általában Burger-rel mennek
    ros2 launch lidar_filter complete_system_dqn_stage4.launch.py
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
    1. Gazebo World - TurtleBot3 DQN Stage 4 szimulációs környezet
    2. LIDAR Filter Node - Objektum detektálás
    3. RViz2 - 3D vizualizáció
    """

    # Csomagok elérési útjának feloldása
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_lidar_filter = FindPackageShare('lidar_filter')

    # Launch argumentumok - külső paraméterek a launch fájlhoz
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # GZ_SIM_RESOURCE_PATH beállítása - KRITIKUS a modellek betöltéséhez!
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'),
            'models'
        )
    )

    # 1. Gazebo World Launch - TurtleBot3 DQN Stage 4 szimuláció indítása
    # A hivatalos Jazzy-s machine learning doksi szerint:
    #   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
    #  [oai_citation:0‡ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/?utm_source=chatgpt.com)
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
            # Ha vannak további paramétereid, ide beteheted pl.:
            # 'min_range': 0.2,
            # 'max_range': 10.0,
            # 'cluster_threshold': 0.25,
            # 'min_cluster_size': 3,
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
        set_env_vars_resources,  # ← KRITIKUS: modellek resource path-ja
        gazebo_launch,
        lidar_filter_node,
        rviz_node,
    ])