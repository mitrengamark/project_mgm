#!/usr/bin/env python3
"""
Komplett rendszer launch fájl - Teljes objektum detektálási rendszer indítása.

Ez a launch fájl elindítja a teljes rendszert:
1. Gazebo szimulátort TurtleBot3-mal
2. LIDAR objektum detektáló node-ot
3. RViz2 vizualizációt egyedi konfigurációval

Használat:
    ros2 launch lidar_filter complete_system.launch.py

A launch fájl ideális demonstrációhoz és teszteléshez, mivel minden
szükséges komponenst egy paranccsal indít.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Teljes rendszer launch description generálása.
    
    Komponensek:
    1. Gazebo World - TurtleBot3 szimulációs környezet
    2. LIDAR Filter Node - Objektum detektálás
    3. RViz2 - 3D vizualizáció
    
    Returns:
        LaunchDescription: Teljes rendszer launch konfigurációja
    """
    
    # Csomagok elérési útjának feloldása
    # FindPackageShare megtalálja a telepített ROS2 csomagok share könyvtárát
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_lidar_filter = FindPackageShare('lidar_filter')
    
    # Launch argumentumok - külső paraméterek a launch fájlhoz
    # Ezek parancssorból felülírhatók: use_sim_time:=false
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Gazebo World Launch - TurtleBot3 szimuláció indítása
    # Include-olja a turtlebot3_world.launch.py fájlt a turtlebot3_gazebo csomagból
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
    )
    
    # 2. LIDAR Filter Node - Objektum detektáló node
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',  # Logok a terminálra
        parameters=[{
            'use_sim_time': use_sim_time,  # Szimulációs idő használata
            # Szűrési paraméterek
            'min_range': 0.1,  # 10 cm minimum távolság
            'max_range': 10.0,  # 10 m maximum távolság
            # Clustering beállítások
            'min_cluster_size': 3,  # Legalább 3 pont = 1 objektum
            'cluster_threshold': 0.2,  # 20 cm max távolság pontok között
        }],
    )
    
    # 3. RViz2 - Vizualizációs tool egyedi konfigurációval
    # A config fájl tartalmazza a display beállításokat (LIDAR, markerek, stb.)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('lidar_filter'),
        'config',
        'lidar_filter_rviz.rviz'  # Egyedi RViz konfiguráció
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],  # Config fájl betöltése
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch description összeállítása
    # DeclareLaunchArgument: argumentum definíció leírással
    # Aztán az összes komponens indítása
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_launch,  # Gazebo + TurtleBot3
        lidar_filter_node,  # Objektum detektáló
        rviz_node,  # Vizualizáció
    ])
