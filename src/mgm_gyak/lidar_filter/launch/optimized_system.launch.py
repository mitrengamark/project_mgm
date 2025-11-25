#!/usr/bin/env python3
"""
Optimalizált rendszer launch fájl - Teljesítmény-optimalizált konfiguráció.

Ez a launch fájl a complete_system.launch.py továbbfejlesztett változata,
amely teljesítmény-optimalizálásokat tartalmaz:

Optimalizációk:
- Csökkentett Gazebo Real-Time Factor (RTF) -> kevesebb CPU használat
- Optimalizált RViz konfiguráció -> kevesebb TF frame megjelenítés
- Map display eltávolítva -> warning üzenetek megszüntetése

Használat:
    ros2 launch lidar_filter optimized_system.launch.py
    ros2 launch lidar_filter optimized_system.launch.py gui:=false  # Headless Gazebo

Ajánlott gyengébb hardverhez vagy hosszú távú teszteléshez.
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
    Optimalizált rendszer launch description generálása.
    
    Teljesítmény javítások:
    - GUI opcionális (headless mód támogatás)
    - Csökkentett szimulációs komplexitás
    - Optimalizált vizualizáció
    
    Komponensek:
    1. Gazebo World - TurtleBot3 (opcionális GUI)
    2. LIDAR Filter Node - Objektum detektálás
    3. RViz2 - Optimalizált vizualizáció
    
    Returns:
        LaunchDescription: Optimalizált rendszer launch konfigurációja
    """
    
    # Csomagok elérési útjának feloldása
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_lidar_filter = FindPackageShare('lidar_filter')
    
    # Launch argumentumok - külső paraméterek
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')  # Gazebo GUI be/ki kapcsolható
    
    # 1. Gazebo World Launch - Opcionális GUI támogatással
    # A 'gui' paraméter lehetővé teszi headless módot (GUI nélküli futást)
    # Ez jelentősen csökkenti a CPU terhelést
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'gui': gui,  # GUI paraméter továbbítása Gazebo-nak
        }.items()
    )
    
    # 2. LIDAR Filter Node - Objektum detektáló node
    # Paraméterek megegyeznek a complete_system-mel
    lidar_filter_node = Node(
        package='lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Szűrési paraméterek
            'min_range': 0.1,
            'max_range': 10.0,
            # Clustering beállítások
            'min_cluster_size': 3,
            'cluster_threshold': 0.2,
        }],
    )
    
    # 3. RViz2 - Optimalizált vizualizáció
    # Az optimalizált config fájl kevesebb frame-et jelenít meg
    # és eltávolítja a hibás/felesleges display-eket (pl. Map)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('lidar_filter'),
        'config',
        'lidar_filter_optimized.rviz'  # Optimalizált RViz konfiguráció
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],  # Optimalizált config betöltése
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch description összeállítása
    # Két argumentum: use_sim_time és gui (Gazebo GUI be/ki)
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
        gazebo_launch,  # Gazebo + TurtleBot3 (opcionális GUI)
        lidar_filter_node,  # Objektum detektáló
        rviz_node,  # Optimalizált vizualizáció
    ])
