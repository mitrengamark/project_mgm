#!/usr/bin/env python3
"""
LIDAR Filter Node egyszerű launch fájl.

Ez a launch fájl csak a lidar_filter_node-ot indítja el a szükséges
paraméterekkel. Használható ha a TurtleBot3 és RViz már fut,
vagy ha csak a node-ot szeretnéd tesztelni.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch description generálása ROS2 launch rendszerhez.
    
    Létrehoz egy Node objektumot a lidar_filter_node futtatásához
    előre konfigurált paraméterekkel.
    
    Returns:
        LaunchDescription: A launch fájl leírása
    """
    
    # LIDAR filter node konfigurálása
    lidar_filter_node = Node(
        package='lidar_filter',  # ROS2 csomag neve
        executable='lidar_filter_node',  # Futtatható fájl neve (setup.py-ban definiálva)
        name='lidar_filter_node',  # Node neve a ROS2 gráfban
        output='screen',  # Log kimenete a terminálra
        parameters=[{
            # Szűrési paraméterek
            'min_range': 0.1,  # Minimális érvényes távolság (m)
            'max_range': 10.0,  # Maximális érvényes távolság (m)
            # Clustering paraméterek
            'min_cluster_size': 3,  # Minimum pontszám egy klaszterben
            'cluster_threshold': 0.2,  # Max távolság két pont között (m)
        }],
        remappings=[
            # Topic átnevezések itt adhatók meg ha szükséges
            # Pl.: ('/scan', '/robot1/scan')
        ]
    )
    
    return LaunchDescription([
        lidar_filter_node,
    ])
