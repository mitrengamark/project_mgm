#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   
    
   
    lidar_filter_node = Node(
        package='lidar_filter',  
        executable='lidar_filter_node', 
        name='lidar_filter_node',  
        output='screen', 
        parameters=[{
           
            'min_range': 0.1,  
            'max_range': 10.0, 
          
            'min_cluster_size': 3,  
            'cluster_threshold': 0.28,  
        }],
        remappings=[
            
        ]
    )
    
    return LaunchDescription([
        lidar_filter_node,
    ])
