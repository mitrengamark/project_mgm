#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
import numpy as np


class LidarFilterNode(Node):
    """
    LIDAR-based object detection and tracking node.
    
    Subscribes to /scan topic and publishes:
    - /filtered_scan: filtered LIDAR data
    - /objects: detected objects as PoseArray
    - /map: occupancy grid map
    """

    def __init__(self):
        super().__init__('lidar_filter_node')
        
        # Publishers
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.objects_pub = self.create_publisher(PoseArray, '/objects', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('cluster_threshold', 0.2)
        
        self.get_logger().info('LIDAR Filter Node initialized')

    def scan_callback(self, msg):
        """Process incoming LIDAR scan data."""
        # Filter scan data
        filtered_scan = self.filter_scan(msg)
        self.filtered_scan_pub.publish(filtered_scan)
        
        # Detect objects
        objects = self.detect_objects(filtered_scan)
        self.objects_pub.publish(objects)
        
        # Create visualization markers
        markers = self.create_markers(objects)
        self.markers_pub.publish(markers)
        
        self.get_logger().debug(f'Detected {len(objects.poses)} objects')

    def filter_scan(self, scan):
        """Filter LIDAR scan data based on range thresholds."""
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        
        filtered = LaserScan()
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = min_range
        filtered.range_max = max_range
        
        # Filter ranges
        filtered.ranges = [
            r if min_range <= r <= max_range else float('inf')
            for r in scan.ranges
        ]
        filtered.intensities = scan.intensities
        
        return filtered

    def detect_objects(self, scan):
        """Detect objects from filtered LIDAR scan using clustering."""
        min_cluster_size = self.get_parameter('min_cluster_size').value
        cluster_threshold = self.get_parameter('cluster_threshold').value
        
        # Convert polar to Cartesian coordinates
        points = []
        for i, r in enumerate(scan.ranges):
            if not np.isinf(r) and not np.isnan(r):
                angle = scan.angle_min + i * scan.angle_increment
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
        
        if len(points) == 0:
            return PoseArray(header=scan.header)
        
        points = np.array(points)
        
        # Simple clustering based on distance
        clusters = self.simple_clustering(points, cluster_threshold, min_cluster_size)
        
        # Create PoseArray from cluster centroids
        pose_array = PoseArray()
        pose_array.header = scan.header
        
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            pose = Pose()
            pose.position.x = centroid[0]
            pose.position.y = centroid[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        return pose_array

    def simple_clustering(self, points, threshold, min_size):
        """Simple distance-based clustering algorithm."""
        if len(points) == 0:
            return []
        
        clusters = []
        visited = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if visited[i]:
                continue
            
            cluster = [points[i]]
            visited[i] = True
            
            # Grow cluster
            for j in range(i + 1, len(points)):
                if visited[j]:
                    continue
                
                # Check distance to any point in cluster
                for point in cluster:
                    dist = np.linalg.norm(points[j] - point)
                    if dist < threshold:
                        cluster.append(points[j])
                        visited[j] = True
                        break
            
            if len(cluster) >= min_size:
                clusters.append(np.array(cluster))
        
        return clusters

    def create_markers(self, objects):
        """Create visualization markers for detected objects."""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(objects.poses):
            marker = Marker()
            marker.header = objects.header
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
