#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class ObjectTracker:
    

    def __init__(self, max_distance=1.5, timeout=5.0):
        # {id: {'position': (x,y), 'last_seen': time, 'visible': bool}}
        self.tracked_objects = {}
        self.next_id = 0
        self.max_distance = max_distance
        self.timeout = timeout

    def update(self, current_objects, current_time):
        """
        current_objects: list[(x,y)]  -> VILÁG (pl. odom) koordinátában!
        """
        if len(self.tracked_objects) == 0:
            for obj in current_objects:
                self.tracked_objects[self.next_id] = {
                    'position': obj,
                    'last_seen': current_time,
                    'visible': True,
                }
                self.next_id += 1
            return self.get_visible_objects()

        tracked_ids = list(self.tracked_objects.keys())
        tracked_positions = np.array(
            [self.tracked_objects[i]['position'] for i in tracked_ids],
            dtype=float,
        )

        if len(current_objects) == 0:
            self.cleanup_old_objects(current_time)
            return self.get_visible_objects()

        curr = np.array(current_objects, dtype=float)

        
        D = np.linalg.norm(tracked_positions[:, None, :] - curr[None, :, :], axis=2)

        row_ind, col_ind = linear_sum_assignment(D)

        assigned_tracked = set()
        assigned_current = set()

       
        for r, c in zip(row_ind, col_ind):
            if D[r, c] < self.max_distance:
                obj_id = tracked_ids[r]
                self.tracked_objects[obj_id]['position'] = tuple(curr[c])
                self.tracked_objects[obj_id]['last_seen'] = current_time
                self.tracked_objects[obj_id]['visible'] = True
                assigned_tracked.add(obj_id)
                assigned_current.add(c)

        for idx, obj in enumerate(current_objects):
            if idx in assigned_current:
                continue

            obj_np = np.array(obj)
            found_invisible = False

            for obj_id in tracked_ids:
                if obj_id in assigned_tracked:
                    continue

                old_pos = np.array(self.tracked_objects[obj_id]['position'])
                dist_to_invisible = np.linalg.norm(obj_np - old_pos)

                if dist_to_invisible < self.max_distance * 1.5 and not self.tracked_objects[obj_id]['visible']:
                    
                    self.tracked_objects[obj_id]['position'] = tuple(obj)
                    self.tracked_objects[obj_id]['last_seen'] = current_time
                    self.tracked_objects[obj_id]['visible'] = True
                    assigned_tracked.add(obj_id)
                    assigned_current.add(idx)
                    found_invisible = True
                    break

            if not found_invisible:
                
                self.tracked_objects[self.next_id] = {
                    'position': obj,
                    'last_seen': current_time,
                    'visible': True,
                }
                self.next_id += 1

        
        for obj_id in tracked_ids:
            if obj_id not in assigned_tracked:
                self.tracked_objects[obj_id]['visible'] = False

        self.cleanup_old_objects(current_time)
        return self.get_visible_objects()

    def get_visible_objects(self):
        result = []
        for obj_id, data in self.tracked_objects.items():
            if data['visible']:
                x, y = data['position']
                result.append((obj_id, x, y))
        return result

    def cleanup_old_objects(self, current_time):
        to_delete = []
        for obj_id, data in self.tracked_objects.items():
            if current_time - data['last_seen'] > self.timeout:
                to_delete.append(obj_id)

        for d in to_delete:
            del self.tracked_objects[d]


class LidarFilterNode(Node):
    

    def __init__(self):
        super().__init__('lidar_filter_node')

        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

       
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('cluster_threshold', 0.28)   
        self.declare_parameter('dbscan_min_samples', None)

       
        self.declare_parameter('max_object_diameter', 1.2)     
        self.declare_parameter('max_object_length', 1.0)      
        self.declare_parameter('max_cluster_points', 80)       

        
        self.declare_parameter('world_frame', 'odom')
        self.world_frame = self.get_parameter('world_frame').value

       
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.objects_pub = self.create_publisher(PoseArray, '/objects', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.labels_pub = self.create_publisher(MarkerArray, '/object_labels', 10)
        self.cpu_time_pub = self.create_publisher(Float32, '/lidar_filter/cpu_time_ms', 10)

        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        
        self.tracker = ObjectTracker(max_distance=1.5, timeout=5.0)

        self.get_logger().info('LIDAR Filter Node (DBSCAN + Tracker) initialized')
        self.get_logger().info('Publishing: /filtered_scan, /objects, /object_markers, /object_labels')
        self.get_logger().info(f'World frame for tracking: {self.world_frame}')

    

    def scan_callback(self, msg: LaserScan):
        process_start = time.perf_counter()
       
        filtered_scan = self.filter_scan(msg)
        self.filtered_scan_pub.publish(filtered_scan)

       
        objects = self.detect_objects(filtered_scan)
        self.objects_pub.publish(objects)

        
        markers = self.create_markers(objects)
        self.markers_pub.publish(markers)

       
        centroids_local = [(p.position.x, p.position.y) for p in objects.poses]
        centroids_world = self.transform_points_to_world(
            centroids_local,
            header_frame=objects.header.frame_id,
            stamp=objects.header.stamp
        )

        now = self.get_clock().now().nanoseconds * 1e-9
        visible = self.tracker.update(centroids_world, now)

        
        label_markers = self.create_label_markers(visible, frame_id=self.world_frame)
        self.labels_pub.publish(label_markers)

        process_time_ms = (time.perf_counter() - process_start) * 1000.0
        self.cpu_time_pub.publish(Float32(data=float(process_time_ms)))

        if len(visible) > 0:
            self.get_logger().info(
                f'Tracking {len(visible)} objects with IDs: {[obj_id for obj_id, _, _ in visible]}',
                throttle_duration_sec=1.0
            )

   

    def filter_scan(self, scan: LaserScan) -> LaserScan:
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

        filtered.ranges = [
            r if min_range <= r <= max_range else float('inf')
            for r in scan.ranges
        ]
        filtered.intensities = scan.intensities

        return filtered

    

    def detect_objects(self, scan: LaserScan) -> PoseArray:
        min_cluster_size = self.get_parameter('min_cluster_size').value
        cluster_threshold = self.get_parameter('cluster_threshold').value
        dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        if dbscan_min_samples is None:
            dbscan_min_samples = min_cluster_size

        max_object_diameter = self.get_parameter('max_object_diameter').value
        max_object_length = self.get_parameter('max_object_length').value
        max_cluster_points = self.get_parameter('max_cluster_points').value

        points = []
        for i, r in enumerate(scan.ranges):
            if not np.isinf(r) and not np.isnan(r):
                angle = scan.angle_min + i * scan.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])

        pose_array = PoseArray()
        pose_array.header = scan.header

        if len(points) == 0:
            return pose_array

        points = np.array(points, dtype=float)

        
        clusters = self.dbscan_clustering(points, eps=cluster_threshold, min_samples=dbscan_min_samples)

        for cluster in clusters:
            num_pts = len(cluster)
            if num_pts < min_cluster_size:
                continue

            
            xs = cluster[:, 0]
            ys = cluster[:, 1]
            dx = float(xs.max() - xs.min())
            dy = float(ys.max() - ys.min())
            length = math.hypot(dx, dy)       
            width = min(abs(dx), abs(dy))     

            centroid = np.mean(cluster, axis=0)
            radii = np.linalg.norm(cluster - centroid, axis=1)
            diameter = float(radii.max() * 2.0)

           
            is_wall_like = (
                length > max_object_length or
                diameter > max_object_diameter or
                num_pts > max_cluster_points
            )

            if is_wall_like:
                
                continue

            
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array

    def dbscan_clustering(self, points, eps=0.2, min_samples=3):
        if len(points) < min_samples:
            return []

        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_

        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            clusters.append(points[labels == label])

        return clusters

   

    def transform_points_to_world(self, points_xy, header_frame: str, stamp):
        
        if not points_xy:
            return []

        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                header_frame,
                Time()  
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(
                f'Cannot transform {header_frame} -> {self.world_frame}: {ex}',
                throttle_duration_sec=1.0
            )
            )
            return points_xy

        trans = t.transform.translation
        rot = t.transform.rotation

        yaw = self._yaw_from_quaternion(rot)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        world_points = []
        for x, y in points_xy:
            wx = cos_y * x - sin_y * y + trans.x
            wy = sin_y * x + cos_y * y + trans.y
            world_points.append((wx, wy))

        return world_points

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

  
    def create_markers(self, objects: PoseArray) -> MarkerArray:
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

    def create_label_markers(self, visible_objects, frame_id="odom") -> MarkerArray:
        
        arr = MarkerArray()
        for obj_id, x, y in visible_objects:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "object_labels"
            m.id = int(obj_id)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.text = f"OBJ_{obj_id}"
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.2
            m.color.a = 1.0
            m.lifetime.sec = 1
            arr.markers.append(m)
        return arr


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
