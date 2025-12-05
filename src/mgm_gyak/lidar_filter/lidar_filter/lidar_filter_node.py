#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
import numpy as np

from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment


class ObjectTracker:
    """
    Objektum-követő perzisztens ID-kkal.
    
    Magyar magyarázat:
    - Minden objektumot egyedi ID-val követünk (OBJ_0, OBJ_1, stb.)
    - Ha egy objektum eltakarodott, 5 másodpercig tartjuk az ID-ját
    - Ha újra megjelenik, ugyanaz az ID-t kapja vissza
    - A párosítás Hungarian algoritmussal történik (optimális hozzárendelés)
    """

    def __init__(self, max_distance=1.5, timeout=5.0):
        self.tracked_objects = {}  # {id: {'position': (x,y), 'last_seen': time, 'visible': bool}}
        self.next_id = 0
        # KULCS: nagyobb max_distance - robot gyors mozgása + LIDAR szóródás miatt
        self.max_distance = max_distance  # 1.5m - nagyobb tolerancia
        self.timeout = timeout  # 5 sec - hosszú ID megőrzés eltakarodáskor

    def update(self, current_objects, current_time):
        """
        Frissítött hozzárendelés a meglévő tracked objektumokhoz.

        Args:
            current_objects: list of (x, y) - jelenlegi detektált objektumok
            current_time: float seconds - jelenlegi idő (ROS clock)
        Returns:
            list of (id, x, y) - követett objektumok ID-kkal
        """
        # Ha nincs semmi rólunk korábban (első frame)
        if len(self.tracked_objects) == 0:
            for obj in current_objects:
                self.tracked_objects[self.next_id] = {
                    'position': obj,
                    'last_seen': current_time,
                    'visible': True
                }
                self.next_id += 1
            return self.get_visible_objects()

        tracked_ids = list(self.tracked_objects.keys())
        tracked_positions = np.array([self.tracked_objects[i]['position'] for i in tracked_ids])

        if len(current_objects) == 0:
            # Nincs jelen most objektum -> csak takarítunk timeout alapján
            self.cleanup_old_objects(current_time)
            return self.get_visible_objects()

        curr = np.array(current_objects)

        # Távolság mátrix (N x M): régi objektumok vs új objektumok
        # N = régi (tracked) objektumok száma
        # M = új (current) objektumok száma
        D = np.linalg.norm(tracked_positions[:, None, :] - curr[None, :, :], axis=2)

        # Hungarian assignment - optimális hozzárendelés (min total distance)
        # Ez biztosítja, hogy a párosítás globálisan optimális
        row_ind, col_ind = linear_sum_assignment(D)

        assigned_tracked = set()
        assigned_current = set()

        # Feldolgozzuk a Hungarian által javasolt párosításokat
        for r, c in zip(row_ind, col_ind):
            # Csak akkor rendeljük hozzá, ha a távolság elfogadható
            if D[r, c] < self.max_distance:
                obj_id = tracked_ids[r]
                # Frissítjük az objektum pozícióját és time-ját
                self.tracked_objects[obj_id]['position'] = tuple(curr[c])
                self.tracked_objects[obj_id]['last_seen'] = current_time
                self.tracked_objects[obj_id]['visible'] = True
                assigned_tracked.add(obj_id)
                assigned_current.add(c)

        # A nem párosított ÚJ objektumok → új ID-kat kapnak
        # DE CSAK HA nincs olyan régi objektum, ami közel van és eltakarodott
        for idx, obj in enumerate(current_objects):
            if idx not in assigned_current:
                # Mielőtt új ID-t adnánk, keressünk eltakarodott (invisible) objektumot közele
                found_invisible = False
                for obj_id in tracked_ids:
                    if obj_id not in assigned_tracked:
                        # Ez az objektum nem lett párosítva - lehet, hogy eltakarodott
                        old_pos = self.tracked_objects[obj_id]['position']
                        dist_to_invisible = np.linalg.norm(np.array(obj) - np.array(old_pos))
                        
                        # Ha közeli és eltakarodott volt < 5 sec
                        if dist_to_invisible < self.max_distance and not self.tracked_objects[obj_id]['visible']:
                            # Ez valószínűleg az eltakarodott objektum
                            self.tracked_objects[obj_id]['position'] = tuple(obj)
                            self.tracked_objects[obj_id]['last_seen'] = current_time
                            self.tracked_objects[obj_id]['visible'] = True
                            assigned_tracked.add(obj_id)
                            assigned_current.add(idx)
                            found_invisible = True
                            break
                
                # Ha nem találtunk eltakarodott objektumot, új ID kell
                if not found_invisible:
                    self.tracked_objects[self.next_id] = {
                        'position': obj,
                        'last_seen': current_time,
                        'visible': True
                    }
                    self.next_id += 1

        # Eltakarodott objektumok megjelölése (nem volt match az ebben a frame-ben)
        for obj_id in tracked_ids:
            if obj_id not in assigned_tracked:
                self.tracked_objects[obj_id]['visible'] = False

        # Timeout alapján törlés (ha > timeout másodperce nem látható)
        self.cleanup_old_objects(current_time)

        return self.get_visible_objects()

    def get_visible_objects(self):
        """
        Visszaadja a jelenleg LÁTHATÓ objektumokat (id, x, y)
        
        Csak azok az objektumok jelennek meg, amelyek az utolsó frame-ben detektálódtak.
        Az eltakarodott objektumok nem jelennek meg az RViz-ben, de az ID-juk még tartódik.
        """
        result = []
        for obj_id, data in self.tracked_objects.items():
            if data['visible']:  # Csak a látható objektumok
                x, y = data['position']
                result.append((obj_id, x, y))
        return result

    def cleanup_old_objects(self, current_time):
        """
        Töröl objektumokat, amelyek már > timeout másodperce nem voltak láthatók.
        Ez garantálja, hogy ha egy objektum újra megjelenik < timeout másodpercen belül,
        ugyanaz az ID-t kapja vissza.
        """
        to_delete = []
        for obj_id, data in self.tracked_objects.items():
            # Ha timeout-olt az objektum, töröljük
            if current_time - data['last_seen'] > self.timeout:
                to_delete.append(obj_id)
        
        for d in to_delete:
            del self.tracked_objects[d]


class LidarFilterNode(Node):
    """
    LIDAR-based object detection and tracking node.

    Subscribes to /scan topic and publishes:
    - /filtered_scan: filtered LIDAR data
    - /objects: detected objects as PoseArray
    - /map: occupancy grid map
    - /object_markers: cylinder markers (existing behaviour)
    - /object_labels: TEXT markers with persistent IDs
    """

    def __init__(self):
        super().__init__('lidar_filter_node')

        # Publishers
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.objects_pub = self.create_publisher(PoseArray, '/objects', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        # New: text label markers for persistent IDs
        self.labels_pub = self.create_publisher(MarkerArray, '/object_labels', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Params
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_cluster_size', 3)
        # cluster_threshold is preserved but will be interpreted as eps for DBSCAN
        self.declare_parameter('cluster_threshold', 0.2)
        # additional DBSCAN min_samples param (optional separate)
        self.declare_parameter('dbscan_min_samples', None)

        # Tracker instance - magyar kommentek a paraméterekhez
        # max_distance: max távolság (meter) az objektumok között az ID megőrzéshez
        # timeout: hány másodpercig tartjuk az ID-ját egy eltakarodott objektumnak
        # Ha objektum újra megjelenik e timeout alatt, ugyanaz az ID-t kapja vissza ✅
        # JAVÍTÁS: nagyobb max_distance (1.5m) hogy stabil ID-kat tartson
        self.tracker = ObjectTracker(max_distance=1.5, timeout=5.0)

        self.get_logger().info('LIDAR Filter Node (DBSCAN + Tracker) initialized')
        self.get_logger().info('Publishing topics: /filtered_scan, /objects, /object_markers, /object_labels')
        self.get_logger().info('Object tracking: 5 sec timeout, 1.5m max distance (increased for stability)')

    def scan_callback(self, msg):
        # 1. Filter scan
        filtered_scan = self.filter_scan(msg)
        self.filtered_scan_pub.publish(filtered_scan)

        # 2. Detect objects (DBSCAN based)
        objects = self.detect_objects(filtered_scan)
        self.objects_pub.publish(objects)

        # 3. Existing cylinder markers (kept for compatibility)
        markers = self.create_markers(objects)
        self.markers_pub.publish(markers)

        # 4. Persistent ID labeling - centroids -> tracker -> labels
        centroids = [(p.position.x, p.position.y) for p in objects.poses]
        now = self.get_clock().now().nanoseconds * 1e-9
        visible = self.tracker.update(centroids, now)
        label_markers = self.create_label_markers(visible, frame_id=objects.header.frame_id)
        self.labels_pub.publish(label_markers)

        if len(visible) > 0:
            self.get_logger().info(f'Tracking {len(visible)} objects with IDs: {[obj_id for obj_id, _, _ in visible]}', throttle_duration_sec=1.0)

    def filter_scan(self, scan):
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

    def detect_objects(self, scan):
        min_cluster_size = self.get_parameter('min_cluster_size').value
        cluster_threshold = self.get_parameter('cluster_threshold').value
        dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        if dbscan_min_samples is None:
            dbscan_min_samples = min_cluster_size

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

        # DBSCAN alapù klaszterezés
        clusters = self.dbscan_clustering(points, eps=cluster_threshold, min_samples=dbscan_min_samples)

        pose_array = PoseArray()
        pose_array.header = scan.header
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array

    def dbscan_clustering(self, points, eps=0.2, min_samples=3):
        """
        DBSCAN-based clustering. Returns a list of numpy arrays (clusters).
        """
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

    def simple_clustering(self, points, threshold, min_size):
        """Régi egyszerű clustering fenntartva kompatibilitás miatt."""
        if len(points) == 0:
            return []

        clusters = []
        visited = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if visited[i]:
                continue
            cluster = [points[i]]
            visited[i] = True
            for j in range(i + 1, len(points)):
                if visited[j]:
                    continue
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
            # lifetime in seconds
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
        return marker_array

    def create_label_markers(self, visible_objects, frame_id="laser"):
        """
        visible_objects: list of (id, x, y)
        Creates TEXT_VIEW_FACING markers with Marker.id == object id so they persist and update.
        """
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
            # short lifetime so if marker not republished it's removed
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
