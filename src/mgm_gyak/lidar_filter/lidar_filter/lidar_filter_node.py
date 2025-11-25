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
        """Node inicializálása - Publishers, Subscribers és paraméterek beállítása."""
        super().__init__('lidar_filter_node')
        
        # Publishers - Kimeneti topic-ok létrehozása
        # /filtered_scan: Szűrt LIDAR adatok (érvénytelen értékek eltávolítva)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        # /objects: Detektált objektumok pozíciói PoseArray formátumban
        self.objects_pub = self.create_publisher(PoseArray, '/objects', 10)
        # /map: Occupancy grid térkép (jelenleg nem használt)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        # /object_markers: Vizualizációs markerek RViz-hez
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        
        # Subscriber - Feliratkozás a LIDAR /scan topic-ra
        # A TurtleBot3 folyamatosan publikálja a LIDAR méréseket erre a topic-ra
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS queue méret - utolsó 10 üzenet bufferelése
        )
        
        # Parameters - Konfigurálható paraméterek definiálása
        # min_range: Minimális érvényes távolság (m) - közelebbi pontok kiszűrése
        self.declare_parameter('min_range', 0.1)
        # max_range: Maximális érvényes távolság (m) - távolabbi pontok kiszűrése
        self.declare_parameter('max_range', 10.0)
        # min_cluster_size: Minimum pontok száma egy érvényes klaszterben
        self.declare_parameter('min_cluster_size', 3)
        # cluster_threshold: Maximális távolság (m) két pont között egy klaszterben
        self.declare_parameter('cluster_threshold', 0.2)
        
        self.get_logger().info('LIDAR Filter Node initialized')

    def scan_callback(self, msg):
        """
        LIDAR scan callback függvény - minden beérkező scan-re meghívódik.
        
        Működés:
        1. Szűri a nyers LIDAR adatokat (távolság alapján)
        2. Detektálja az objektumokat clustering algoritmussal
        3. Létrehozza a vizualizációs markereket
        4. Publikálja az eredményeket a megfelelő topic-okra
        
        Args:
            msg (LaserScan): Beérkező LIDAR scan üzenet a TurtleBot3-tól
        """
        # 1. Scan szűrés - Érvénytelen értékek (túl közeli/távoli) kiszűrése
        filtered_scan = self.filter_scan(msg)
        self.filtered_scan_pub.publish(filtered_scan)
        
        # 2. Objektum detektálás - Klaszterezés és centroid számítás
        objects = self.detect_objects(filtered_scan)
        self.objects_pub.publish(objects)
        
        # 3. Vizualizációs markerek létrehozása RViz-hez
        markers = self.create_markers(objects)
        self.markers_pub.publish(markers)
        
        # Debug log - Detektált objektumok száma
        self.get_logger().debug(f'Detected {len(objects.poses)} objects')

    def filter_scan(self, scan):
        """
        LIDAR scan adatok szűrése távolság alapján.
        
        Eltávolítja az érvénytelen méréseket:
        - Túl közeli pontok (< min_range): sensor zaj, robot teste
        - Túl távoli pontok (> max_range): megbízhatatlan mérések
        
        Args:
            scan (LaserScan): Nyers LIDAR scan
            
        Returns:
            LaserScan: Szűrt scan, érvénytelen értékek inf-re állítva
        """
        # Paraméterek lekérése a konfigurációból
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        
        # Új LaserScan üzenet létrehozása
        filtered = LaserScan()
        # Metaadatok másolása az eredeti scan-ből
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min  # Kezdő szög (radián)
        filtered.angle_max = scan.angle_max  # Záró szög (radián)
        filtered.angle_increment = scan.angle_increment  # Szög lépésköz
        filtered.time_increment = scan.time_increment  # Időkülönbség mérések között
        filtered.scan_time = scan.scan_time  # Teljes scan idő
        filtered.range_min = min_range
        filtered.range_max = max_range
        
        # Távolság értékek szűrése - tartományon kívüli értékek inf-re
        # inf érték jelzi hogy nincs érvényes mérés az adott szögben
        filtered.ranges = [
            r if min_range <= r <= max_range else float('inf')
            for r in scan.ranges
        ]
        filtered.intensities = scan.intensities  # Intenzitás értékek megtartása
        
        return filtered

    def detect_objects(self, scan):
        """
        Objektumok detektálása szűrt LIDAR scan-ből clustering használatával.
        
        Algoritmus lépései:
        1. Polár koordináták (távolság, szög) -> Descartes koordinátákra (x, y)
        2. Közeli pontok klaszterezése (távolság alapú)
        3. Klaszter centroidok számítása -> objektum pozíciók
        
        Args:
            scan (LaserScan): Szűrt LIDAR scan
            
        Returns:
            PoseArray: Detektált objektumok pozíciói (robothoz képest)
        """
        # Clustering paraméterek lekérése
        min_cluster_size = self.get_parameter('min_cluster_size').value
        cluster_threshold = self.get_parameter('cluster_threshold').value
        
        # 1. Polár -> Descartes koordináta transzformáció
        # LIDAR polár koordinátákat ad: (távolság, szög)
        # Objektum detektáláshoz Descartes koordináták kellenek: (x, y)
        points = []
        for i, r in enumerate(scan.ranges):
            # Érvényes mérések kiválasztása (nem inf és nem NaN)
            if not np.isinf(r) and not np.isnan(r):
                # Szög kiszámítása: kezdő szög + index * lépésköz
                angle = scan.angle_min + i * scan.angle_increment
                # Polár -> Descartes konverzió
                x = r * np.cos(angle)  # x koordináta (előre)
                y = r * np.sin(angle)  # y koordináta (oldalt)
                points.append([x, y])
        
        # Ha nincs érvényes pont, üres PoseArray visszaadása
        if len(points) == 0:
            return PoseArray(header=scan.header)
        
        points = np.array(points)  # Lista -> NumPy array (gyorsabb számítás)
        
        # 2. Távolság alapú clustering - közeli pontok csoportosítása
        clusters = self.simple_clustering(points, cluster_threshold, min_cluster_size)
        
        # 3. PoseArray létrehozása klaszter centroidokból
        pose_array = PoseArray()
        pose_array.header = scan.header  # Időbélyeg és koordináta-rendszer
        
        for cluster in clusters:
            # Centroid = klaszter átlagos pozíciója -> objektum közepe
            centroid = np.mean(cluster, axis=0)
            pose = Pose()
            pose.position.x = centroid[0]  # x pozíció (m)
            pose.position.y = centroid[1]  # y pozíció (m)
            pose.position.z = 0.0  # z = 0 (2D síkban)
            pose.orientation.w = 1.0  # Egység quaternion (nincs forgatás)
            pose_array.poses.append(pose)
        
        return pose_array

    def simple_clustering(self, points, threshold, min_size):
        """
        Egyszerű távolság alapú clustering algoritmus.
        
        Működés:
        - Végigmegy minden ponton
        - Klasztert épít a küszöbértéken belüli közeli pontokból
        - Csak a minimum méretnél nagyobb klasztereket tartja meg
        
        Ez egy egyszerűsített DBSCAN-szerű megközelítés:
        - Nem használ core points koncepciót
        - Csak szomszédsági távolságot néz
        
        Args:
            points (np.array): 2D pontok tömbje [[x1,y1], [x2,y2], ...]
            threshold (float): Maximális távolság (m) két pont között egy klaszterben
            min_size (int): Minimum pontok száma egy érvényes klaszterhez
            
        Returns:
            list: Klaszterek listája, mindegyik egy NumPy array pontokkal
        """
        if len(points) == 0:
            return []
        
        clusters = []  # Talált klaszterek
        visited = np.zeros(len(points), dtype=bool)  # Feldolgozott pontok jelzése
        
        # Minden pont feldolgozása
        for i in range(len(points)):
            if visited[i]:  # Már feldolgozott pont kihagyása
                continue
            
            # Új klaszter indítása az aktuális ponttal
            cluster = [points[i]]
            visited[i] = True
            
            # Klaszter növesztése - közeli pontok hozzáadása
            for j in range(i + 1, len(points)):
                if visited[j]:  # Már feldolgozott pont kihagyása
                    continue
                
                # Távolság ellenőrzése a klaszter bármely pontjától
                # Ha van közeli pont a klaszterben -> hozzáadás
                for point in cluster:
                    dist = np.linalg.norm(points[j] - point)  # Euklideszi távolság
                    if dist < threshold:
                        cluster.append(points[j])
                        visited[j] = True
                        break  # Elég egy közeli pont, tovább a következő j-re
            
            # Csak elég nagy klasztereket tartjuk meg
            # Kis klaszterek = zaj, nem valós objektumok
            if len(cluster) >= min_size:
                clusters.append(np.array(cluster))
        
        return clusters

    def create_markers(self, objects):
        """
        Vizualizációs markerek létrehozása RViz-hez.
        
        Minden detektált objektumhoz egy piros hengert hoz létre,
        amely RViz-ben megjeleníthető. Ez segít debug-olni és
        demonstrálni az objektum detektálást.
        
        Args:
            objects (PoseArray): Detektált objektumok pozíciói
            
        Returns:
            MarkerArray: Vizualizációs markerek tömbje RViz-hez
        """
        marker_array = MarkerArray()
        
        # Minden objektumhoz egy marker létrehozása
        for i, pose in enumerate(objects.poses):
            marker = Marker()
            marker.header = objects.header  # Időbélyeg és koordináta-rendszer
            marker.ns = "objects"  # Namespace - marker csoportosításhoz
            marker.id = i  # Egyedi ID - marker azonosításhoz és frissítéshez
            marker.type = Marker.CYLINDER  # Forma: henger (objektumot reprezentál)
            marker.action = Marker.ADD  # Művelet: marker hozzáadása/frissítése
            marker.pose = pose  # Pozíció és orientáció
            
            # Marker mérete
            marker.scale.x = 0.2  # Átmérő X irányban (m)
            marker.scale.y = 0.2  # Átmérő Y irányban (m)
            marker.scale.z = 0.5  # Magasság (m)
            
            # Marker színe - piros, félig átlátszó
            marker.color.r = 1.0  # Piros csatorna
            marker.color.g = 0.0  # Zöld csatorna
            marker.color.b = 0.0  # Kék csatorna
            marker.color.a = 0.8  # Alpha (átlátszóság, 0=teljesen átlátszó, 1=teljesen tömör)
            
            # Élettartam - 1 sec után automatikusan törlődik
            # Ez biztosítja hogy eltűnő objektumok markerei ne maradjanak
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        return marker_array


def main(args=None):
    """
    Fő belépési pont a node futtatásához.
    
    Inicializálja a ROS2 környezetet, létrehozza a node-ot,
    és futtatja amíg nem érkezik leállítási jel (Ctrl+C).
    """
    # ROS2 környezet inicializálása
    rclpy.init(args=args)
    
    # LIDAR filter node létrehozása
    node = LidarFilterNode()
    
    try:
        # Node futtatása - folyamatosan dolgozza fel a beérkező üzeneteket
        # Ez egy blokkoló hívás, amíg a node fut
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C kezelése - tiszta leállás
        pass
    finally:
        # Cleanup - erőforrások felszabadítása
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
