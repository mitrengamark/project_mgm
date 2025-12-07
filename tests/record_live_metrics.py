#!/usr/bin/env python3
"""Élő metrika gyűjtő a lidar_filter node-hoz.

Gyűjtött metrikák (másodpercenként egy sor):
- Scan rate (Hz) a /filtered_scan topicból
- Detektált objektumok száma (/objects)
- Átlag obj/scan (ablakon belül)
- Sikeres detektálás (%) = legalább 1 objektum/scan arány
- CPU-idő (ms/frame) = lidar_filter_node scan_callback futási ideje (/lidar_filter/cpu_time_ms)

Használat (külön terminálban, miközben a rendszer fut):
    cd ~/codes/mgm/project_mgm
    source install/setup.bash
    python3 tests/record_live_metrics.py --duration 60 --output live_metrics.csv

A megadott idő (alap 60 mp) lejárta után automatikusan leáll, és a fájl végére
összefoglaló sort ír (SUMMARY).
"""

import argparse
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32


class LiveMetricsNode(Node):
    def __init__(self, duration_sec: float, output: str):
        super().__init__('live_metrics_recorder')
        self.duration_sec = duration_sec
        self.output = output

        # Ablak (1s) számlálók
        self.scan_count_window = 0
        self.objects_sum_window = 0
        self.success_window = 0
        self.cpu_sum_window = 0.0
        self.cpu_count_window = 0

        # Összesített számlálók
        self.total_scans = 0
        self.total_objects = 0
        self.total_success = 0
        self.total_cpu_sum = 0.0
        self.total_cpu_count = 0

        # Utolsó ismert értékek
        self.last_objects = 0
        self.last_cpu_ms = None

        self.start_time = time.monotonic()
        self.last_tick = self.start_time

        # CSV előkészítés
        self.csv_file = open(self.output, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp',
            'scan_rate_hz',
            'object_count',
            'avg_objects_per_scan',
            'success_rate_percent',
            'cpu_time_ms_per_frame'
        ])

        # Feliratkozások
        self.create_subscription(LaserScan, '/filtered_scan', self.scan_cb, 10)
        self.create_subscription(PoseArray, '/objects', self.objects_cb, 10)
        self.create_subscription(Float32, '/lidar_filter/cpu_time_ms', self.cpu_cb, 10)

        # Timer: másodpercenként kiírjuk az értékeket
        self.timer = self.create_timer(1.0, self.tick)

    def scan_cb(self, msg: LaserScan):
        self.scan_count_window += 1
        self.total_scans += 1

        self.objects_sum_window += self.last_objects
        self.total_objects += self.last_objects

        if self.last_objects > 0:
            self.success_window += 1
            self.total_success += 1

        if self.last_cpu_ms is not None:
            self.cpu_sum_window += self.last_cpu_ms
            self.cpu_count_window += 1
            self.total_cpu_sum += self.last_cpu_ms
            self.total_cpu_count += 1

    def objects_cb(self, msg: PoseArray):
        self.last_objects = len(msg.poses)

    def cpu_cb(self, msg: Float32):
        self.last_cpu_ms = float(msg.data)

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.last_tick
        if elapsed <= 0:
            elapsed = 1e-6

        scans = self.scan_count_window
        scan_rate = scans / elapsed
        avg_objects = self.objects_sum_window / scans if scans > 0 else 0.0
        success_rate = (self.success_window / scans * 100.0) if scans > 0 else 0.0
        cpu_ms = self.cpu_sum_window / self.cpu_count_window if self.cpu_count_window > 0 else 0.0

        ts = datetime.now().isoformat()
        self.writer.writerow([
            ts,
            f"{scan_rate:.2f}",
            self.last_objects,
            f"{avg_objects:.2f}",
            f"{success_rate:.1f}",
            f"{cpu_ms:.2f}",
        ])
        self.csv_file.flush()

        # Ciklus reset az ablakra
        self.scan_count_window = 0
        self.objects_sum_window = 0
        self.success_window = 0
        self.cpu_sum_window = 0.0
        self.cpu_count_window = 0
        self.last_tick = now

        # Időtartam lejárt?
        if now - self.start_time >= self.duration_sec:
            self.finish(now)

    def finish(self, now_monotonic: float):
        duration = now_monotonic - self.start_time
        if duration <= 0:
            duration = 1e-6

        avg_scan_rate = self.total_scans / duration
        avg_objects_per_scan = self.total_objects / self.total_scans if self.total_scans > 0 else 0.0
        success_rate = (self.total_success / self.total_scans * 100.0) if self.total_scans > 0 else 0.0
        cpu_ms = self.total_cpu_sum / self.total_cpu_count if self.total_cpu_count > 0 else 0.0

        self.writer.writerow([])
        self.writer.writerow([
            'SUMMARY',
            f"{avg_scan_rate:.2f}",
            self.last_objects,
            f"{avg_objects_per_scan:.2f}",
            f"{success_rate:.1f}",
            f"{cpu_ms:.2f}",
        ])
        self.csv_file.flush()
        self.get_logger().info(f'Kész: {self.output} | scan_rate={avg_scan_rate:.2f} Hz, success={success_rate:.1f}%, cpu={cpu_ms:.2f} ms')
        self.csv_file.close()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Élő metrika rögzítő /filtered_scan, /objects, /lidar_filter/cpu_time_ms topicokra.')
    parser.add_argument('--duration', type=float, default=60.0, help='Mérés hossza másodpercben (alap: 60)')
    parser.add_argument('--output', type=str, default='live_metrics.csv', help='Kimeneti CSV fájl neve')
    args = parser.parse_args()

    rclpy.init()
    node = LiveMetricsNode(duration_sec=args.duration, output=args.output)
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'csv_file') and not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
