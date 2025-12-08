#!/usr/bin/env python3

"""
Tracking debuggolásra: kiírja az objektum ID-kat, pozíciókat és státuszokat.
Futtatás: python3 test_tracking.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import sys


class TrackingDebugger(Node):
    def __init__(self):
        super().__init__('tracking_debugger')
        
        # Subscribes
        self.objects_sub = self.create_subscription(
            PoseArray,
            '/objects',
            self.objects_callback,
            10
        )
        
        self.labels_sub = self.create_subscription(
            MarkerArray,
            '/object_labels',
            self.labels_callback,
            10
        )
        
        self.frame_count = 0
        print("\n" + "="*70)
        print("🎯 TRACKING DEBUGGER - Objektum nyomkövetés figyelő")
        print("="*70 + "\n")

    def objects_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 10 == 0:  # 10 frame-enként írunk ki
            num_objects = len(msg.poses)
            print(f"\n[Frame {self.frame_count}] Detektált objektumok: {num_objects}")
            
            for i, pose in enumerate(msg.poses):
                x = pose.position.x
                y = pose.position.y
                dist = (x**2 + y**2) ** 0.5
                angle = __import__('math').atan2(y, x) * 180 / __import__('math').pi
                print(f"  [{i}] Pozíció: ({x:.2f}m, {y:.2f}m) | Távolság: {dist:.2f}m | Szög: {angle:.1f}°")

    def labels_callback(self, msg):
        """Kiírja az objektum ID-kat a TEXT markereken keresztül"""
        if len(msg.markers) > 0:
            print(f"\n  📍 ID-k az RViz-ben:")
            for marker in msg.markers:
                obj_id = marker.id
                text = marker.text
                x = marker.pose.position.x
                y = marker.pose.position.y
                print(f"    {text} @ ({x:.2f}m, {y:.2f}m)")


def main(args=None):
    rclpy.init(args=args)
    node = TrackingDebugger()
    
    try:
        print("Figyelés indul... (Ctrl+C a leállításhoz)\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("🛑 Debugger leállt")
        print("="*70 + "\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
