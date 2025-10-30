#!/usr/bin/env python3
"""
Rosbag /objects elemző - rclpy alapú
Lejátssza a bag-et és valós időben elemzi az objektumszámokat
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import sys
import signal
from collections import defaultdict

class ObjectsAnalyzer(Node):
    def __init__(self):
        super().__init__('objects_analyzer')
        
        self.subscription = self.create_subscription(
            PoseArray,
            '/objects',
            self.objects_callback,
            10
        )
        
        self.message_count = 0
        self.object_counts = []
        self.count_distribution = defaultdict(int)
        
        self.get_logger().info('📊 Objects Analyzer elindítva')
        self.get_logger().info('Várakozás /objects üzenetekre...')
    
    def objects_callback(self, msg):
        """
        /objects topic callback - PoseArray üzenet
        """
        self.message_count += 1
        num_objects = len(msg.poses)
        self.object_counts.append(num_objects)
        self.count_distribution[num_objects] += 1
        
        if self.message_count <= 5:
            self.get_logger().info(f'[{self.message_count}] Detektált objektumok: {num_objects}')
        elif self.message_count % 10 == 0:
            self.get_logger().info(f'[{self.message_count}] Feldolgozott üzenetek...')
    
    def print_statistics(self):
        """
        Statisztikák kiírása
        """
        print("\n" + "=" * 60)
        print("📈 STATISZTIKÁK")
        print("=" * 60)
        
        if not self.object_counts:
            print("❌ Nincs feldolgozott üzenet!")
            return
        
        avg_objects = sum(self.object_counts) / len(self.object_counts)
        min_objects = min(self.object_counts)
        max_objects = max(self.object_counts)
        
        print(f"\nÖsszesen üzenetek: {self.message_count}")
        print(f"Átlagos objektumszám/scan: {avg_objects:.2f}")
        print(f"Minimum objektumszám: {min_objects}")
        print(f"Maximum objektumszám: {max_objects}")
        
        print("\n📊 Objektumszám eloszlás:")
        for obj_count in sorted(self.count_distribution.keys()):
            freq = self.count_distribution[obj_count]
            percent = (freq / self.message_count) * 100
            bar = "█" * int(percent / 2)
            print(f"  {obj_count} objektum: {freq:3d} scan ({percent:5.1f}%) {bar}")
        
        # Sikeres detektálások (> 0 objektum)
        successful_detections = sum(1 for count in self.object_counts if count > 0)
        success_rate = (successful_detections / self.message_count) * 100
        print(f"\n✅ Sikeres detektálások (> 0 obj): {successful_detections}/{self.message_count} ({success_rate:.1f}%)")

def signal_handler(sig, frame, analyzer):
    """
    Ctrl+C kezelés - statisztikák kiírása
    """
    print("\n\n🛑 Elemzés leállítva (Ctrl+C)")
    analyzer.print_statistics()
    rclpy.shutdown()
    sys.exit(0)

def main():
    print("=" * 60)
    print("🔍 Rosbag /objects Elemző")
    print("=" * 60)
    print("\n⚠️ FONTOS:")
    print("1. Először indítsd el a rosbag lejátszását:")
    print("   ros2 bag play <bag_path>")
    print("\n2. Majd futtasd ezt a scriptet")
    print("\n3. Ctrl+C a statisztikák kiírásához\n")
    print("Várakozás /objects topic-ra...")
    print("=" * 60)
    
    rclpy.init()
    analyzer = ObjectsAnalyzer()
    
    # Signal handler Ctrl+C-hez
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, analyzer))
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.print_statistics()
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
