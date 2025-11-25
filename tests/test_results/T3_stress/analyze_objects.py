#!/usr/bin/env python3
"""
ROS2 Rosbag /objects topic valós idejű elemző - rclpy alapú.

Ez a szkript feliratkozik a /objects topic-ra és valós időben elemzi
a beérkező objektum detektálási üzeneteket. Használható rosbag lejátszás
közben vagy élő rendszer teszteléséhez.

Működés:
1. Terminál 1: ros2 bag play <bag_path>
2. Terminál 2: python3 analyze_objects.py
3. Ctrl+C: Statisztikák megjelenítése

Kimenetek:
- Valós idejű objektumszám megjelenítés
- Objektumszám eloszlás hisztogram
- Sikeres detektálások aránya
- Átlag/min/max statisztikák
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import sys
import signal
from collections import defaultdict

class ObjectsAnalyzer(Node):
    """
    ROS2 Node a /objects topic elemzéséhez.
    
    Feliratkozik a /objects topic-ra és minden beérkező üzenetnél
    összegyűjti a statisztikákat az objektumszámokról.
    """
    
    def __init__(self):
        """Node inicializálása - feliratkozás és számlálók beállítása."""
        super().__init__('objects_analyzer')
        
        # Feliratkozás a /objects topic-ra (PoseArray típusú üzenetek)
        # Queue size = 10: utolsó 10 üzenet bufferelése
        self.subscription = self.create_subscription(
            PoseArray,
            '/objects',
            self.objects_callback,
            10
        )
        
        # Statisztikai számlálók inicializálása
        self.message_count = 0  # Összes beérkezett üzenet
        self.object_counts = []  # Objektumszámok listája (időrendi sorrend)
        self.count_distribution = defaultdict(int)  # Objektumszám -> előfordulás
        
        self.get_logger().info('📊 Objects Analyzer elindítva')
        self.get_logger().info('Várakozás /objects üzenetekre...')
    
    def objects_callback(self, msg):
        """
        /objects topic callback - minden új üzenetnél meghívódik.
        
        A PoseArray üzenet poses mezője tartalmazza a detektált objektumokat.
        Minden Pose egy objektum pozícióját reprezentálja.
        
        Args:
            msg (PoseArray): Beérkező objektum lista a lidar_filter_node-tól
        """
        # Üzenet számláló növelése
        self.message_count += 1
        
        # Objektumszám kinyerése: poses tömb hossza = detektált objektumok száma
        num_objects = len(msg.poses)
        
        # Statisztikák frissítése
        self.object_counts.append(num_objects)  # Időrendi lista
        self.count_distribution[num_objects] += 1  # Eloszlás hisztogram
        
        # Első 5 üzenet részletes loggolása (debug célokra)
        if self.message_count <= 5:
            self.get_logger().info(f'[{self.message_count}] Detektált objektumok: {num_objects}')
        # Minden 10. üzenetnél státusz frissítés
        elif self.message_count % 10 == 0:
            self.get_logger().info(f'[{self.message_count}] Feldolgozott üzenetek...')
    
    def print_statistics(self):
        """
        Összegyűjtött statisztikák kiírása a konzolra.
        
        Megjeleníti:
        - Alapvető statisztikák (átlag, min, max)
        - Objektumszám eloszlás grafikus hisztogram
        - Sikeres detektálások aránya
        """
        print("\n" + "=" * 60)
        print("📈 STATISZTIKÁK")
        print("=" * 60)
        
        # Ellenőrzés: van-e feldolgozott adat
        if not self.object_counts:
            print("❌ Nincs feldolgozott üzenet!")
            return
        
        # Alapvető statisztikák számítása
        avg_objects = sum(self.object_counts) / len(self.object_counts)
        min_objects = min(self.object_counts)
        max_objects = max(self.object_counts)
        
        # Alapstatisztikák kiírása
        print(f"\nÖsszesen üzenetek: {self.message_count}")
        print(f"Átlagos objektumszám/scan: {avg_objects:.2f}")
        print(f"Minimum objektumszám: {min_objects}")
        print(f"Maximum objektumszám: {max_objects}")
        
        # Eloszlás hisztogram kirajzolása szöveges formában
        print("\n📊 Objektumszám eloszlás:")
        for obj_count in sorted(self.count_distribution.keys()):
            freq = self.count_distribution[obj_count]  # Előfordulások száma
            percent = (freq / self.message_count) * 100  # Százalékos arány
            bar = "█" * int(percent / 2)  # Grafikus sáv (50% = 25 karakter)
            print(f"  {obj_count} objektum: {freq:3d} scan ({percent:5.1f}%) {bar}")
        
        # Sikeres detektálások (legalább 1 objektum)
        successful_detections = sum(1 for count in self.object_counts if count > 0)
        success_rate = (successful_detections / self.message_count) * 100
        print(f"\n✅ Sikeres detektálások (> 0 obj): {successful_detections}/{self.message_count} ({success_rate:.1f}%)")

def signal_handler(sig, frame, analyzer):
    """
    SIGINT (Ctrl+C) kezelő függvény.
    
    Amikor a felhasználó Ctrl+C-t nyom, nem csak leáll a program,
    hanem előtte kiírja a összegyűjtött statisztikákat.
    
    Args:
        sig: Signal típus (SIGINT)
        frame: Stack frame
        analyzer: ObjectsAnalyzer instance a statisztikákhoz
    """
    print("\n\n🛑 Elemzés leállítva (Ctrl+C)")
    analyzer.print_statistics()
    rclpy.shutdown()
    sys.exit(0)

def main():
    """
    Főprogram - használati utasítások és node futtatása.
    
    Kiírja a használati utasításokat, inicializálja a ROS2 környezetet,
    létrehozza az analyzer node-ot és futtatja amíg Ctrl+C nem érkezik.
    """
    # Használati utasítások megjelenítése
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
    
    # ROS2 inicializálás
    rclpy.init()
    
    # Analyzer node létrehozása
    analyzer = ObjectsAnalyzer()
    
    # Signal handler regisztrálása Ctrl+C kezeléshez
    # Lambda függvénnyel az analyzer instance továbbítása
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, analyzer))
    
    try:
        # Node futtatása - blokkoló hívás, folyamatosan dolgozza fel az üzeneteket
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        # Ctrl+C kezelése (ha a signal handler nem kapná el)
        pass
    finally:
        # Cleanup - statisztikák kiírása és erőforrások felszabadítása
        analyzer.print_statistics()
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
