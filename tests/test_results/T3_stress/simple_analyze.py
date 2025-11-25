#!/usr/bin/env python3
"""
Rosbag egyszerű elemző és útmutató - ROS2 topic echo alapú elemzés.

Ez a szkript NEM elemzi közvetlenül a rosbag-et, hanem:
1. Kiírja a rosbag alapvető információit (ros2 bag info)
2. Útmutatót ad különböző elemzési módszerekhez
3. Segít megérteni hogyan lehet elemezni a /objects topic-ot

Hasznos ha nem vagy biztos hogyan kezdd el az elemzést, vagy
szeretnéd látni milyen eszközök állnak rendelkezésre.

Használat:
    python3 simple_analyze.py <rosbag_path>
    
Példa:
    python3 simple_analyze.py rosbag/test_run_stress_v2
"""

import subprocess
import sys
import time
import signal
from pathlib import Path

def analyze_rosbag_simple(bag_path):
    """
    Rosbag alapvető információinak lekérése és megjelenítése.
    
    A ros2 bag info parancsot futtatja, amely kiírja:
    - Bag időtartamát
    - Topic-ok listáját
    - Üzenetek számát topic-onként
    - Üzenet típusokat
    
    Args:
        bag_path (str): Rosbag könyvtár elérési útja
    """
    print(f"📊 Rosbag egyszerű elemzés: {bag_path}")
    print("=" * 60)
    
    # Ellenőrzés: létezik-e a rosbag könyvtár
    bag_dir = Path(bag_path)
    if not bag_dir.exists():
        print(f"❌ Nem található: {bag_dir}")
        return
    
    # ros2 bag info parancs futtatása
    # Ez kiírja a bag metaadatait (topic-ok, üzenetek száma, stb.)
    print("🎬 Rosbag info lekérése...")
    info_cmd = f"ros2 bag info {bag_path}"
    result = subprocess.run(info_cmd, shell=True, capture_output=True, text=True)
    
    # Info kimenet megjelenítése
    print(result.stdout)
    
    # Hibaellenőrzés
    if result.stderr:
        print("⚠️ Figyelmeztetések/Hibák:")
        print(result.stderr)
    
    # Útmutató megjelenítése
    print("\n" + "=" * 60)
    print("📋 /objects topic részletes elemzése")
    print("=" * 60)
    print("\n⚠️ FIGYELEM: Ez a metódus a bag lejátszását igényli!")
    print("Futtasd manuálisan:")
    print(f"\n1. Terminal 1:")
    print(f"   ros2 bag play {bag_path}")
    print(f"\n2. Terminal 2:")
    print(f"   ros2 topic echo /objects")
    print(f"\nVagy használd a Python ROS2 API-t (rclpy):\n")

def suggest_analysis_methods():
    """
    Különböző rosbag elemzési módszerek bemutatása.
    
    Ez a függvény részletes útmutatót ad 4 különböző megközelítéshez:
    1. Manuális terminal alapú elemzés
    2. Python rclpy könyvtár használata
    3. mcap library (alacsony szintű bag olvasás)
    4. ROS2 topic statisztikák
    """
    print("=" * 60)
    print("🔧 RÉSZLETES ELEMZÉSI MÓDSZEREK")
    print("=" * 60)
    
    # 1. Manuális terminal módszer
    print("\n1️⃣ **Manuális Módszer (Terminal):**")
    print("""
    # Terminal 1 - Bag play (rosbag lejátszása)
    ros2 bag play test_run_stress_v2
    
    # Terminal 2 - Topic echo (minden üzenet kimentése fájlba)
    ros2 topic echo /objects > objects_dump.txt
    
    # Elemzés (objektumok számolása a kimeneti fájlból)
    grep -c "position:" objects_dump.txt  # Objektumok számlálása
    """)
    
    # 2. Python rclpy módszer
    print("\n2️⃣ **Python rclpy Módszer:**")
    print("""
    # Python script ami feliratkozik és számolja
    import rclpy
    from geometry_msgs.msg import PoseArray
    
    # Feliratkozás /objects-ra
    # Minden üzenetben len(msg.poses) = objektumszám
    # Lásd: analyze_objects.py (komplett példa)
    """)
    
    # 3. mcap library módszer
    print("\n3️⃣ **mcap Library Módszer:**")
    print("""
    # mcap: Alacsony szintű rosbag olvasás (Python library)
    pip install mcap mcap-ros2-support
    python3 analyze_with_mcap.py test_run_stress_v2
    
    # Előny: Nem kell lejátszani a bag-et, közvetlenül olvasható
    # Hátrány: Komplex API, üzenet deserializáció szükséges
    """)
    
    # 4. ROS2 topic statisztikák
    print("\n4️⃣ **ros2 topic Statisztika:**")
    print("""
    # Bag lejátszása közben valós idejű statisztikák
    ros2 bag play test_run_stress_v2 &
    
    # Topic frekvencia (Hz) mérése
    ros2 topic hz /objects
    
    # Egy üzenet mintája (struktúra megismerése)
    ros2 topic echo /objects --once
    
    # Bandwidth mérés
    ros2 topic bw /objects
    """)

if __name__ == "__main__":
    """
    Főprogram - parancssor argumentumok kezelése és elemzés indítása.
    """
    # Parancssor argumentum ellenőrzés
    if len(sys.argv) < 2:
        print("❌ Használat: python3 simple_analyze.py <rosbag_path>")
        print("📝 Példa: python3 simple_analyze.py rosbag/test_run_stress_v2")
        sys.exit(1)
    
    # Rosbag elérési út kinyerése
    bag_path = sys.argv[1]
    
    # Egyszerű elemzés futtatása
    analyze_rosbag_simple(bag_path)
    
    # Elemzési módszerek bemutatása
    print()
    suggest_analysis_methods()
