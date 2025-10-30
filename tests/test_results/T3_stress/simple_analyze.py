#!/usr/bin/env python3
"""
Rosbag elemző script - ROS2 topic echo alapú elemzés
Lejátssza a bag-et és elemzi az /objects üzeneteket
"""

import subprocess
import sys
import time
import signal
from pathlib import Path

def analyze_rosbag_simple(bag_path):
    """
    Egyszerű elemzés ros2 bag play + topic echo kombinációval
    """
    print(f"📊 Rosbag egyszerű elemzés: {bag_path}")
    print("=" * 60)
    
    bag_dir = Path(bag_path)
    if not bag_dir.exists():
        print(f"❌ Nem található: {bag_dir}")
        return
    
    print("🎬 Rosbag info lekérése...")
    info_cmd = f"ros2 bag info {bag_path}"
    result = subprocess.run(info_cmd, shell=True, capture_output=True, text=True)
    
    print(result.stdout)
    
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
    Javaslatok részletes elemzéshez
    """
    print("=" * 60)
    print("🔧 RÉSZLETES ELEMZÉSI MÓDSZEREK")
    print("=" * 60)
    
    print("\n1️⃣ **Manuális Módszer (Terminal):**")
    print("""
    # Terminal 1 - Bag play
    ros2 bag play test_run_stress_v2
    
    # Terminal 2 - Topic echo (minden üzenet)
    ros2 topic echo /objects > objects_dump.txt
    
    # Elemzés
    grep -c "position:" objects_dump.txt  # Objektumok számlálása
    """)
    
    print("\n2️⃣ **Python rclpy Módszer:**")
    print("""
    # Python script ami feliratkozik és számolja
    import rclpy
    from geometry_msgs.msg import PoseArray
    
    # Feliratkozás /objects-ra
    # Minden üzenetben len(msg.poses) = objektumszám
    """)
    
    print("\n3️⃣ **mcap Library Módszer:**")
    print("""
    pip install mcap mcap-ros2-support
    python3 analyze_with_mcap.py test_run_stress_v2
    """)
    
    print("\n4️⃣ **ros2 topic Statisztika:**")
    print("""
    # Bag lejátszása közben
    ros2 bag play test_run_stress_v2 &
    
    # Topic frekvencia
    ros2 topic hz /objects
    
    # Egy üzenet mintája
    ros2 topic echo /objects --once
    """)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Használat: python3 simple_analyze.py <rosbag_path>")
        print("Példa: python3 simple_analyze.py rosbag/test_run_stress_v2")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    analyze_rosbag_simple(bag_path)
    print()
    suggest_analysis_methods()
