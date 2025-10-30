#!/usr/bin/env python3
"""
Rosbag elemző script - T3 teszt /objects topic elemzése
Meghatározza hogy scan-enként hány objektum volt detektálva
"""

import sqlite3
import sys
from pathlib import Path
import json
from collections import defaultdict

def analyze_rosbag(bag_path):
    """
    Elemzi a rosbag /objects topic-ját és statisztikákat készít
    """
    print(f"📊 Rosbag elemzés: {bag_path}")
    print("=" * 60)
    
    # MCAP fájl megkeresése
    bag_dir = Path(bag_path)
    mcap_files = list(bag_dir.glob("*.mcap"))
    
    if not mcap_files:
        print(f"❌ Nem található MCAP fájl: {bag_dir}")
        return
    
    mcap_file = mcap_files[0]
    print(f"✅ MCAP fájl: {mcap_file.name}\n")
    
    try:
        # MCAP formátum sqlite3 metadata-val
        db_path = bag_dir / "metadata.db"
        
        if not db_path.exists():
            print("⚠️ Nincs metadata.db, próbálom közvetlenül olvasni...")
            analyze_mcap_direct(mcap_file)
            return
            
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        
        # Topic információk
        cursor.execute("SELECT name, type, serialization_format FROM topics")
        topics = cursor.fetchall()
        
        print("📋 Topicok:")
        for topic in topics:
            print(f"  - {topic[0]} ({topic[1]})")
        
        print()
        
        # /objects üzenetek elemzése
        cursor.execute("""
            SELECT m.timestamp, m.data 
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = '/objects'
            ORDER BY m.timestamp
        """)
        
        messages = cursor.fetchall()
        print(f"✅ /objects üzenetek: {len(messages)} db\n")
        
        if len(messages) == 0:
            print("❌ Nincs /objects üzenet a rosbag-ben!")
            conn.close()
            return
        
        # Objektumszámok statisztikája
        object_counts = []
        
        for idx, (timestamp, data) in enumerate(messages):
            # PoseArray struktúra: header + poses tömb
            # Egyszerű számolás: poses tömb hossza
            # (CDR szerializációval bonyolult lenne full parse)
            
            # Egyszerűsített becslés: minden üzenet objektumszámát becsüljük
            # A data mérete alapján (approximáció)
            data_size = len(data)
            
            # PoseArray poses: minden Pose ~56 byte (position 3*8 + orientation 4*8)
            # Header ~30-40 byte
            # Becsült objektumszám: (data_size - 50) / 56
            
            estimated_objects = max(0, (data_size - 50) // 56)
            object_counts.append(estimated_objects)
            
            if idx < 5:  # Első 5 üzenet részletei
                print(f"  [{idx+1}] Timestamp: {timestamp}, Data méret: {data_size} byte, Becsült obj: {estimated_objects}")
        
        conn.close()
        
        # Statisztikák
        print("\n" + "=" * 60)
        print("📈 STATISZTIKÁK")
        print("=" * 60)
        
        if object_counts:
            avg_objects = sum(object_counts) / len(object_counts)
            min_objects = min(object_counts)
            max_objects = max(object_counts)
            
            print(f"Összesen üzenetek: {len(object_counts)}")
            print(f"Átlagos objektumszám/scan: {avg_objects:.2f}")
            print(f"Minimum objektumszám: {min_objects}")
            print(f"Maximum objektumszám: {max_objects}")
            
            # Eloszlás
            count_dist = defaultdict(int)
            for count in object_counts:
                count_dist[count] += 1
            
            print("\n📊 Objektumszám eloszlás:")
            for obj_count in sorted(count_dist.keys()):
                freq = count_dist[obj_count]
                percent = (freq / len(object_counts)) * 100
                bar = "█" * int(percent / 2)
                print(f"  {obj_count} objektum: {freq:3d} scan ({percent:5.1f}%) {bar}")
        
    except Exception as e:
        print(f"❌ Hiba: {e}")
        import traceback
        traceback.print_exc()

def analyze_mcap_direct(mcap_file):
    """
    Közvetlenül MCAP fájl olvasása (ha van mcap library)
    """
    print("⚠️ Próbálom mcap library-vel...")
    try:
        from mcap.reader import make_reader
        
        with open(mcap_file, "rb") as f:
            reader = make_reader(f)
            
            objects_messages = []
            
            for schema, channel, message in reader.iter_messages(topics=["/objects"]):
                objects_messages.append(message)
            
            print(f"✅ /objects üzenetek: {len(objects_messages)} db")
            
            # Egyszerű statisztika data méret alapján
            for idx, msg in enumerate(objects_messages[:5]):
                data_size = len(msg.data)
                estimated_objects = max(0, (data_size - 50) // 56)
                print(f"  [{idx+1}] Data méret: {data_size} byte, Becsült obj: {estimated_objects}")
            
    except ImportError:
        print("❌ mcap library nincs telepítve!")
        print("Telepítsd: pip install mcap")
    except Exception as e:
        print(f"❌ Hiba mcap olvasásnál: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Használat: python3 analyze_rosbag.py <rosbag_path>")
        print("Példa: python3 analyze_rosbag.py test_run_stress_v2")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    analyze_rosbag(bag_path)
