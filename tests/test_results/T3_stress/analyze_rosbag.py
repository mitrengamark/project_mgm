#!/usr/bin/env python3
"""
Rosbag /objects topic közvetlen elemző - SQLite/MCAP alapú.

Ez a szkript közvetlenül olvassa a rosbag fájlokat anélkül, hogy
le kellene játszani őket. Két módszert támogat:
1. SQLite metadata.db olvasás (ROS2 Humble)
2. Közvetlen MCAP fájl olvasás (mcap library-vel)

FIGYELEM: Ez egy kísérleti szkript. A PoseArray üzenetek deserializálása
bonyolult, ezért a szkript az üzenet méretéből becsüli az objektumszámot.

Pontosabb eredményhez használd: analyze_objects.py (rclpy alapú)

Használat:
    python3 analyze_rosbag.py <rosbag_path>
    
Példa:
    python3 analyze_rosbag.py test_run_stress_v2
"""

import sqlite3
import sys
from pathlib import Path
import json
from collections import defaultdict

def analyze_rosbag(bag_path):
    """
    Rosbag /objects topic elemzése SQLite metadata-n keresztül.
    
    ROS2 Humble rosbag formátum:
    - MCAP fájl: Bináris üzenet adatok
    - metadata.db: SQLite adatbázis topic/üzenet információkkal
    
    Ez a módszer a metadata.db-t olvassa és az üzenet méretéből
    becsüli az objektumszámot (nem 100% pontos!).
    
    Args:
        bag_path (str): Rosbag könyvtár elérési útja
    """
    print(f"📊 Rosbag elemzés: {bag_path}")
    print("=" * 60)
    
    # MCAP fájl keresése a rosbag könyvtárban
    bag_dir = Path(bag_path)
    mcap_files = list(bag_dir.glob("*.mcap"))
    
    if not mcap_files:
        print(f"❌ Nem található MCAP fájl: {bag_dir}")
        return
    
    mcap_file = mcap_files[0]
    print(f"✅ MCAP fájl: {mcap_file.name}\n")
    
    try:
        # SQLite metadata.db elérési útja
        db_path = bag_dir / "metadata.db"
        
        # Ha nincs metadata.db, próbáljuk közvetlenül az MCAP-ot
        if not db_path.exists():
            print("⚠️ Nincs metadata.db, próbálom közvetlenül olvasni...")
            analyze_mcap_direct(mcap_file)
            return
        
        # SQLite kapcsolat létrehozása
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        
        # Topic információk lekérése
        cursor.execute("SELECT name, type, serialization_format FROM topics")
        topics = cursor.fetchall()
        
        print("📋 Topicok:")
        for topic in topics:
            print(f"  - {topic[0]} ({topic[1]})")
        
        print()
        
        # /objects üzenetek lekérése (timestamp és nyers adat)
        # JOIN: messages és topics táblák összekapcsolása
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
        
        # Objektumszámok becslése üzenet méretből
        # FIGYELEM: Ez csak approximáció!
        object_counts = []
        
        print("📏 Objektumszám becslése üzenet méretből:")
        print("   (Ez csak approximáció, pontos értékhez használd analyze_objects.py-t)\n")
        
        for idx, (timestamp, data) in enumerate(messages):
            # PoseArray szerializáció CDR (Common Data Representation) formátumban
            # Struktúra: Header (~30-40 byte) + Pose array
            # Egy Pose: position (3x double = 24 byte) + orientation (4x double = 32 byte) = 56 byte
            
            data_size = len(data)  # Üzenet bináris mérete
            
            # Becsült objektumszám = (teljes méret - header) / pose méret
            # Header méret ~50 byte (konzervatív becslés)
            # Egy Pose ~56 byte
            estimated_objects = max(0, (data_size - 50) // 56)
            object_counts.append(estimated_objects)
            
            # Első 5 üzenet részletei (debug)
            if idx < 5:
                print(f"  [{idx+1}] Timestamp: {timestamp}, Data méret: {data_size} byte, Becsült obj: {estimated_objects}")
        
        conn.close()
        
        # Statisztikák számítása és megjelenítése
        print_statistics(object_counts)
        
    except Exception as e:
        print(f"❌ Hiba: {e}")
        import traceback
        traceback.print_exc()

def analyze_mcap_direct(mcap_file):
    """
    Közvetlen MCAP fájl olvasás (ha van mcap library).
    
    Az mcap library lehetővé teszi a rosbag MCAP fájljának
    közvetlen olvasását metadata.db nélkül.
    
    Előny: Nem kell ROS2 környezet
    Hátrány: mcap library telepítése szükséges
    
    Args:
        mcap_file (Path): MCAP fájl elérési útja
    """
    print("⚠️ Próbálom mcap library-vel...")
    try:
        from mcap.reader import make_reader
        
        # MCAP fájl megnyitása
        with open(mcap_file, "rb") as f:
            reader = make_reader(f)
            
            objects_messages = []
            
            # Iterálás a /objects topic üzenetein
            for schema, channel, message in reader.iter_messages(topics=["/objects"]):
                objects_messages.append(message)
            
            print(f"✅ /objects üzenetek: {len(objects_messages)} db")
            
            # Objektumszám becslése (hasonlóan az SQLite módszerhez)
            object_counts = []
            for idx, msg in enumerate(objects_messages[:5]):  # Első 5 példa
                data_size = len(msg.data)
                estimated_objects = max(0, (data_size - 50) // 56)
                object_counts.append(estimated_objects)
                print(f"  [{idx+1}] Data méret: {data_size} byte, Becsült obj: {estimated_objects}")
            
            # Statisztikák az összes üzenetre
            all_counts = []
            for msg in objects_messages:
                data_size = len(msg.data)
                estimated_objects = max(0, (data_size - 50) // 56)
                all_counts.append(estimated_objects)
            
            print_statistics(all_counts)
            
    except ImportError:
        print("❌ mcap library nincs telepítve!")
        print("Telepítsd: pip install mcap mcap-ros2-support")
    except Exception as e:
        print(f"❌ Hiba mcap olvasásnál: {e}")

def print_statistics(object_counts):
    """
    Objektumszám statisztikák kiírása.
    
    Megjeleníti az alapvető statisztikákat és eloszlást az
    objektumszámokból.
    
    Args:
        object_counts (list): Becsült objektumszámok listája
    """
    print("\n" + "=" * 60)
    print("📈 STATISZTIKÁK")
    print("=" * 60)
    
    if not object_counts:
        print("❌ Nincs adat!")
        return
    
    # Alapstatisztikák
    avg_objects = sum(object_counts) / len(object_counts)
    min_objects = min(object_counts)
    max_objects = max(object_counts)
    
    print(f"Összesen üzenetek: {len(object_counts)}")
    print(f"Átlagos objektumszám/scan: {avg_objects:.2f}")
    print(f"Minimum objektumszám: {min_objects}")
    print(f"Maximum objektumszám: {max_objects}")
    
    # Eloszlás hisztogram
    count_dist = defaultdict(int)
    for count in object_counts:
        count_dist[count] += 1
    
    print("\n📊 Objektumszám eloszlás:")
    for obj_count in sorted(count_dist.keys()):
        freq = count_dist[obj_count]
        percent = (freq / len(object_counts)) * 100
        bar = "█" * int(percent / 2)
        print(f"  {obj_count} objektum: {freq:3d} scan ({percent:5.1f}%) {bar}")
    
    # Figyelmeztetés a becslésről
    print("\n⚠️ FIGYELEM: Ezek BECSÜLT értékek!")
    print("   Pontos eredményhez használd: analyze_objects.py")

if __name__ == "__main__":
    """
    Főprogram - parancssor argumentumok és elemzés.
    """
    if len(sys.argv) < 2:
        print("❌ Használat: python3 analyze_rosbag.py <rosbag_path>")
        print("📝 Példa: python3 analyze_rosbag.py test_run_stress_v2")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    analyze_rosbag(bag_path)
