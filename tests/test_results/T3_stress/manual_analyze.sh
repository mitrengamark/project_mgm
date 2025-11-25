#!/bin/bash
# Egyszerű manuális elemzés - topic echo alapú
# Nem használ rclpy-t, így nincs library konfliktus

echo "📊 T3 v2 Rosbag Manuális Elemzés"
echo "================================"
echo ""

BAG_PATH="rosbag/test_run_stress_v2"

if [ ! -d "$BAG_PATH" ]; then
    echo "❌ Nem található: $BAG_PATH"
    exit 1
fi

echo "✅ Rosbag megtalálva: $BAG_PATH"
echo ""
echo "🎬 Bag lejátszása 10x sebességgel és objektumok dump-olása..."
echo ""

# Rosbag info
echo "📋 Rosbag Info:"
ros2 bag info "$BAG_PATH"

echo ""
echo "================================"
echo "📊 /objects Topic Elemzés"
echo "================================"
echo ""
echo "Lejátszom a bag-et és gyűjtöm az objektum adatokat..."
echo "⏳ Ez ~10-15 másodpercig tart..."
echo ""

# Bag lejátszás háttérben, gyorsított
ros2 bag play "$BAG_PATH" --rate 10.0 > /dev/null 2>&1 &
PLAY_PID=$!

# Rövid várakozás az inicializálásra
sleep 2

# Topic echo timeout-tal, kiírás fájlba
timeout 15 ros2 topic echo /objects 2>/dev/null > /tmp/objects_dump.txt

# Play megállítása
kill $PLAY_PID 2>/dev/null
wait $PLAY_PID 2>/dev/null

echo "✅ Adatgyűjtés befejezve!"
echo ""

# Elemzés
if [ ! -f /tmp/objects_dump.txt ]; then
    echo "❌ Nem sikerült az adatgyűjtés!"
    exit 1
fi

TOTAL_LINES=$(wc -l < /tmp/objects_dump.txt)
echo "📄 Összesen sorok: $TOTAL_LINES"

# "---" elválasztók = üzenetek száma
MESSAGES=$(grep -c "^---$" /tmp/objects_dump.txt)
echo "📨 Összesen üzenetek: $MESSAGES"

# "position:" előfordulások = objektumok száma
TOTAL_OBJECTS=$(grep -c "position:" /tmp/objects_dump.txt)
echo "🎯 Összesen objektumok (összes scan): $TOTAL_OBJECTS"

if [ $MESSAGES -gt 0 ]; then
    AVG_OBJECTS=$(awk "BEGIN {printf \"%.2f\", $TOTAL_OBJECTS / $MESSAGES}")
    echo "📊 Átlagos objektumszám/scan: $AVG_OBJECTS"
fi

echo ""
echo "================================"
echo "📈 Részletes Elemzés"
echo "================================"
echo ""

# Python segítségével részletesebb statisztika
python3 << 'PYTHON_SCRIPT'
import sys

try:
    with open('/tmp/objects_dump.txt', 'r') as f:
        content = f.read()
    
    # Üzenetek szétválasztása "---" alapján
    messages = content.split('---\n')[1:]  # Első elem üres
    
    object_counts = []
    
    for msg in messages:
        if not msg.strip():
            continue
        # "position:" előfordulások számlálása
        count = msg.count('position:')
        object_counts.append(count)
    
    if not object_counts:
        print("❌ Nincs feldolgozható üzenet!")
        sys.exit(1)
    
    # Statisztikák
    total_messages = len(object_counts)
    avg_objects = sum(object_counts) / total_messages
    min_objects = min(object_counts)
    max_objects = max(object_counts)
    
    print(f"Összesen üzenetek: {total_messages}")
    print(f"Átlagos objektumszám: {avg_objects:.2f}")
    print(f"Minimum objektumszám: {min_objects}")
    print(f"Maximum objektumszám: {max_objects}")
    
    # Eloszlás
    from collections import Counter
    distribution = Counter(object_counts)
    
    print("\n📊 Objektumszám Eloszlás:")
    for obj_count in sorted(distribution.keys()):
        freq = distribution[obj_count]
        percent = (freq / total_messages) * 100
        bar = "█" * int(percent / 2)
        print(f"  {obj_count} objektum: {freq:3d} scan ({percent:5.1f}%) {bar}")
    
    # Sikeres detektálások
    successful = sum(1 for c in object_counts if c > 0)
    success_rate = (successful / total_messages) * 100
    print(f"\n✅ Sikeres detektálások (> 0 obj): {successful}/{total_messages} ({success_rate:.1f}%)")
    
    # CSV export
    print("\n💾 CSV export: /tmp/t3_objects_analysis.csv")
    with open('/tmp/t3_objects_analysis.csv', 'w') as csv_file:
        csv_file.write("scan_id,object_count\n")
        for i, count in enumerate(object_counts, 1):
            csv_file.write(f"{i},{count}\n")
    
    print("   Használd grafikonokhoz!")

except Exception as e:
    print(f"❌ Hiba: {e}")
    import traceback
    traceback.print_exc()
PYTHON_SCRIPT

echo ""
echo "✅ Elemzés befejezve!"
echo ""
echo "📁 Generált fájlok:"
echo "   - /tmp/objects_dump.txt (teljes dump)"
echo "   - /tmp/t3_objects_analysis.csv (CSV adatok)"
