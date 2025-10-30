# 📊 Rosbag Részletes Elemzés - Útmutató

**Cél:** T3 v2 rosbag `/objects` topic elemzése - pontosan hány objektum volt detektálva scan-enként

---

## 🎯 Használat

### Módszer 1: Python rclpy Script (AJÁNLOTT)

**1. Terminal - Rosbag lejátszása:**
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash
ros2 bag play test_run_stress_v2 --rate 5.0
```

**2. Második Terminal - Elemzés:**
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress
source /opt/ros/jazzy/setup.bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash
./run_analyze.sh
```

**Vagy egyszerűen:**
```bash
# Terminal 1: bag play (előző terminál)
# Terminal 2:
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress
./run_analyze.sh
```

**Ctrl+C** megnyomásával megáll és kiírja a statisztikákat! 📈

---

### Módszer 2: Manuális Topic Echo

**Terminal 1 - Bag play:**
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
ros2 bag play test_run_stress_v2
```

**Terminal 2 - Topic echo:**
```bash
ros2 topic echo /objects > objects_dump.txt
```

**Ctrl+C** után elemzés:
```bash
# Objektumok számlálása (minden "position:" egy objektum)
grep -c "position:" objects_dump.txt

# Vagy részletesebben:
grep "position:" objects_dump.txt | wc -l
```

---

### Módszer 3: Egy Minta Üzenet Megtekintése

```bash
# Bag lejátszása közben:
ros2 topic echo /objects --once
```

**Output példa:**
```yaml
header:
  stamp:
    sec: 1761848...
    nanosec: ...
  frame_id: odom
poses:
- position:
    x: 2.05
    y: 1.48
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: -2.98
    y: 0.02
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
# ... további objektumok
```

→ **poses tömb hossza = detektált objektumok száma**

---

## 📈 Várható Eredmények

**T3 v2 teszt (manuális spawning):**
- Összesen üzenetek: ~89
- Átlagos objektumszám/scan: ~3-5
- Objektumok eloszlása:
  - 0 objektum: ~1 scan (init)
  - 3 objektum: ~XX scan
  - 4 objektum: ~XX scan
  - 5 objektum: ~XX scan

---

## 🛠️ Szkriptek Leírása

### `analyze_objects.py`
- **Cél:** Valós idejű /objects topic elemzés
- **Működés:** rclpy node feliratkozik, számolja az objektumokat
- **Kimenet:** Statisztikák Ctrl+C után
- **Használat:** `./run_analyze.sh` (wrapper script)

### `run_analyze.sh`
- **Cél:** ROS környezet beállítása és analyze_objects.py futtatása
- **Működés:** Source-olja a ROS-t, elkerüli conda konfliktust
- **Használat:** `./run_analyze.sh`

### `simple_analyze.py`
- **Cél:** Rosbag info kiírása és módszer javaslatok
- **Használat:** `python3 simple_analyze.py rosbag/test_run_stress_v2`

### `analyze_rosbag.py`
- **Cél:** Közvetlen MCAP fájl olvasás (opcionális)
- **Függőség:** mcap library (pip install mcap)
- **Használat:** Speciális esetekben

---

## 🐛 Troubleshooting

### "ImportError: GLIBCXX_3.4.30 not found"
**Probléma:** Conda/Miniconda környezet konfliktus  
**Megoldás:** Használd a `run_analyze.sh` wrapper scriptet (source-olja a ROS-t)

### "WARNING: topic [/objects] does not appear to be published yet"
**Probléma:** Bag nem fut vagy még nem kezdődött el a topic publikálás  
**Megoldás:** 
1. Ellenőrizd hogy a `ros2 bag play` fut
2. Várj pár másodpercet az inicializálásra
3. Használd `--rate 5.0` a gyorsabb lejátszáshoz

### "No such file or directory: rosbag/test_run_stress_v2"
**Probléma:** Rossz working directory  
**Megoldás:** `cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress`

---

## 📁 Output Fájlok

Ha manuális módszert használsz:
- `objects_dump.txt` - Teljes /objects topic dump
- Elemzési eredmények a képernyőn

Ha analyze_objects.py-t használsz:
- Statisztikák a konzolra kerülnek
- Átirányíthatod: `./run_analyze.sh > analysis_results.txt`

---

**Készítette:** GitHub Copilot  
**Kapcsolódó:** ANALYSIS_T3_v2.md, notes_t3.md
