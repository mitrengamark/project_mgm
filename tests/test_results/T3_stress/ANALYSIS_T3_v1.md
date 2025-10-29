# T3 Teszt Elemzés - Első Kísérlet (Sikertelen Spawning)

**Dátum:** 2025-10-29 23:56  
**Teszt típus:** Stresszteszt (folyamatos objektum spawning)  
**Státusz:** ❌ Sikertelen - spawning nem működött

---

## 📊 Gyors Összefoglaló

| Metrika | Érték | T2 Baseline | Különbség |
|---------|-------|-------------|-----------|
| **Teszt időtartam** | 61.08 sec | 276.7 sec | -78% (rövidebb) |
| **Bag méret** | 2.7 MiB | 15.2 MiB | -82% |
| **Scan rate** | 0.72 Hz | 0.86 Hz | -16% |
| **Objektumok** | 0 | 1-3 | N/A |
| **Detektálás** | 42/42 (100%)* | 237/238 (99.6%) | *üres scan |

---

## 🔍 Probléma Azonosítása

### Spawning Script Hibái

**1. `Package 'gazebo_ros' not found`**
```bash
ros2 run gazebo_ros spawn_entity.py -entity box1 -database unit_box -x 2.0 -y 1.5 -z 0.5
Package 'gazebo_ros' not found
```

**Kiváltó ok:**
- A `spawn_objects.sh` script NEM source-olta a ROS környezetet
- Csak a felhasználói terminál volt source-olva, de a script futtatásakor új shell indult
- A script-ben hiányzott: `source /opt/ros/jazzy/setup.bash` vagy workspace setup

**Megoldás:**
- Script módosítása hogy tartalmazza a setup.bash source-olást
- VAGY: Felhasználó manuálisan source-olja a terminált mielőtt futtatja a scriptet

---

**2. Script Azonnal Leáll**

A `spawn_objects.sh` script egymás után spawn-olt 7 objektumot, majd **azonnal kilépett**.

```bash
[1/7] Box spawning (2.0, 1.5, 0.5)...
[2/7] Cylinder spawning (2.0, -1.5, 0.5)...
...
[7/7] Box spawning (2.5, 0.0, 0.5)...

============================================
  Spawning befejezve!
  Létrehozott objektumok: 7
============================================
```

**Probléma:**
- Mire a felhasználó elindította a rosbag rögzítést (Terminal 3), a script már régen befejeződött
- Objektumok NEM voltak a világban (ha sikeres lett volna a spawn)
- Nincs folyamatosság - egyszeri batch spawn

---

## 💡 Megoldás: Folyamatos Spawning

### Új Script: `continuous_spawn.sh`

**Jellemzők:**
- ♾️ **Végtelen ciklus** - Ctrl+C-ig fut
- 🎲 **Random objektumok** - Box vagy Cylinder véletlenszerűen
- ⏱️ **5-10 sec élettartam** - Objektum automatikusan törlődik
- 🔄 **1-3 sec spawn intervallum** - Új objektum késéssel
- 📍 **12 pozíció** - Robot körül 2-4m távolságban
- ✅ **Gazebo ellenőrzés** - Script leáll ha Gazebo nem fut

**Előnyök:**
1. **Folyamatos működés** - Rosbag rögzítés alatt is spawn-ol
2. **Valódi stresszteszt** - Változó objektumszám (2-5 egyidejűleg)
3. **Dinamikus környezet** - Objektumok jelennek meg és tűnnek el
4. **Detektálás tesztelése** - Rendszer követi a változásokat

**Használat:**
```bash
cd tests/test_results/T3_stress
source /home/mark/codes/mgm/project_mgm/install/setup.bash
./continuous_spawn.sh
```

Kilépés: **Ctrl+C**

---

## 📈 Rosbag Eredmények (Objektumok Nélkül)

### Topic Frekvenciák

```
Duration: 61.08 sec
Messages: 8466

/scan:           44 msg  (0.72 Hz)
/filtered_scan:  43 msg  (0.70 Hz)
/objects:        42 msg  (0.69 Hz)
/object_markers: 42 msg  (0.69 Hz)
/odom:          218 msg  (3.57 Hz)
/tf:           8077 msg (132.3 Hz)
/cmd_vel:         0 msg  (statikus robot)
```

**Megfigyelések:**
- **Scan rate csökkent:** 0.72 Hz (T2: 0.86 Hz) - valószínűleg csak természetes variáció
- **Detektálás működött:** 42 /objects üzenet - de minden üzenet **üres PoseArray** (0 objektum)
- **Bag méret kicsi:** 2.7 MiB (T2: 15.2 MiB) - rövidebb futás miatt

---

## 🎯 Következő Lépések - T3 v2 Teszt

### Teszt Terv

1. **Terminal 1:** `ros2 launch lidar_filter optimized_system.launch.py`
2. **Várakozás:** 10-15 másodperc (Gazebo init)
3. **Terminal 2:** 
   ```bash
   cd tests/test_results/T3_stress
   source /home/mark/codes/mgm/project_mgm/install/setup.bash
   ./continuous_spawn.sh
   ```
4. **Terminal 3:** Rosbag rögzítés indítása (spawn script futása közben!)
5. **Futtatás:** 90-120 másodperc
6. **Leállítás:** Terminal 3 (Ctrl+C) → Terminal 2 (Ctrl+C) → Terminal 1 (Ctrl+C)

### Várható Eredmények

- **Objektumok:** 2-5 egyidejűleg (folyamatosan változik)
- **Detektálás:** Követi az objektum spawn/delete ciklusokat
- **Scan rate:** ~0.7-0.9 Hz (hasonló T2-höz)
- **Bag méret:** ~5-8 MiB (90-120 sec futás)
- **RViz:** Markerek megjelennek/eltűnnek dinamikusan

### Sikerkritériumok

- ✅ Objektumok láthatók Gazebo-ban
- ✅ Objektumok láthatók RViz-ben (markerek)
- ✅ /objects topic tartalmaz PoseArray adatokat (nem üres)
- ✅ Detektálás követi a spawn/delete ciklusokat
- ✅ Nincs crash vagy node restart
- ✅ Rosbag teljes minden topic-kal

---

## 🔧 További Javítások

### spawn_objects.sh Módosítása

Ha az eredeti batch script-et szeretnénk használni (7 objektum egyszerre):

```bash
#!/bin/bash

# ROS környezet source-olása
source /opt/ros/jazzy/setup.bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# ... spawn parancsok ...

# FONTOS: Ne lépjen ki azonnal!
echo ""
echo "Spawning befejezve. Nyomj ENTER-t a script leállításához..."
read -r

# Vagy alternatíva: várakozás X másodpercig
# echo "Várakozás 120 másodpercig..."
# sleep 120
```

---

## 📝 Tanulságok

1. **Script source-olás kritikus** - Bash script-ekben is source-olni kell a ROS-t
2. **Timing fontos** - Spawn és rosbag rögzítés szinkronizálása
3. **Folyamatos tesztelés jobb** - Batch spawn helyett folyamatos spawn dinamikusabb
4. **Ellenőrzés először** - Spawning sikerességét validálni kell mielőtt tovább lépünk
5. **Dokumentáció értékes** - Hibák rögzítése segíti a következő kísérletet

---

**Státusz:** T3 v1 sikertelen, de értékes tanulságok! 🔧  
**Következő:** T3 v2 folyamatos spawning-gal 🚀
