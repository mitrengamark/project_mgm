# 🚀 Futtatási útmutató - MGM Projekt

**Projekt:** LIDAR alapú objektum detektálás és követés  
**Készítette:** Mitrenga Márk  
**Utolsó frissítés:** 2025. október 30. 19:30  
**Verzió:** 3.1 (T3 Stresszteszt eredményekkel)

---

## 📋 Előfeltételek

- ✅ ROS 2 Jazzy telepítve
- ✅ Ubuntu / WSL Linux környezet
- ✅ Workspace lefordítva (`colcon build` sikeres)
- ⚠️ **FONTOS:** Ne aktiválj conda/virtualenv környezetet!

---

## 🎯 Gyors indítás - OPTIMALIZÁLT RENDSZER! 🚀

**⭐ AJÁNLOTT - Optimalizált verzió (v2):**

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

**Ez elindítja:**
- ✅ Gazebo szimulációt TurtleBot3-mal
- ✅ LIDAR Filter Node-ot (237 objektum detektálás, 99.6% siker!)
- ✅ RViz2-t **optimalizált** konfigurációval (csak 3 TF frame, Map nélkül)

**Alternatív - Eredeti verzió (v1):**

```bash
ros2 launch lidar_filter complete_system.launch.py
```

**Különbségek v1 vs v2:**
- v1: Több TF frame, Map display (warning), eredeti RViz config
- v2: Csak 3 TF frame (odom, base_link, base_scan), Map nélkül, tisztább vizualizáció ✅

---

## 🔧 Részletes indítás (3 terminál)

Ha külön-külön szeretnéd indítani a komponenseket debug célra:

### Terminal 1️⃣ - Gazebo szimuláció

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Várt eredmény:** Gazebo ablak megnyílik TurtleBot3 robottal

---

### Terminal 2️⃣ - LIDAR Filter Node

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
ros2 run lidar_filter lidar_filter_node
```

**Vagy közvetlenül:**
```bash
~/codes/mgm/project_mgm/install/lidar_filter/lib/lidar_filter/lidar_filter_node
```

**Várt eredmény:**
```
[INFO] [lidar_filter_node]: LIDAR Filter Node initialized
```

**Publikált topicok:**
- `/filtered_scan` - Szűrt LIDAR adatok (LaserScan)
- `/objects` - Detektált objektumok (PoseArray) - 237 objektum a T2 tesztben ✅
- `/object_markers` - Vizualizációs markerek (MarkerArray)
- `/map` - Térképadatok (OccupancyGrid)

**⚠️ FONTOS:** A node már a `lib/lidar_filter/` mappában van (setup.cfg javítás után)!

---

### Terminal 3️⃣ - RViz2 vizualizáció

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
ros2 run rviz2 rviz2
```

**RViz2 beállítások:**

1. **Fixed Frame** beállítása:
   - Bal oldali panel → Global Options → Fixed Frame: `odom` vagy `base_scan`

2. **Display-ek hozzáadása** (Add gomb):
   - **LaserScan** → Topic: `/scan` (nyers LIDAR)
     - Color: piros
   - **LaserScan** → Topic: `/filtered_scan` (szűrt LIDAR)
     - Color: zöld
   - **MarkerArray** → Topic: `/object_markers` (detektált objektumok)
   - **PoseArray** → Topic: `/objects` (objektum pozíciók)
   - **Map** → Topic: `/map` (térképadatok)

3. **Kamera nézet beállítása:**
   - Egérrel forgathatsz és zoomolhatsz
   - Shift+görgő: pásztázás

**Mentés:** File → Save Config As... → `rviz_config.rviz`

---

## 🎮 Robot mozgatása (opcionális - 4. terminál)

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Irányítás:**
- `w` - előre
- `a` - balra forgás
- `d` - jobbra forgás
- `x` - hátra
- `s` - megállás
- `q` / `z` - sebesség állítás

---

## 📊 Topicok ellenőrzése

### Összes topic listázása:
```bash
ros2 topic list
```

### Konkrét topic tartalmának megtekintése:
```bash
# LIDAR adatok
ros2 topic echo /scan

# Detektált objektumok
ros2 topic echo /objects

# Topic frekvencia mérése
ros2 topic hz /scan
ros2 topic hz /objects
```

### Node-ok és kapcsolatok vizualizálása:
```bash
rqt_graph
```

---

## 💾 Adatrögzítés (rosbag)

### Minden topic rögzítése:
```bash
mkdir -p ~/codes/mgm/project_mgm/results
cd ~/codes/mgm/project_mgm/results
ros2 bag record -a -o test_run1
```

### Csak fontos topicok rögzítése (AJÁNLOTT):
```bash
# ⚠️ FONTOS: Helyes topic nevek!
ros2 bag record /scan /filtered_scan /objects /object_markers /odom /tf /cmd_vel -o test_run2
```

**⚠️ NE használd ezeket a neveket:**
- ❌ `/scan_filtered` → ✅ `/filtered_scan`
- ❌ `/detected_objects` → ✅ `/objects`
- ❌ `/markers` → ✅ `/object_markers`

### Rosbag visszajátszása:
```bash
ros2 bag play test_run1 --clock
```

### Rosbag információk:
```bash
ros2 bag info test_run1
```

**Teszt eredmények összefoglalója:**

**T2 v2 (mozgó robot):**
- Időtartam: 276.7 sec (~4.6 perc)
- Méret: 15.2 MiB
- Scan rate: 0.86 Hz
- Detektálás: 237/238 (99.6%)

**T3 v2 (statikus robot, stresszteszt):**
- Időtartam: 81.7 sec (~1.4 perc)
- Méret: 1.3 MiB
- Scan rate: 1.11 Hz (+29% vs T2!) 🚀
- Detektálás: 89/90 (98.9%)
- Objektumok: ~3-5 egyidejűleg
- Összes üzenet: 50,338
- Detektált objektumok: 237 (99.6% siker!)

**Tipp:** Használd a `--loop` flag-et ismétlődő lejátszáshoz

---

## 🛑 Leállítás

1. **Terminal 4 (teleop):** `Ctrl+C`
2. **Terminal 3 (RViz2):** Bezárás vagy `Ctrl+C`
3. **Terminal 2 (lidar_filter):** `Ctrl+C`
4. **Terminal 1 (Gazebo):** `Ctrl+C`

Vagy minden ROS folyamat gyors leállítása:
```bash
killall -9 gzserver gzclient ruby rviz2
```

---

## 🐛 Hibaelhárítás

### Probléma: "Package 'lidar_filter' not found"
**Megoldás:**
```bash
cd ~/codes/mgm/project_mgm
rm -rf build/lidar_filter install/lidar_filter
colcon build --packages-select lidar_filter
source install/setup.bash
```

### Probléma: "No executable found"
**Megoldás 1:** Rebuild a setup.cfg-vel:
```bash
cd ~/codes/mgm/project_mgm
rm -rf build/lidar_filter install/lidar_filter
colcon build --packages-select lidar_filter
source install/setup.bash
```

**Megoldás 2:** Közvetlenül futtatás (új hely!):
```bash
# ✅ setup.cfg után (HELYES):
~/codes/mgm/project_mgm/install/lidar_filter/lib/lidar_filter/lidar_filter_node

# ❌ régi hely (már nem itt van):
# ~/codes/mgm/project_mgm/install/lidar_filter/bin/lidar_filter_node
```

### Probléma: "ModuleNotFoundError: No module named 'catkin_pkg'"
**Megoldás:** Ne használj conda/virtualenv környezetet!
```bash
conda deactivate
# vagy
deactivate
```

### Probléma: Gazebo nem nyílik meg (WSL)
**Megoldás:** VcXsrv / X11 kell Windows-on
```bash
export DISPLAY=:0
```

### Probléma: "/scan topic does not exist"
**Megoldás:** 
1. Ellenőrizd, hogy Gazebo fut-e
2. Ellenőrizd: `export TURTLEBOT3_MODEL=waffle`
3. Várj 10-15 másodpercet a Gazebo inicializálására

---

## 📂 Fájlstruktúra

```
project_mgm/
├── src/mgm_gyak/lidar_filter/          # LIDAR filter csomag
│   ├── lidar_filter/
│   │   ├── lidar_filter_node.py        # Fő node (99.6% detektálási siker!)
│   │   └── __init__.py
│   ├── launch/
│   │   ├── lidar_filter.launch.py      # Egyszerű launch
│   │   ├── complete_system.launch.py   # Teljes rendszer (v1)
│   │   └── optimized_system.launch.py  # ⭐ Optimalizált rendszer (v2) AJÁNLOTT!
│   ├── config/
│   │   ├── lidar_filter_rviz.rviz      # RViz config (eredeti)
│   │   └── lidar_filter_optimized.rviz # ⭐ Optimalizált (3 TF frame)
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg                       # ✅ ÚJ! Script telepítési helyek
├── install/lidar_filter/               # Build kimenet
│   ├── lib/lidar_filter/               # ✅ Node itt van (javítva!)
│   │   └── lidar_filter_node           # Executable (ros2 run működik!)
│   └── bin/                            # Symlink (régi hely)
├── tests/test_results/                 # Tesztek
│   ├── T1_static/                      # ✅ T1 kész
│   └── T2_moving/                      # ✅ T2 kész (v2: 237 objektum!)
│       ├── rosbag/
│       │   ├── test_run_moving/        # v1 (hiányos)
│       │   └── test_run_moving_v2/     # ⭐ v2 (teljes, 15.2 MiB)
│       ├── notes_t2_v2.md              # Részletes jegyzet
│       ├── README_T2_OPTIMIZED.md      # Optimalizált útmutató
│       └── ANALYSIS_T2_v2.md           # Eredmény elemzés
└── docs/
    ├── TODO_MitrengaMark.md            # Feladatlista (frissítve)
    └── FUTTATAS_UTMUTATO.md            # Ez a fájl (v3.0)
```

---

## 🔍 Hasznos parancsok

```bash
# Környezet ellenőrzése
echo $ROS_DISTRO                 # -> jazzy
which python3                    # -> /usr/bin/python3 (NEM miniconda!)

# Build és újrafordítás
colcon build --symlink-install   # Teljes build
colcon build --packages-select lidar_filter  # Csak egy csomag

# Node információk
ros2 node list                   # Futó node-ok
ros2 node info /lidar_filter_node

# Topic információk
ros2 topic list                  # Összes topic
ros2 topic info /scan            # Topic részletei
ros2 topic bw /scan              # Bandwidth mérés

# Interface információk
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show geometry_msgs/msg/PoseArray
```

---

## 📞 Kapcsolat

**Készítő:** Mitrenga Márk  
**Projekt:** MGM - LIDAR objektum detektálás  
**Határidő:** 2025. november 3.

---

## 🎉 Tesztelési Eredmények

### T2 Teszt v2 - Mozgó Robot ✅
- ✅ **237 objektum detektálva** (99.6% sikeres!)
- ✅ **Rosbag teljes:** 276.7 sec, 50,338 üzenet, 15.2 MiB
- ✅ **Scan rate:** 0.86 Hz
- ✅ **Topic-ok:** /scan, /filtered_scan, /objects, /object_markers, /odom, /tf, /cmd_vel

### T3 Teszt v2 - Stresszteszt (Több Objektum) ✅
- ✅ **89 objektum detektálva** (98.9% sikeres!)
- ✅ **Rosbag teljes:** 81.7 sec, 1,442 üzenet, 1.3 MiB
- ✅ **Scan rate:** 1.11 Hz (+29% javulás vs T2!) 🚀
- ✅ **Objektumok:** ~3-5 egyidejűleg (manuális spawning)
- ✅ **Környezet:** Statikus robot (nincs navigációs overhead)
- ✅ **RViz optimalizálva:** Csak 3 TF frame, tisztább vizualizáció
- ✅ **Topic nevek javítva:** `/filtered_scan`, `/objects`, `/object_markers`
- ⚠️ **CPU 100%:** WSL limitáció (elfogadható teszteléshez)

**Részletes elemzés:** `tests/test_results/T2_moving/ANALYSIS_T2_v2.md`

---

**Utolsó frissítés:** 2025-10-29 23:15  
**Verzió:** 3.0 (Optimalizált)
