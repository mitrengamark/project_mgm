# 🚀 Futtatási útmutató - MGM Projekt

**Projekt:** LIDAR alapú objektum detektálás és követés  
**Készítette:** Mitrenga Márk  
**Utolsó frissítés:** 2025. október 28. 23:45

---

## 📋 Előfeltételek

- ✅ ROS 2 Jazzy telepítve
- ✅ Ubuntu / WSL Linux környezet
- ✅ Workspace lefordítva (`colcon build` sikeres)
- ⚠️ **FONTOS:** Ne aktiválj conda/virtualenv környezetet!

---

## 🎯 Gyors indítás - EGYETLEN PARANCS! 🚀

**Legegyszerűbb módszer - Minden komponens egyszerre indul:**

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter complete_system.launch.py
```

**Ez elindítja:**
- ✅ Gazebo szimulációt TurtleBot3-mal
- ✅ LIDAR Filter Node-ot
- ✅ RViz2-t előre beállított konfigurációval

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
~/codes/mgm/project_mgm/install/lidar_filter/bin/lidar_filter_node
```

**Várt eredmény:**
```
[INFO] [lidar_filter_node]: LIDAR Filter Node initialized
```

**Publikált topicok:**
- `/filtered_scan` - Szűrt LIDAR adatok
- `/objects` - Detektált objektumok (PoseArray)
- `/object_markers` - Vizualizációs markerek (MarkerArray)
- `/map` - Térképadatok (OccupancyGrid)

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

### Csak fontos topicok rögzítése:
```bash
ros2 bag record /scan /filtered_scan /objects /object_markers /map -o test_run2
```

### Rosbag visszajátszása:
```bash
ros2 bag play test_run1 --clock
```

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
**Megoldás:** Közvetlenül futtatás:
```bash
~/codes/mgm/project_mgm/install/lidar_filter/bin/lidar_filter_node
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
│   │   ├── lidar_filter_node.py        # Fő node
│   │   └── __init__.py
│   ├── launch/
│   │   └── lidar_filter.launch.py      # Launch fájl (NEM működik!)
│   ├── package.xml
│   └── setup.py
├── install/lidar_filter/               # Build kimenet
│   └── bin/lidar_filter_node           # Executable itt van!
└── docs/
    ├── TODO_MitrengaMark.md            # Feladatlista
    └── FUTTATAS_UTMUTATO.md            # Ez a fájl
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

**Utolsó frissítés:** 2025-10-28 23:45  
**Verzió:** 2.0
