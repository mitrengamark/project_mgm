# LIDAR Filter - Objektum Detektálás és Követés

**Csomag neve:** `lidar_filter`  
**Típus:** ROS 2 Python csomag  
**Verzió:** 0.0.1  
**Készítő:** Mitrenga Márk

---

## 📋 Leírás

LIDAR alapú objektum detektálási és követési rendszer ROS 2-ben. A csomag szűri a LIDAR adatokat, detektálja az objektumokat klaszterezési algoritmussal, és vizualizációs markereket publikál.

---

## 🚀 Használat

### Egyszerű indítás (csak a node):
```bash
source install/setup.bash
ros2 run lidar_filter lidar_filter_node
```

### Teljes rendszer indítása (Gazebo + Node + RViz):
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter complete_system.launch.py
```

---

## 📡 Topicok

### Feliratkozások (Subscriptions):
- `/scan` (`sensor_msgs/LaserScan`) - Nyers LIDAR adatok

### Publikálások (Publications):
- `/filtered_scan` (`sensor_msgs/LaserScan`) - Szűrt LIDAR adatok
- `/objects` (`geometry_msgs/PoseArray`) - Detektált objektumok pozíciói
- `/object_markers` (`visualization_msgs/MarkerArray`) - Vizualizációs markerek
- `/map` (`nav_msgs/OccupancyGrid`) - Térképadatok

---

## ⚙️ Paraméterek

| Paraméter | Típus | Alapértelmezett | Leírás |
|-----------|-------|----------------|--------|
| `min_range` | float | 0.1 | Minimális LIDAR távolság (m) |
| `max_range` | float | 10.0 | Maximális LIDAR távolság (m) |
| `min_cluster_size` | int | 3 | Minimális pontok száma egy klaszterben |
| `cluster_threshold` | float | 0.2 | Klaszterezési távolság küszöb (m) |

---

## 🗂️ Fájlstruktúra

```
lidar_filter/
├── lidar_filter/
│   ├── __init__.py
│   └── lidar_filter_node.py      # Fő node implementáció
├── launch/
│   ├── lidar_filter.launch.py    # Csak a node
│   └── complete_system.launch.py # Teljes rendszer
├── config/
│   └── lidar_filter_rviz.rviz    # RViz konfiguráció
├── resource/
│   └── lidar_filter
├── package.xml
├── setup.py
└── README.md                      # Ez a fájl
```

---

## 🧪 Tesztelés

### Topicok ellenőrzése:
```bash
ros2 topic list
ros2 topic echo /objects
ros2 topic hz /filtered_scan
```

### Node információk:
```bash
ros2 node info /lidar_filter_node
```

### Paraméterek módosítása futás közben:
```bash
ros2 param set /lidar_filter_node min_range 0.2
ros2 param set /lidar_filter_node cluster_threshold 0.3
```

---

## 📊 Algoritmus

1. **LIDAR szűrés**: Távolság alapú szűrés (min_range < r < max_range)
2. **Objektum detektálás**: 
   - Poláris → Karteziális koordináta transzformáció
   - Távolság alapú klaszterezés
   - Klaszter centroid számítás
3. **Publikálás**: PoseArray és MarkerArray generálás

---

## 🔧 Függőségek

- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - LaserScan üzenetek
- `geometry_msgs` - Pose, PoseArray üzenetek
- `visualization_msgs` - Marker, MarkerArray üzenetek
- `nav_msgs` - OccupancyGrid üzenetek
- `numpy` - Numerikus számítások

---

## 📝 TODO

- [ ] Objektum követés (tracking) implementálása
- [ ] Térkép építés (SLAM-szerű)
- [ ] Kalman-filter integráció
- [ ] Objektum osztályozás (méret alapján)
- [ ] Performance optimalizálás

---

## 📚 Kapcsolódó dokumentáció

- [Főprojekt README](../../../../README.md)
- [TODO - Feladatlista](../../../../docs/TODO_MitrengaMark.md)
- [Futtatási útmutató](../../../../docs/FUTTATAS_UTMUTATO.md)

---

**Utolsó frissítés:** 2025-10-28  
**Licensz:** Apache-2.0
