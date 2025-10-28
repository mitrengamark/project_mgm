# MGM Projekt - LIDAR alapú objektum detektálás és követés

**Készítő:** Mitrenga Márk  
**Téma:** Objektum térkép összeállítása, sík LIDAR alapú objektum detektálás és követés  
**ROS 2 verzió:** Jazzy  
**Határidő:** 2025. november 3.

---

## 📖 Dokumentáció

- **[TODO - Feladatlista](docs/TODO_MitrengaMark.md)** - Részletes feladatok, státusz, és mérföldkövek (95% kész Fázis 1)
- **[Futtatási útmutató](docs/FUTTATAS_UTMUTATO.md)** - Lépésről-lépésre rendszerindítási útmutató (v2.0)
- **[Munkamenet összefoglaló](docs/MUNKA_OSSZEFOGLALO_2025-10-28.md)** - 2025.10.28 munkamenet részletei
- **[LIDAR Filter README](src/mgm_gyak/lidar_filter/README.md)** - Csomag-specifikus dokumentáció

---

## 🚀 Gyors indítás

**Legegyszerűbb módszer (1 parancs - AJÁNLOTT):**

```bash
cd ~/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter complete_system.launch.py
```

Ez elindítja a Gazebo-t, LIDAR filter node-ot, és RViz2-t előre beállított konfigurációval!

---

**Részletes indítás (debug célra):**

```bash
# 1. Build
cd ~/codes/mgm/project_mgm
colcon build --symlink-install
source install/setup.bash

# 2. Gazebo szimuláció (Terminal 1)
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 3. LIDAR Filter Node (Terminal 2)
~/codes/mgm/project_mgm/install/lidar_filter/bin/lidar_filter_node

# 4. RViz2 vizualizáció (Terminal 3)
ros2 run rviz2 rviz2 -d install/lidar_filter/share/lidar_filter/config/lidar_filter_rviz.rviz
```

**Részletes útmutató:** [docs/FUTTATAS_UTMUTATO.md](docs/FUTTATAS_UTMUTATO.md)

---

## 📦 Csomag: `lidar_filter`

### Funkciók:
- ✅ LIDAR adatok szűrése (távolság alapján)
- ✅ Objektum detektálás klaszterezéssel
- ✅ Objektum pozíciók publikálása
- ✅ Vizualizációs markerek generálása
- 🔜 Objektum követés (tracking)
- 🔜 Térkép építés (mapping)

### Publikált topicok:
- `/filtered_scan` - Szűrt LIDAR adatok
- `/objects` - Detektált objektumok (PoseArray)
- `/object_markers` - Vizualizációs markerek (MarkerArray)
- `/map` - Térképadatok (OccupancyGrid)

### Feliratkozott topicok:
- `/scan` - Nyers LIDAR adatok (LaserScan)

---

## 🏗️ Projektstátusz

**Aktuális állapot:** 🟢 Fejlesztés alatt (Fázis 1: 95% kész)

### ✅ Kész:
- ROS 2 környezet beállítása (Jazzy)
- `lidar_filter` csomag implementálása
- Objektum detektálás alapfunkciók
- Gazebo szimuláció TurtleBot3-mal
- RViz konfiguráció
- Complete system launch fájl
- Dokumentáció (README, TODO, Futtatási útmutató)

### 🔜 Következő:
- RViz konfiguráció finomhangolása
- Rosbag tesztadatok rögzítése
- Tesztelési terv kidolgozása (Overleaf)
- Metrikák gyűjtése és kiértékelése

---

## 📁 Workspace struktúra

```
project_mgm/
├── src/
│   ├── DynamixelSDK/
│   ├── mgm_gyak/
│   │   ├── lidar_filter/          ← Fő projekt
│   │   ├── gyak2-6/
│   │   └── hamster_simulation/
│   ├── turtlebot3/
│   ├── turtlebot3_msgs/
│   ├── turtlebot3_autorace/
│   └── turtlebot3_simulations/
├── build/                         ← Build fájlok
├── install/                       ← Telepített csomagok
├── log/                           ← Build log-ok
├── docs/
│   ├── TODO_MitrengaMark.md
│   └── FUTTATAS_UTMUTATO.md
└── README.md                      ← Ez a fájl
```

---

## 🛠️ Technikai követelmények

- **OS:** Ubuntu 24.04 / WSL2
- **ROS 2:** Jazzy
- **Python:** 3.12+ (rendszer Python, NEM conda!)
- **Függőségek:**
  - `sensor_msgs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `nav_msgs`
  - `numpy`

---

## 📚 További információk

- [ROS 2 Jazzy dokumentáció](https://docs.ros.org/en/jazzy/)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo dokumentáció](https://gazebosim.org/)

---

**Utolsó frissítés:** 2025-10-28 23:45  
**Git branch:** `mark`
