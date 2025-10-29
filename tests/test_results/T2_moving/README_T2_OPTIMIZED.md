# T2 Teszt Javított Verzió - Optimalizált Beállítások

**Dátum:** 2025-10-29  
**Verzió:** 2.0 (Optimalizált)

## 🔧 Javított Problémák

### 1. CPU 100% probléma
- **Megoldás:** Optimalizált RViz config, kevesebb TF frame megjelenítés
- **Eredmény:** Csökkentett CPU terhelés várható

### 2. RViz "ocsmány" megjelenés - túl sok koordináta rendszer
- **Megoldás:** Új `lidar_filter_optimized.rviz` config
- **Változások:**
  - Csak 3 TF frame látható: `odom`, `base_link`, `base_scan`
  - Eltávolítva: Map display (warning okozó)
  - Marker méret optimalizálva

### 3. Hiányzó rosbag topic-ok
- **Probléma:** Rossz topic nevek a record-ban
- **Helyes topic nevek:**
  - ✅ `/filtered_scan` (nem `/scan_filtered`)
  - ✅ `/objects` (nem `/detected_objects`)
  - ✅ `/object_markers` (nem `/markers`)

---

## 📋 Javított Teszt Lépések

### Terminál 1 - Optimalizált rendszer indítása

```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

**Várj 10-15 másodpercet** hogy minden node inicializálódjon!

---

### Terminál 2 - Helyes rosbag rögzítés

```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T2_moving/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# JAVÍTOTT topic nevek:
ros2 bag record -o test_run_moving_v2 \
  /scan \
  /filtered_scan \
  /objects \
  /object_markers \
  /odom \
  /tf \
  /cmd_vel
```

---

### Terminál 3 - Robot mozgatás

```bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mozgási terv (2 perc):**
1. 15 sec: Egyenes előre (w)
2. 5 sec: Fordulás jobbra (d)
3. 15 sec: Egyenes előre (w)
4. 5 sec: Fordulás balra (a)
5. Ismételd 2x

---

## ✅ Ellenőrzési Lista

### RViz-ben figyelendő:
- [ ] **LaserScan (Raw)** - PIROS pontok láthatók
- [ ] **LaserScan (Filtered)** - ZÖLD pontok láthatók
- [ ] **MarkerArray** - Objektumok körül CYLINDEREK
- [ ] **PoseArray** - SÁRGA nyilak az objektumoknál
- [ ] **TF Frames** - CSAK 3 koordináta rendszer: odom, base_link, base_scan

### Terminálban figyelendő:
- [ ] lidar_filter_node: "Detected X objects" üzenetek
- [ ] Nincs ERROR üzenet
- [ ] Rosbag: "Recording..." üzenet látható

---

## 📊 Várható Eredmények

### Rosbag tartalom (ellenőrzés teszt után):
```bash
ros2 bag info test_run_moving_v2
```

**Várható topic-ok üzenetszámmal:**
- `/scan` - ~200-300 üzenet
- `/filtered_scan` - ~200-300 üzenet
- `/objects` - ~200-300 üzenet
- `/object_markers` - ~200-300 üzenet
- `/odom` - ~800-1200 üzenet
- `/tf` - ~30000-40000 üzenet
- `/cmd_vel` - ~1500-2000 üzenet

---

## 🐛 Troubleshooting

### Ha továbbra is lassú:
1. **Csökkentsd a Gazebo GUI-t:**
   ```bash
   ros2 launch lidar_filter optimized_system.launch.py gui:=false
   ```
   (Ekkor csak a szimuláció fut, nincs Gazebo ablak)

2. **Zárj be felesleges alkalmazásokat** (böngésző, stb.)

3. **Ellenőrizd a CPU-t:**
   ```bash
   htop
   ```

### Ha az RViz nem jelenik meg rendesen:
1. Töröld az RViz cache-t:
   ```bash
   rm -rf ~/.rviz2
   ```
2. Indítsd újra az RViz-t

---

## 📝 Jegyzetek Frissítése

A teszt után töltsd ki: `tests/test_results/T2_moving/notes_t2_v2.md`

Figyelj a következőkre:
- CPU használat (htop-ból)
- Detektált objektumok száma (terminál kimenet)
- RViz framerate
- Gazebo RTF (Real Time Factor)
