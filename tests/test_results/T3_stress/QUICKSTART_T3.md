# 🚀 T3 Teszt - Gyors Indítási Útmutató

**Cél:** Stresszteszt 2-5 objektummal (folyamatos spawning/delete)  
**⚠️ FONTOS:** Ez a v2 verzió! v1 spawning sikertelen volt - lásd ANALYSIS_T3_v1.md

---

## ⚡ Gyors Indítás - 4 Terminál

### Terminal 1️⃣ - Rendszer indítása
```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

**Várj 10-15 másodpercet!** ⏳

---

### Terminal 2️⃣ - Objektumok spawning (FOLYAMATOS)
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress
source /home/mark/codes/mgm/project_mgm/install/setup.bash
./continuous_spawn.sh
```

**🔄 FOLYAMATOS SPAWNING:**
- Random objektumok spawn-olnak 1-3 másodpercenként
- Minden objektum 5-10 másodpercig látható, aztán eltűnik
- Ctrl+C leállítja a script-et
- Így mindig van 2-5 objektum egyszerre a világban

**Vagy egyedileg (manuális):**
```bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Box-ok
ros2 run gazebo_ros spawn_entity.py -entity box1 -database unit_box -x 2.0 -y 1.5 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity box2 -database unit_box -x -2.0 -y 1.5 -z 0.5

# Cylinder-ek
ros2 run gazebo_ros spawn_entity.py -entity cyl1 -database unit_cylinder -x 2.0 -y -1.5 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity cyl2 -database unit_cylinder -x -2.0 -y -1.5 -z 0.5

# Törlés
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box1'}"
```

**Ellenőrzés Gazebo-ban:** Látszanak az objektumok? ✅

---

### Terminal 3️⃣ - Rosbag rögzítés
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Ha már létezik a mappa, töröld:
# rm -rf test_run_stress_v2

ros2 bag record -o test_run_stress_v2 --topics \
  /scan \
  /filtered_scan \
  /objects \
  /object_markers \
  /odom \
  /tf \
  /cmd_vel
```

**Futtatási idő:** 90-120 sec (T3 v2 folyamatos spawning teszthez)

---

### Terminal 4️⃣ (Opcionális) - Monitoring
```bash
# Topic frekvencia
ros2 topic hz /objects

# Detektált objektumok élőben
ros2 topic echo /objects --once

# CPU monitoring
htop
```

---

## 🛑 Leállítás

1. **Terminal 3:** Ctrl+C (rosbag stop)
2. **Terminal 4:** Ctrl+C (monitoring)
3. **Terminal 2:** Ctrl+C (folyamatos spawning leállítása)
4. **Terminal 1:** Ctrl+C (rendszer leállítás)

---

## ✅ Ellenőrzés Teszt Után

```bash
# Rosbag info
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
ros2 bag info test_run_stress

# Várható:
# - Duration: 60-120 sec
# - /objects: ~60-120 üzenet
# - /filtered_scan: ~60-120 üzenet
# - Detektált obj/scan: 5-10+
```

**Jegyzet kitöltése:**
```bash
nano notes_t3.md
# vagy VS Code-ban
code notes_t3.md
```

---

## 🎯 Sikerkritériumok

- ✅ 5+ objektum spawning sikeres
- ✅ Minden objektum látható az RViz-ben
- ✅ Detektálás működik (markerek megjelennek)
- ✅ Rosbag teljes (minden topic rögzítve)
- ✅ Nincs crash vagy ERROR
- ✅ CPU/Memory elfogadható szinten

---

## 💡 Tippek

1. **Ha lassú a rendszer:** Használd a headless mode-ot:
   ```bash
   ros2 launch lidar_filter optimized_system.launch.py gui:=false
   ```

2. **Ha objektumok nem látszanak:** Ellenőrizd a pozíciókat:
   - 0.1-10m tartományban (LIDAR range)
   - Nem túl közel (< 0.1m)
   - Nem túl messze (> 10m)

3. **Ha spawning nem működik:** Használd a Gazebo GUI Insert tab-ot manuálisan

4. **Optimális objektumszám:** 5-7 objektum (WSL környezetben)

---

**Készítette:** Mitrenga Márk  
**Kapcsolódó:** README_T3.md, notes_t3.md, spawn_objects.sh
