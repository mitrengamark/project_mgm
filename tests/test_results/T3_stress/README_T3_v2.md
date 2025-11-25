# 🚀 T3 v2 Teszt - Folyamatos Spawning Útmutató

**Státusz:** ✅ Javított verzió - folyamatos objektum spawning  
**Előző probléma:** T3 v1 - spawning sikertelen (gazebo_ros not found + script azonnal kilépett)

---

## 📋 Mi Változott?

### T3 v1 (Sikertelen)
- ❌ Spawning script NEM source-olta a ROS környezetet
- ❌ Script azonnal kilépett 7 objektum spawn után
- ❌ Mire a rosbag indult, a script már nem futott
- ❌ 0 objektum detektálva

### T3 v2 (Javított)
- ✅ **continuous_spawn.sh** - Végtelen ciklus Ctrl+C-ig
- ✅ Source-olja `/opt/ros/jazzy/setup.bash` és workspace-t
- ✅ Ellenőrzi hogy Gazebo fut-e
- ✅ Random objektumok 1-3 mp-enként
- ✅ Objektumok 5-10 mp után automatikusan törlődnek
- ✅ Folyamatos detektálás tesztelése

---

## 🎯 Teszt Lépések

### 1️⃣ Terminal 1 - Rendszer Indítása
```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

**Várakozás:** 10-15 másodperc (Gazebo + RViz init)

---

### 2️⃣ Terminal 2 - Folyamatos Spawning
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress
source /home/mark/codes/mgm/project_mgm/install/setup.bash
./continuous_spawn.sh
```

**Output példa:**
```
============================================
  T3 Folyamatos Objektum Spawning
============================================

Objektumok folyamatosan spawn-olnak és tűnnek el.
Nyomj Ctrl+C a leállításhoz!

✅ Gazebo fut, spawning indítása...

[23:56:15] Spawning: stress_Box_1 at (2.0, 1.5, 0.5)
   ↳ Élettartam: 7s
   ↳ Törlés: stress_Box_1

[23:56:18] Spawning: stress_Cyl_2 at (-3.0, 0.0, 0.5)
   ↳ Élettartam: 5s
...
```

**Ellenőrzés:**
- Gazebo-ban látszanak az objektumok? ✅
- RViz-ben markerek megjelennek? ✅
- Objektumok eltűnnek pár mp után? ✅

---

### 3️⃣ Terminal 3 - Rosbag Rögzítés
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash

ros2 bag record -o test_run_stress_v2 \
  /scan \
  /filtered_scan \
  /objects \
  /object_markers \
  /odom \
  /tf \
  /cmd_vel
```

**Futtatási idő:** 90-120 másodperc

---

### 4️⃣ (Opcionális) Terminal 4 - Monitoring
```bash
# Detektált objektumok számának monitorozása
watch -n 2 'ros2 topic echo /objects --once | grep -c "position:"'

# Vagy CPU monitoring
htop
```

---

## 🛑 Leállítás (Sorrendben!)

1. **Terminal 3:** `Ctrl+C` → Rosbag rögzítés leáll
2. **Terminal 4:** `Ctrl+C` → Monitoring leáll (ha fut)
3. **Terminal 2:** `Ctrl+C` → Spawning leáll
4. **Terminal 1:** `Ctrl+C` → Rendszer leáll

---

## ✅ Teszt Utáni Ellenőrzés

### Rosbag Info
```bash
cd tests/test_results/T3_stress/rosbag
ros2 bag info test_run_stress_v2
```

**Várható eredmények:**
```
Duration: 90-120 sec
Messages: ~15,000-20,000

/scan:           80-100 msg   (~0.8 Hz)
/filtered_scan:  80-100 msg   (~0.8 Hz)
/objects:        80-100 msg   (~0.8 Hz)  ← VÁRHATÓ: PoseArray-k objektumokkal!
/object_markers: 80-100 msg
/odom:           ~400-600 msg
/tf:             ~12,000-18,000 msg
```

**Sikerkritérium:** `/objects` topic nem üres (> 0 objektum detektálva)

---

### Objektumok Ellenőrzése
```bash
# Egy /objects üzenet megtekintése
ros2 bag play test_run_stress_v2 &
ros2 topic echo /objects --once
```

**Várható output:**
```yaml
header:
  stamp:
    sec: 1761778...
    nanosec: ...
  frame_id: odom
poses:
- position:
    x: 2.05
    y: 1.48
    z: 0.0
  orientation: ...
- position:
    x: -2.98
    y: 0.02
    z: 0.0
  orientation: ...
# 2-5 objektum pozíció...
```

---

## 📊 Várható Metrikák (T3 v2)

| Metrika | T2 Baseline | T3 v2 Várható | Megjegyzés |
|---------|-------------|---------------|------------|
| **Objektumok** | 1-3 | 2-5 egyidejűleg | Folyamatosan változik |
| **Teszt időtartam** | 276.7 sec | 90-120 sec | Rövidebb, de intenzívebb |
| **Bag méret** | 15.2 MiB | 5-8 MiB | Rövidebb futás |
| **Scan rate** | 0.86 Hz | ~0.7-0.9 Hz | Hasonló |
| **Det. obj/scan** | ~1-3 | 2-5 | Változó |
| **CPU használat** | ~100% | ~100% | WSL limit |

---

## 🎯 Sikerkritériumok

### Funkcionális
- ✅ Objektumok spawning sikeres (láthatók Gazebo-ban)
- ✅ Objektumok detektálása működik (markerek RViz-ben)
- ✅ Objektumok törlése automatikus (5-10 mp után)
- ✅ Folyamatos működés 90-120 másodpercig
- ✅ Rosbag tartalmaz objektum detektálásokat

### Teljesítmény
- ✅ Nincs crash vagy node restart
- ✅ Scan rate stabil (~0.7-0.9 Hz)
- ✅ CPU/Memory nem robban fel (WSL limiteken belül)
- ✅ RViz responsive marad

---

## 🔧 Troubleshooting

### "Package 'gazebo_ros' not found"
```bash
# Ellenőrizd hogy source-oltad a workspace-t
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Vagy futtasd a scriptet újra
./continuous_spawn.sh
```

### "Gazebo nem fut!" hiba
```bash
# Először indítsd a rendszert (Terminal 1)
ros2 launch lidar_filter optimized_system.launch.py

# Várj 10-15 mp-et, AZTÁN indítsd a spawn scriptet
```

### Objektumok nem látszanak RViz-ben
```bash
# Ellenőrizd hogy a markerek display engedélyezve van
# RViz → Displays → MarkerArray → Enabled: ✓

# Ellenőrizd a topic-ot
ros2 topic echo /object_markers --once
```

### CPU 100% és lag
```bash
# Használd a headless mode-ot (nincs Gazebo GUI)
ros2 launch lidar_filter optimized_system.launch.py gui:=false
```

---

## 📝 Jegyzet Kitöltése

Teszt végrehajtása után töltsd ki: `notes_t3.md`

**Fontos mezők:**
- Spawning objektumok táblázat (hány db spawn-olt)
- Rosbag ellenőrzés (méret, duration, topic üzenetek)
- T2 vs T3 összehasonlítás
- Problémák / Hibák
- Következtetések

---

**Készítette:** Mitrenga Márk  
**Verzió:** T3 v2 (Javított)  
**Kapcsolódó:** continuous_spawn.sh, ANALYSIS_T3_v1.md, notes_t3.md
