# T3 Teszt - Stresszteszt (Több Objektum)

**Dátum:** 2025-10-29  
**Teszt típus:** Több objektumos stresszteszt  
**Cél:** Rendszer stabilitásának és teljesítményének tesztelése nagy számú objektum esetén

---

## 📋 Teszt Leírás

A T3 teszt célja a LIDAR objektum detektálási rendszer **stressztesztelése** több objektummal egyszerre. Ellenőrizzük:
- Detektálási pontosság több objektum esetén
- CPU és memória használat
- FPS/latency változása terhelés alatt
- Rendszer stabilitása

---

## 🎯 Teszt Konfiguráció

### Alapbeállítások:
- **Robot:** TurtleBot3 Waffle (statikus vagy lassú mozgás)
- **Világ:** turtlebot3_world.world + **extra objektumok**
- **Objektumok száma:** 5-10+ (dobozok, hengerek)
- **Teszt időtartam:** 60-120 sec (rövidebb mint T2)
- **Gazebo mode:** Headless opcionális (gui:=false)

### Node paraméterek:
```yaml
min_range: 0.1
max_range: 10.0
min_cluster_size: 3
cluster_threshold: 0.2
```

---

## 🚀 Futtatási Lépések

### **Opció 1: Gazebo GUI-val (Visual debug)**

#### Terminál 1 - Rendszer indítása
```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

**Várj 10-15 másodpercet az inicializálásra!**

#### Terminál 2 - Extra objektumok spawning
```bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Spawning parancsok (példák):
# Box spawning (1x1x1 méter, különböző pozíciókban)
ros2 run gazebo_ros spawn_entity.py -entity box1 -database box -x 2.0 -y 1.0 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity box2 -database box -x -2.0 -y 1.0 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity box3 -database box -x 1.0 -y -2.0 -z 0.5

# Cylinder spawning
ros2 run gazebo_ros spawn_entity.py -entity cylinder1 -database cylinder -x 3.0 -y 0.0 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity cylinder2 -database cylinder -x -3.0 -y 0.0 -z 0.5
```

**Vagy használd a Gazebo GUI-t:**
- Insert tab → Modellek húzása a világba
- Helyezd el őket a robot körül (1-4 méter távolság)

#### Terminál 3 - Rosbag rögzítés
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Rögzítés indítása
ros2 bag record -o test_run_stress \
  /scan \
  /filtered_scan \
  /objects \
  /object_markers \
  /odom \
  /tf \
  /cmd_vel
```

**Futtatási idő:** Hagyd futni 60-120 másodpercig

#### Terminál 4 (Opcionális) - Robot mozgatása
```bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mozgás (opcionális):**
- Lassan mozogj körbe (w, a, d)
- Vagy hagyd statikusan (több objektumot látni)

---

### **Opció 2: Headless Mode (CPU-optimalizált)**

Ha a CPU túl magas (100%), használd ezt:

```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py gui:=false
```

**Előnyök:**
- ✅ Alacsonyabb CPU használat (nincs Gazebo GUI)
- ✅ Gyorsabb futás
- ❌ Nincs vizuális feedback (csak RViz)

---

## 📊 Mérési Metrikák

### Automatikus gyűjtés (rosbag-ből):
- `/scan` frekvencia
- `/objects` üzenetek száma
- Detektált objektumok száma/üzenet
- Timestamp-ek (latency számításhoz)

### Manuális mérés:
- **CPU használat:**
  ```bash
  htop
  # vagy
  top -p $(pgrep -f lidar_filter_node)
  ```

- **Memory használat:**
  ```bash
  ps aux | grep lidar_filter_node
  ```

- **Topic frekvencia:**
  ```bash
  ros2 topic hz /objects
  ros2 topic hz /scan
  ```

- **Topic bandwidth:**
  ```bash
  ros2 topic bw /objects
  ```

---

## ✅ Ellenőrzési Lista

### Indítás előtt:
- [ ] Workspace sourced (`source install/setup.bash`)
- [ ] TURTLEBOT3_MODEL=waffle exportálva
- [ ] Gazebo világ megnyílt
- [ ] RViz látható és konfigurált

### Tesztelés közben:
- [ ] 5+ objektum spawning megtörtént
- [ ] RViz-ben látszanak az objektumok (scan)
- [ ] Markerek megjelennek az objektumoknál
- [ ] Rosbag rögzítés folyamatban
- [ ] Nincs ERROR üzenet a konzolon
- [ ] CPU < 100% (ha headless mode-ban)

### Tesztelés után:
- [ ] Rosbag fájl létrehozva (`test_run_stress/`)
- [ ] CPU/Memory adatok rögzítve
- [ ] Jegyzetek kitöltve (`notes_t3.md`)
- [ ] Bag info ellenőrizve (`ros2 bag info test_run_stress`)

---

## 🧪 Várható Eredmények

### Sikeres teszt kritériumai:
- ✅ **5+ objektum** egyidejű detektálása
- ✅ **80%+ detektálási arány** (több objektum = nehezebb)
- ✅ **Stabil FPS** (~0.5-1 Hz elfogadható)
- ✅ **Nincs crash** vagy node restart
- ✅ **Rosbag teljes** (minden topic rögzítve)

### Összehasonlítás T2-vel:
| Metrika | T2 (1-3 objektum) | T3 (5-10 objektum) | Változás |
|---------|-------------------|--------------------|----------|
| Detektálási siker | 99.6% | ___% | -___% |
| CPU használat | ~100% | ___% | ___% |
| Scan rate | 0.86 Hz | ___Hz | ___Hz |
| Detektált obj/scan | 1-3 | 5-10+ | +___% |

---

## 🐛 Troubleshooting

### Probléma: Objektumok nem spawnolnak
**Megoldás:**
```bash
# Ellenőrizd a Gazebo service-t
ros2 service list | grep spawn

# Vagy használd a Gazebo GUI Insert tab-ot
```

### Probléma: CPU 100% továbbra is
**Megoldás:** Headless mode:
```bash
ros2 launch lidar_filter optimized_system.launch.py gui:=false
```

### Probléma: Túl sok objektum, lassú a rendszer
**Megoldás:** Csökkentsd az objektumok számát 5-7-re

### Probléma: Objektumok nem detektálódnak
**Ellenőrizd:**
- Objektumok a LIDAR hatótávolságában vannak? (0.1-10m)
- RViz-ben látszanak a scan pontok?
- `/filtered_scan` topic publikál adatot?

---

## 📝 Jegyzetek Kitöltése

Teszt után töltsd ki: `tests/test_results/T3_stress/notes_t3.md`

Figyelj ezekre:
- Spawning módat objektumok száma és pozíciói
- Detektálási siker változása több objektumnál
- CPU/Memory használat változása
- Rendszer stabilitása (crash, restart, hibák)
- Összehasonlítás T2-vel

---

## 🎯 Következő Lépés

Teszt végrehajtása után:
1. ✅ Rosbag info ellenőrzés
2. ✅ Jegyzetek kitöltése
3. ✅ Összehasonlító elemzés T2 vs T3
4. ⏳ Screenshot-ok (később, dokumentációnál)
5. ⏳ Metrikák elemzése és grafikonok

---

**Készítette:** Mitrenga Márk  
**Verzió:** 1.0  
**Kapcsolódó tesztek:** T1 (statikus), T2 (mozgó)
