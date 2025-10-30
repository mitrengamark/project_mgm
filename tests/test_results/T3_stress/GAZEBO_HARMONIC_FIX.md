# 🔧 T3 Gazebo Harmonic Kompatibilitás - Gyorsjavítás

**Dátum:** 2025-10-30  
**Probléma:** spawn_objects.sh és continuous_spawn.sh **Gazebo Classic** parancsokat használt, de a rendszer **Gazebo Harmonic**-ot futtat!

---

## 🚨 Azonosított Probléma

### Eredeti Hiba
```bash
❌ HIBA: Gazebo nem fut!
Indítsd el először: ros2 launch lidar_filter optimized_system.launch.py
```

### Kiváltó Ok

**1. Gazebo Version Mismatch**
- Script elvárta: **Gazebo Classic** (`ros2 run gazebo_ros spawn_entity.py`)
- Rendszer fut: **Gazebo Harmonic** (`gz sim`)

**2. Service Ellenőrzés Hiba**
```bash
# Régi kód (nem működött):
if ! ros2 service list | grep -q "/gazebo"; then

# Probléma: Gazebo Harmonic NEM használ /gazebo ROS2 service-eket!
```

**3. Spawning Parancsok Inkompatibilisek**
```bash
# Gazebo Classic (NEM MŰKÖDIK):
ros2 run gazebo_ros spawn_entity.py -entity box1 -database unit_box -x 2.0 -y 1.5 -z 0.5

# Gazebo Harmonic (MŰKÖDIK):
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --req 'sdf: "<model name=\"box1\">...</model>"'
```

---

## ✅ Megoldás

### 1. Gazebo Futás Ellenőrzés (Javítva)
```bash
# continuous_spawn.sh
if ! pgrep -f "gz sim" > /dev/null; then
    echo "❌ HIBA: Gazebo nem fut!"
    exit 1
fi
```

### 2. Spawning Parancs (Gazebo Harmonic SDF)
```bash
# Box spawning
gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: '<model name=\"stress_Box_1\">
      <static>false</static>
      <pose>2 1.5 0.5 0 0 0</pose>
      <link name=\"link\">
        <inertial><mass>1.0</mass></inertial>
        <collision name=\"collision\">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        </collision>
        <visual name=\"visual\">
          <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>'"
```

### 3. Törlés Parancs (Gazebo Harmonic)
```bash
gz service -s /world/default/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "name: 'stress_Box_1', type: MODEL"
```

---

## 🧪 Teszt Eredmények

```bash
# Spawning teszt
$ gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'sdf: "<model name=\"test_box_1\">...</model>"'
data: true  ✅

# Törlés teszt
$ gz service -s /world/default/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 2000 --req "name: 'test_box_1', type: MODEL"
data: true  ✅
```

---

## 📝 Frissített Fájlok

### ✅ continuous_spawn.sh
- **Ellenőrzés:** `pgrep -f "gz sim"` (process ellenőrzés)
- **Spawning:** Gazebo Harmonic SDF format
- **Törlés:** `gz service -s /world/default/remove`
- **Geometria:** Box (0.5x0.5x0.5), Cylinder (r=0.25, h=0.5)
- **Szín:** Zöld (ambient/diffuse 0 1 0 1)

### ⚠️ spawn_objects.sh
- **Státusz:** Még régi Gazebo Classic parancsokat használ
- **TODO:** Átírni Gazebo Harmonic-ra (ha batch mode szükséges)
- **Javaslat:** Használd a `continuous_spawn.sh`-t helyette!

---

## 🚀 Használat (Frissített)

### Terminal 1: Rendszer Indítás
```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter optimized_system.launch.py
```

### Terminal 2: Folyamatos Spawning (Gazebo Harmonic)
```bash
cd tests/test_results/T3_stress
source /home/mark/codes/mgm/project_mgm/install/setup.bash
./continuous_spawn.sh
```

**Output:**
```
============================================
  T3 Folyamatos Objektum Spawning
============================================

✅ Gazebo (Harmonic) fut, spawning indítása...

[18:55:30] Spawning: stress_Box_1 at (2.0, 1.5, 0.5)
   ↳ Élettartam: 7s
   ↳ Törlés: stress_Box_1

[18:55:33] Spawning: stress_Cyl_2 at (-3.0, 0.0, 0.5)
   ↳ Élettartam: 5s
...
```

---

## 🎯 Következő Lépések

1. ✅ **continuous_spawn.sh működik** - Gazebo Harmonic kompatibilis
2. ⏳ **T3 v2 teszt futtatása** - 90-120 sec rosbag rögzítéssel
3. ⏳ **spawn_objects.sh frissítés** (opcionális - ha batch mode kell)
4. ⏳ **README_T3_v2.md frissítés** - Gazebo Harmonic megjegyzésekkel

---

**Státusz:** ✅ JAVÍTVA - continuous_spawn.sh Gazebo Harmonic ready!  
**Tesztelve:** Spawning és törlés működik (`data: true`)  
**Futtasd újra:** `./continuous_spawn.sh` és nézd meg a Gazebo-ban! 🎉
