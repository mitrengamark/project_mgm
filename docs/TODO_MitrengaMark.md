# 🎯 Mitrenga Márk – Teendők és mérföldkövek (MGM projekt)

**Téma:** Objektum térkép összeállítása, sík lidar alapú objektum detektálás és követés  
**Feladatkör:** Tesztelési terv és prezentáció  
**Határidő:** 2025. november 3.

---

## 🧱 Projektkörnyezet

- **ROS 2 Jazzy** (telepítve a `ros_setup.sh` segítségével)
- **Python 3.10+**
- **Ubuntu / WSL Linux környezet**
- **RViz2, rosbag, Gazebo** (teszteléshez)
- **VS Code + GitHub Copilot** (kódíráshoz)

> Ellenőrzéshez:
> ```bash
> echo $ROS_DISTRO   # -> jazzy
> ros2 run demo_nodes_cpp talker
> ```

---

## 🧩 Feladataim összefoglalva

| Fázis | Cél | Részfeladatok | Eredmény | Állapot |
|-------|-----|----------------|-----------|---------|
| 1️⃣ Tesztkörnyezet előkészítés | ROS2 workspace és tesztadatok előkészítése | Build, rosbag futtatás, RViz megjelenítés | működő tesztkörnyezet | ✅ 100% |
| 2️⃣ Tesztelési terv kidolgozása | Szcenáriók, metrikák, futtatási terv | 3 különböző teszteset dokumentálása | Overleaf PDF (2–3 oldal) | 🔶 60% |
| 3️⃣ Tesztfuttatás és adatgyűjtés | Bag replay, CPU/latency mérés, screenshotok | Futtatási napló, ábrák, metrikák | teszt report és ábrák | 🔶 80% |
| 4️⃣ Prezentáció | Összefoglaló 6–7 perces bemutató | Ábrák, mérési eredmények, konklúzió | ppt/beamer export Overleaf-ből | ⏳ 10% |

---

## ⚙️ 1. Tesztkörnyezet előkészítése

**Cél:** a ROS2 rendszer futtatása, a workspace és a szimulációs adatforrások ellenőrzése.

### 1.1 Build és környezet inicializálás ✅

```bash
cd ~/codes/mgm/project_mgm
colcon build --symlink-install
source install/setup.bash
```

**Státusz:** ✅ Sikeres - 29 csomag lefordult

### 1.2 Alap teszt – LIDAR node-ok és launch fájl ✅

```bash
ros2 launch lidar_filter lidar_filter.launch.py
ros2 node list
ros2 topic list
ros2 topic echo /scan
```

**Státusz:** ✅ A `lidar_filter` csomag létrehozva és működik
- Node: `lidar_filter_node` ✅
- Topicok: `/scan`, `/filtered_scan`, `/objects`, `/object_markers`, `/map` ✅

### 1.3 Rosbag lejátszás és RViz megjelenítés ✅

```bash
ros2 bag play data/test_scene.bag --clock
ros2 run rviz2 rviz2
```

**Státusz:** ✅ Kész!
- ✅ RViz2 indítható
- ✅ Gazebo szimuláció fut TurtleBot3-mal
- ✅ RViz konfiguráció létrehozva és mentve (`config/lidar_filter_rviz.rviz`)
- ✅ Complete system launch fájl készített
- ⏳ Rosbag fájl még nem készült

### 1.4 Ellenőrizd: ✅

- ✅ `/scan` → LIDAR adatfolyam látható
- ✅ `/filtered_scan`, `/objects`, `/map` topicok megjelennek
- ✅ `rqt_graph` elindítva - Node topológia vizualizálható

### 1.5 Logolás és mentés ✅

```bash
mkdir -p ~/codes/mgm/project_mgm/results
ros2 bag record /scan /filtered_scan /objects /object_markers /odom /tf /cmd_vel -o test_run
```

**Státusz:** ✅ T1 és T2 tesztesetek KÉSZ!
- ✅ **T1 (statikus környezet)** - rosbag rögzítve, metrikák gyűjtve
- ✅ **T2 v1 (mozgó robot)** - Futott, de hiányos (0 filter topic)
- ✅ **T2 v2 (optimalizált)** - SIKERES! 237 objektum, 99.6% siker
  - Időtartam: 276.7 sec
  - Méret: 15.2 MiB
  - Üzenetek: 50,338
  - Detektálás: 237/238 scan = 99.6% ✅
- ⏳ **T3 (stresszteszt)** - Következik

---

## 🧪 2. Tesztelési terv kidolgozása

**Cél:** meghatározni, mit, hogyan és mivel fogsz mérni.

**Státusz:** 🔶 Folyamatban (40%)

### 2.1 Tesztszcenáriók

| ID | Leírás | Elvárt eredmény | Státusz |
|----|--------|-----------------|---------|
| T1 | Statikus akadály (fix tárgy) | Stabil pozíció, alacsony hiba (<5cm) | ✅ KÉSZ |
| T2 | Mozgó objektum (robottal) | Objektum azonosítása és folytonos követés | ✅ KÉSZ (99.6% siker!) |
| T3 | Több objektum egyidejűleg (stresszteszt) | Konzisztens detektálás, 90%+ arány | ✅ KÉSZ (98.9% siker!) |

**T2 Eredmények (mozgó robot):**
- ✅ 237 objektum detektálva 238 scan-ből
- ✅ 99.6% sikeres detektálás
- ✅ Scan rate: 0.86 Hz
- ✅ Rosbag teljes (276.7s, 15.2 MiB)

**T3 Eredmények (statikus robot, több objektum):**
- ✅ 89 objektum detektálva 90 scan-ből
- ✅ 98.9% sikeres detektálás
- ✅ Scan rate: 1.11 Hz (+29% vs T2!) 🚀
- ✅ Rosbag teljes (81.7s, 1.3 MiB)
- ✅ Objektumok: ~3-5 egyidejűleg (manuális spawning)

### 2.2 Mérési metrikák

| Metrika | Jelentés | Mérési módszer |
|---------|----------|----------------|
| Precision / Recall | Detektálási pontosság | valós vs detektált objektumok száma |
| Pozíciós hiba (m) | Objektum becsült és valós pozíciója | Python script vagy bag elemzés |
| CPU-idő (ms/frame) | Feldolgozási sebesség | ros2 topic hz + htop |
| Késleltetés (s) | Üzenetküldés és feldolgozás közti idő | timestamp differencia (rostopic echo) |

### 2.3 Tesztelési folyamat dokumentálása

- Overleaf: "Tesztelési terv és eredmények – Mitrenga Márk"
- Futtatási paraméterek (rate, QoS, node-ok száma)
- Minden méréshez screenshot és rövid jegyzet

---

## 📊 3. Tesztfuttatás és adatgyűjtés

**Cél:** a fejlesztett node-ok (detektálás, követés, térképezés) valós működésének mérése.

**Státusz:** 🔶 Folyamatban (60%)

### 3.1 Futtatás ✅

```bash
# Optimalizált rendszer (AJÁNLOTT)
ros2 launch lidar_filter optimized_system.launch.py

# Alternatív - eredeti verzió
ros2 launch lidar_filter complete_system.launch.py
```

**Futtatva:**
- ✅ T1 teszt (statikus környezet)
- ✅ T2 v1 teszt (hiányos)
- ✅ T2 v2 teszt (sikeres, optimalizált, mozgó robot)
- ✅ T3 v1 teszt (spawning sikertelen)
- ✅ T3 v2 teszt (sikeres, manuális spawning, statikus robot)

### 3.2 Adatrögzítés ✅

```bash
# ⚠️ FONTOS: Helyes topic nevek!
ros2 bag record /scan /filtered_scan /objects /object_markers /odom /tf /cmd_vel -o test_run
```

**Rögzítve:**
- ✅ T1: test_run1_static (~6 sec)
- ✅ T2 v1: test_run_moving (214s, 9.3 MiB) - hiányos
- ✅ T2 v2: test_run_moving_v2 (276.7s, 15.2 MiB) - teljes ✅
- ✅ T3 v1: test_run_stress (61.08s, 2.7 MiB) - spawning sikertelen
- ✅ T3 v2: test_run_stress_v2 (81.7s, 1.3 MiB) - teljes ✅

### 3.3 Metrikák számítása 🔶

**Elkészült:**
- ✅ T1: metrics_t1.csv
- ✅ T2 v2: Részletes elemzés (ANALYSIS_T2_v2.md)
  - Scan rate: 0.86 Hz
  - Detektálási siker: 99.6%
  - Üzenetek/sec: 182 msg/sec
- ✅ T3 v1: Részletes elemzés (ANALYSIS_T3_v1.md)
  - Spawning sikertelen probléma azonosítva
  - Gazebo Harmonic kompatibilitási javítások
- ✅ T3 v2: Részletes elemzés (ANALYSIS_T3_v2.md)
  - Scan rate: 1.11 Hz (+29% vs T2!)
  - Detektálási siker: 98.9%
  - Manuális objektum spawning

**Még kell:**
- ⏳ Python script: rosbag → CSV konverzió
- ⏳ Grafikonok készítése (matplotlib) - T1 vs T2 vs T3
- ⏳ Precision/Recall számítás
- ⏳ T3 rosbag részletes elemzés (objektumszám/scan)

### 3.4 Eredmények 🔶

**Elkészült:**
- ✅ Rosbag fájlok (T1, T2 v1, T2 v2, T3 v1, T3 v2)
- ✅ Teszt jegyzetek (notes_t1.md, notes_t2_v2.md, notes_t3.md)
- ✅ Részletes elemzések:
  - ANALYSIS_T2_v2.md (mozgó robot)
  - ANALYSIS_T3_v1.md (spawning problémák)
  - ANALYSIS_T3_v2.md (stresszteszt sikeres)
  - GAZEBO_HARMONIC_FIX.md (kompatibilitás)
- ✅ Összehasonlító metrikák (T2 vs T3)

**Még kell:**
- ⏳ **Screenshot-ok:** RViz, Gazebo, rqt_graph
- ⏳ **Táblázatok:** Precision/Recall, latency, CPU-idő
- ⏳ **Grafikonok:** T1 vs T2 vs T3, scan rate, detektálási arány

---

## 🗣️ 4. Prezentáció (6–7 perc)

**Cél:** bemutatni a teljes projekt eredményét a három rész (elmélet–architektúra–teszt) mentén.

**Státusz:** ⏳ Még nem kezdődött el

### 4.1 Tartalom

- **Bevezetés:** projekt célja, bemenet–kimenet (1 perc)
- **Architektúra:** node-topológia, topicok, frame-ek (2 perc)
- **Tesztelés:** metrikák, eredmények, ábrák (3 perc)
- **Következtetés:** konzisztencia, javaslatok, lezárás (1 perc)

### 4.2 Anyag

- 6–8 slide, egységes dizájnnal
- Saját rész: "Testing and Validation – Mitrenga Márk"
- Overleaf → Beamer formátum, PDF export
- Ábrák: RViz, rqt_graph, metrikák diagramjai


## ✅ Fájlstruktúra

```
project_mgm/
├── src/
│   └── mgm_gyak/
│       └── lidar_filter/                    ✅ Létrehozva és optimalizálva
│           ├── lidar_filter/
│           │   ├── __init__.py              ✅
│           │   └── lidar_filter_node.py     ✅ LIDAR objektum detektálás
│           ├── launch/
│           │   ├── lidar_filter.launch.py   ✅ Egyszerű launch
│           │   ├── complete_system.launch.py ✅ Teljes rendszer launch (v1)
│           │   └── optimized_system.launch.py ✅ ÚJ! Optimalizált launch (v2)
│           ├── config/
│           │   ├── lidar_filter_rviz.rviz   ✅ RViz konfiguráció (eredeti)
│           │   └── lidar_filter_optimized.rviz ✅ ÚJ! Optimalizált RViz (3 TF)
│           ├── resource/
│           │   └── lidar_filter             ✅
│           ├── package.xml                   ✅ ROS 2 package
│           ├── setup.py                      ✅ Python setup
│           ├── setup.cfg                     ✅ ÚJ! Script telepítési helyek
│           └── README.md                     ✅ Csomag dokumentáció
├── install/lidar_filter/                     ✅ Build output
│   ├── lib/lidar_filter/                    ✅ ÚJ! Node itt van (javítva)
│   │   └── lidar_filter_node                ✅ Executable (ros2 run működik!)
│   └── bin/                                 ✅ Symlink (setup.cfg előtt itt volt)
├── config/
│   └── lidar_filter_rviz.rviz               ✅ RViz konfiguráció (másolat)
├── tests/                                    ✅ Tesztek folyamatban!
│   ├── test_cases.md                        ✅ Tesztesetek dokumentálva (T1, T2, T3)
│   └── test_results/
│       ├── T1_static/                       ✅ T1 KÉSZ!
│       │   ├── rosbag/
│       │   │   └── test_run1_static/        ✅ Rosbag rögzítve (~6 sec)
│       │   ├── screenshots/                 ⏳ Következik
│       │   ├── metrics_t1.csv               ✅ Metrikák CSV-ben
│       │   └── notes_t1.md                  ✅ Teszt jegyzet
│       ├── T2_moving/                       ✅ T2 KÉSZ!
│       │   ├── rosbag/
│       │   │   ├── test_run_moving/         ✅ v1 (hiányos, 214s, 9.3 MiB)
│       │   │   └── test_run_moving_v2/      ✅ v2 (teljes, 276s, 15.2 MiB)
│       │   ├── screenshots/                 ⏳ Következik
│       │   ├── notes_t2.md                  ✅ v1 jegyzet
│       │   ├── notes_t2_v2.md               ✅ ÚJ! v2 jegyzet (részletes)
│       │   ├── README_T2.md                 ✅ Teszt leírás (v1)
│       │   ├── README_T2_OPTIMIZED.md       ✅ ÚJ! Optimalizált útmutató (v2)
│       │   └── ANALYSIS_T2_v2.md            ✅ ÚJ! Részletes eredmény elemzés
│       ├── T3_stress/                       ✅ T3 KÉSZ!
│       │   ├── rosbag/
│       │   │   ├── test_run_stress/         ✅ v1 (spawning sikertelen, 61s)
│       │   │   └── test_run_stress_v2/      ✅ v2 (sikeres, 81.7s, 1.3 MiB)
│       │   ├── continuous_spawn.sh          ✅ Gazebo Harmonic spawning script
│       │   ├── spawn_objects.sh             ✅ Batch spawning (javítva)
│       │   ├── notes_t3.md                  ✅ Teszt jegyzet (v1 + v2)
│       │   ├── QUICKSTART_T3.md             ✅ Gyors indítási útmutató
│       │   ├── README_T3.md                 ✅ Részletes teszt leírás
│       │   ├── README_T3_v2.md              ✅ v2 javított útmutató
│       │   ├── ANALYSIS_T3_v1.md            ✅ v1 elemzés (spawning hiba)
│       │   ├── ANALYSIS_T3_v2.md            ✅ v2 elemzés (sikeres!)
│       │   └── GAZEBO_HARMONIC_FIX.md       ✅ Kompatibilitási javítások
│       └── TESZT_OSSZEFOGLALO.md            ✅ Összefoglaló
├── results/                                  ✅ Mappa létrehozva
│   └── (rosbag fájlok ide kerülnek)
└── docs/
    ├── TODO_MitrengaMark.md                 ✅ Ez a fájl (frissítve 2025-10-29)
    ├── FUTTATAS_UTMUTATO.md                 ✅ Futtatási útmutató (frissítve)
    ├── MUNKA_OSSZEFOGLALO_2025-10-28.md     ✅ Előző munkamenet összefoglalója
    ├── TesztelésiTerv_MitrengaMark.pdf      ⏳ Overleaf
    └── Presentation_MitrengaMark.pdf        ⏳ Prezentáció
```

---

## 📚 Dokumentációs fájlok

1. **[README.md](../README.md)** - Projekt főoldal, gyors áttekintés
2. **[TODO_MitrengaMark.md](TODO_MitrengaMark.md)** (ez a fájl) - Részletes feladatlista, státuszok
3. **[FUTTATAS_UTMUTATO.md](FUTTATAS_UTMUTATO.md)** - Lépésről-lépésre rendszerindítási útmutató (v2.0)
4. **[MUNKA_OSSZEFOGLALO_2025-10-28.md](MUNKA_OSSZEFOGLALO_2025-10-28.md)** - Mai munkamenet összefoglalója

**Mindig ezeket a fájlokat frissítsd, amikor egy feladat elkészül!** ✅

## 📝 Állapot (2025. október 30. - 19:30)

### ✅ Kész feladatok:
- ✅ **ROS 2 környezet beállítva** - Jazzy build sikeres (1 csomag: lidar_filter)
- ✅ **Workspace tisztítva** - Felesleges csomagok törölve (gyak2-6, hamster_simulation)
- ✅ **lidar_filter csomag létrehozva** - Objektum detektálás implementálva
- ✅ **Setup.cfg javítás** - Node telepítés lib/lidar_filter könyvtárba helyesen
- ✅ **Node működik** - `/filtered_scan`, `/objects`, `/object_markers` topicok publikálva
- ✅ **TurtleBot3 telepítve** - apt-ból rendszerszinten (Gazebo, description, msgs, teleop)
- ✅ **Gazebo szimuláció fut** - TurtleBot3 Waffle modell LIDAR-ral
- ✅ **RViz2 konfiguráció optimalizálva** - Csak 3 TF frame, Map display eltávolítva
- ✅ **Complete system launch (v1)** - Egy paranccsal indul minden komponens
- ✅ **Optimized system launch (v2)** - CPU-optimalizált verzió
- ✅ **rqt_graph vizualizáció** - Node topológia megtekinthető
- ✅ **Tesztesetek dokumentálva** - T1, T2, T3 szcenáriók leírva
- ✅ **T1 teszt végrehajtva** - Rosbag rögzítve, metrikák gyűjtve
- ✅ **T2 teszt v1 futtatva** - Problémák azonosítva (hiányzó topic-ok, CPU 100%)
- ✅ **T2 optimalizálás** - RViz config, helyes topic nevek, új launch
- ✅ **T2 teszt v2 SIKERES!** - 237 objektum detektálva, teljes rosbag (15.2 MiB)
- ✅ **T2 eredmények elemzése** - Részletes összehasonlítás v1 vs v2 (ANALYSIS_T2_v2.md)
- ✅ **T3 teszt v1 futtatva** - Spawning sikertelen, problémák azonosítva
- ✅ **Gazebo Harmonic javítások** - continuous_spawn.sh átírva gz parancsokra
- ✅ **T3 teszt v2 SIKERES!** - 89 objektum detektálva (98.9%), scan rate 1.11 Hz (+29%!)
- ✅ **T3 eredmények elemzése** - T2 vs T3 összehasonlítás (ANALYSIS_T3_v2.md)
- ✅ **Dokumentáció frissítve (T3)** - README, TODO, notes, elemzések, javítások

### 🎯 Fő Eredmények (T1, T2, T3):
- **T1 (statikus):** Alap működés validálva
- **T2 (mozgó):** 99.6% detektálás, 0.86 Hz scan rate, 276.7s
- **T3 (stresszteszt):** 98.9% detektálás, 1.11 Hz scan rate (+29%), 81.7s

### ⏳ Következő lépések (prioritási sorrendben):
1. **Screenshot-ok készítése** - RViz, Gazebo, rqt_graph (T2 és T3) ⏳
2. **Metrikák elemzése** - CSV adatok, grafikonok (T1 vs T2 vs T3) ⏳
3. **Overleaf dokumentum** - Tesztelési terv írása (2-3 oldal) ⏳
4. **Tesztelési terv írása** - Overleaf dokumentum kezdése (Fázis 2)
5. **Prezentáció készítése** - 6-7 perces bemutató (Fázis 4)

### 🔧 Technikai megjegyzések:
- ✅ **Node telepítés javítva:** `setup.cfg` hozzáadásával a lib/lidar_filter mappába kerül
- ✅ **Topic nevek javítva:** `/filtered_scan`, `/objects`, `/object_markers` (NEM `/scan_filtered`!)
- ✅ **RViz optimalizálva:** Csak 3 TF frame (odom, base_link, base_scan), Map display eltávolítva
- ✅ **T2 teszt sikeres:** 237 objektum detektálva, 99.6% sikeres detektálás
- ⚠️ **CPU 100%:** WSL limitáció, GPU hiánya - teszteléshez elfogadható
- ⚠️ **Scan rate alacsony:** ~0.86 Hz (Gazebo lassúság) - T3-ban headless mode?
- 📦 **Rosbag adatok:** T2 v2: 276.7 sec, 50,338 üzenet, 15.2 MiB
- 🚀 **Indítás:** `ros2 launch lidar_filter optimized_system.launch.py`