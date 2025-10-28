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
| 2️⃣ Tesztelési terv kidolgozása | Szcenáriók, metrikák, futtatási terv | 3 különböző teszteset dokumentálása | Overleaf PDF (2–3 oldal) | 🔶 30% |
| 3️⃣ Tesztfuttatás és adatgyűjtés | Bag replay, CPU/latency mérés, screenshotok | Futtatási napló, ábrák, metrikák | teszt report és ábrák | 🔶 10% |
| 4️⃣ Prezentáció | Összefoglaló 6–7 perces bemutató | Ábrák, mérési eredmények, konklúzió | ppt/beamer export Overleaf-ből | ⏳ 0% |

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
ros2 bag record -a -o results/test_run1
```

**Státusz:** ✅ Első teszteset (T1) rögzítve!
- ✅ T1 (statikus környezet) rosbag rögzítve
- ✅ Metrikák gyűjtve (CSV formátum)
- ✅ Teszt jegyzet készítve
- ⏳ T2 és T3 tesztesetek következnek

---

## 🧪 2. Tesztelési terv kidolgozása

**Cél:** meghatározni, mit, hogyan és mivel fogsz mérni.

**Státusz:** ⏳ Még nem kezdődött el

### 2.1 Tesztszcenáriók

| ID | Leírás | Elvárt eredmény |
|----|--------|-----------------|
| T1 | Statikus akadály (fix tárgy) | Stabil pozíció, alacsony hiba (<5cm) |
| T2 | Mozgó objektum (bag replay) | Objektum azonosítása és folytonos követés |
| T3 | Több objektum térképezése | Konzisztens térkép, 90%+ detektálási arány |

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

**Státusz:** ⏳ Még nem kezdődött el

### 3.1 Futtatás

```bash
ros2 launch lidar_filter lidar_filter.launch.py
ros2 bag play data/test_scene.bag --clock
```

### 3.2 Adatrögzítés

```bash
ros2 bag record /scan /objects /map -o results/run2
```

### 3.3 Metrikák számítása

Python script vagy Jupyter segítségével:

```python
import rosbag2_py
import numpy as np
# mérési statisztikák, késleltetés és pozíciós hiba számítása
```

### 3.4 Eredmények

- **Ábrák:** RViz, rqt_graph, PlotJuggler képernyőmentések
- **Táblázat:** Precision/Recall, latency, CPU-idő
- Minden tesztfuttatás mentve a `results/` mappába

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
│       └── lidar_filter/                    ✅ Létrehozva
│           ├── lidar_filter/
│           │   ├── __init__.py              ✅
│           │   └── lidar_filter_node.py     ✅ LIDAR objektum detektálás
│           ├── launch/
│           │   ├── lidar_filter.launch.py   ✅ Egyszerű launch
│           │   └── complete_system.launch.py ✅ ÚJ! Teljes rendszer launch
│           ├── config/
│           │   └── lidar_filter_rviz.rviz   ✅ ÚJ! RViz konfiguráció
│           ├── resource/
│           │   └── lidar_filter             ✅
│           ├── package.xml                   ✅ ROS 2 package
│           ├── setup.py                      ✅ Python setup
│           └── README.md                     ✅ ÚJ! Csomag dokumentáció
├── install/lidar_filter/                     ✅ Build output
│   └── bin/lidar_filter_node                 ✅ Executable itt van!
├── config/
│   └── lidar_filter_rviz.rviz               ✅ ÚJ! RViz konfiguráció (másolat)
├── tests/                                    ✅ Tesztek megkezdve!
│   ├── test_cases.md                        ✅ Tesztesetek dokumentálva (T1, T2, T3)
│   └── test_results/
│       ├── T1_static/                       ✅ T1 KÉSZ!
│       │   ├── rosbag/
│       │   │   └── test_run1_static/        ✅ Rosbag rögzítve (~6 sec)
│       │   ├── screenshots/                 ⏳ Következik
│       │   ├── metrics_t1.csv               ✅ Metrikák CSV-ben
│       │   └── notes_t1.md                  ✅ Teszt jegyzet
│       ├── T2_moving/                       ⏳ Előkészítve
│       ├── T3_stress/                       ⏳ Előkészítve
│       └── TESZT_OSSZEFOGLALO.md            ✅ ÚJ! Összefoglaló
├── results/                                  ✅ Mappa létrehozva
│   └── (rosbag fájlok ide kerülnek)
└── docs/
    ├── TODO_MitrengaMark.md                 ✅ Ez a fájl (frissítve)
    ├── FUTTATAS_UTMUTATO.md                 ✅ Futtatási útmutató (v2.0)
    ├── MUNKA_OSSZEFOGLALO_2025-10-28.md     ✅ ÚJ! Mai munkamenet összefoglalója
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

## 📝 Állapot (2025. október 28. - 23:55)

### ✅ Kész feladatok:
- ✅ **ROS 2 környezet beállítva** - Jazzy build sikeres (29 csomag)
- ✅ **lidar_filter csomag létrehozva** - Objektum detektálás implementálva
- ✅ **Node működik** - `/filtered_scan`, `/objects`, `/object_markers`, `/map` topicok publikálva
- ✅ **Gazebo szimuláció fut** - TurtleBot3 Waffle modell LIDAR-ral
- ✅ **RViz2 konfiguráció** - Display-ek beállítva és .rviz fájl mentve
- ✅ **Complete system launch** - Egy paranccsal indul minden komponens
- ✅ **rqt_graph vizualizáció** - Node topológia megtekinthető
- ✅ **Tesztesetek dokumentálva** - T1, T2, T3 szcenáriók leírva
- ✅ **T1 teszt végrehajtva** - Rosbag rögzítve, metrikák gyűjtve
- ✅ **Dokumentáció** - README, TODO, Futtatási útmutató, Teszt jegyzet

### ⏳ Következő lépések (prioritási sorrendben):
1. **T2 és T3 tesztek futtatása** - Mozgó robot és stresszteszt
2. **Screenshot készítés** - RViz, rqt_graph, Gazebo képernyőmentések
3. **Gazebo LIDAR konfiguráció javítása** - Frekvencia növelése
4. **Metrikák elemzése** - CSV → grafikonok, táblázatok
5. **Tesztelési terv írása** - Overleaf dokumentum kezdése (Fázis 2)

### 🔧 Technikai megjegyzések:
- A `lidar_filter_node` executable a `install/lidar_filter/bin/` mappában található
- Conda környezetet NE aktiváld ROS 2 használatakor (rendszer Python kell)
- Gazebo indítás: `export TURTLEBOT3_MODEL=waffle` kell előtte
- Teljes rendszer indítás: `ros2 launch lidar_filter complete_system.launch.py`
- ⚠️ **LIDAR frekvencia alacsony** (~0.9 Hz helyett 10 Hz) - Gazebo konfiguráció javítandó!