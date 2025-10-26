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

| Fázis | Cél | Részfeladatok | Eredmény |
|-------|-----|----------------|-----------|
| 1️⃣ Tesztkörnyezet előkészítés | ROS2 workspace és tesztadatok előkészítése | Build, rosbag futtatás, RViz megjelenítés | működő tesztkörnyezet |
| 2️⃣ Tesztelési terv kidolgozása | Szcenáriók, metrikák, futtatási terv | 3 különböző teszteset dokumentálása | Overleaf PDF (2–3 oldal) |
| 3️⃣ Tesztfuttatás és adatgyűjtés | Bag replay, CPU/latency mérés, screenshotok | Futtatási napló, ábrák, metrikák | teszt report és ábrák |
| 4️⃣ Prezentáció | Összefoglaló 6–7 perces bemutató | Ábrák, mérési eredmények, konklúzió | ppt/beamer export Overleaf-ből |

---

## ⚙️ 1. Tesztkörnyezet előkészítése

**Cél:** a ROS2 rendszer futtatása, a workspace és a szimulációs adatforrások ellenőrzése.

### 1.1 Build és környezet inicializálás

```bash
cd ~/codes/mgm/project_mgm
colcon build --symlink-install
source install/setup.bash
```

### 1.2 Alap teszt – LIDAR node-ok és launch fájl

```bash
ros2 launch lidar_filter lidar_filter.launch.py
ros2 node list
ros2 topic list
ros2 topic echo /scan
```

### 1.3 Rosbag lejátszás és RViz megjelenítés

```bash
ros2 bag play data/test_scene.bag --clock
ros2 run rviz2 rviz2
```

### 1.4 Ellenőrizd:

- `/scan` → LIDAR adatfolyam látható
- `/filtered_scan`, `/objects`, `/map` topicok megjelennek
- `rqt_graph`-ban látszik a node-topology

### 1.5 Logolás és mentés

```bash
mkdir -p ~/codes/mgm/project_mgm/results
ros2 bag record -a -o results/test_run1
```

---

## 🧪 2. Tesztelési terv kidolgozása

**Cél:** meghatározni, mit, hogyan és mivel fogsz mérni.

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
```css
project_mgm/
 ├── src/
 │   ├── lidar_filter/
 │   │   ├── lidar_filter_node.py
 │   │   └── launch/lidar_filter.launch.py
 ├── tests/
 │   ├── test_cases.md
 │   ├── test_results/
 │   │   ├── run1.png
 │   │   ├── run2.png
 │   │   └── metrics.csv
 ├── results/
 │   ├── run1/
 │   ├── run2/
 │   └── summary.txt
 ├── docs/
 │   ├── TODO_MitrengaMark.md
 │   ├── TesztelésiTerv_MitrengaMark.pdf
 │   └── Presentation_MitrengaMark.pdf
 └── README.md
 ```