# 📋 TODO Lista - Következő Fejlesztési Lépések

**Projekt:** MGM LIDAR Objektum Detektálás  
**Utolsó frissítés:** 2025-11-25  
**Státusz:** Fejlesztés alatt 🔄

---

## 🎯 Áttekintés

Ez a dokumentum tartalmazza a projekt következő fejlesztési lépéseit. A feladatok prioritás és függőség szerint vannak rendezve.

---

## 📝 Feladatok

### ✅ Kész Feladatok

- [x] Alapvető LIDAR szűrés implementálása
- [x] Egyszerű clustering algoritmus (távolság alapú)
- [x] RViz2 vizualizáció (markerek)
- [x] Rosbag támogatás
- [x] Alapvető tesztek (T1, T2, T3)
- [x] Metrika vizualizáció (Python matplotlib)
- [x] Dokumentáció (README, fejlesztői útmutató)

---

### 🔴 1. PRIORITÁS: DBSCAN Integráció és Objektum Címkézés

**Feladat:** Jelenlegi egyszerű clustering algoritmus lecserélése DBSCAN-re és perzisztens objektum címkézés implementálása.

#### 1.1. DBSCAN Algoritmus Integráció

**Cél:** Robusztusabb objektum detektálás
- [ ] `sklearn.cluster.DBSCAN` importálása a `lidar_filter_node.py`-ba
- [ ] `simple_clustering()` metódus átírása DBSCAN használatára
- [ ] Paraméterek finomhangolása:
  - `eps`: Klaszteren belüli max távolság (jelenlegi `cluster_threshold` ~0.2m)
  - `min_samples`: Min pontok száma klaszterben (jelenlegi `min_cluster_size` ~3)
- [ ] Tesztelés különböző paraméterekkel
- [ ] Összehasonlítás az eredeti algoritmussal (metrikák: sebesség, pontosság)

**Implementációs útmutató:**
```python
from sklearn.cluster import DBSCAN
import numpy as np

def dbscan_clustering(self, points, eps=0.2, min_samples=3):
    """
    DBSCAN alapú clustering
    
    Args:
        points: np.array, alakja (N, 2) - x,y koordináták
        eps: float, maximum távolság klaszteren belül (meter)
        min_samples: int, minimum pontok száma egy klaszterben
        
    Returns:
        clusters: list of np.array - klaszterenként a pontok listája
    """
    if len(points) < min_samples:
        return []
    
    # DBSCAN futtatása
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    
    # Klaszterek szeparálása (noise: label=-1)
    clusters = []
    unique_labels = set(labels)
    for label in unique_labels:
        if label == -1:  # Noise pontok kihagyása
            continue
        cluster_points = points[labels == label]
        clusters.append(cluster_points)
    
    return clusters
```

**Fájlok módosítása:**
- `src/mgm_gyak/lidar_filter/lidar_filter/lidar_filter_node.py`
- `src/mgm_gyak/lidar_filter/package.xml` (sklearn dependency)
- `src/mgm_gyak/lidar_filter/setup.py` (sklearn requirement)

#### 1.2. Perzisztens Objektum Címkézés

**Cél:** Objektumok egyedi ID-val való követése frame-ek között

**Követelmények:**
- ✅ Ha objektum látható RViz-ben → címke megjelenítése
- ✅ Ha objektum eltakarva (nem látható) → címke NEM jelenik meg
- ✅ Ha objektum újra megjelenik → UGYANAZ a címke (ID-tracking)
- ✅ Új objektum → új címke
- ✅ Objektum eltűnik végleg (timeout) → címke felszabadul

**Implementációs terv:**

```python
class ObjectTracker:
    """Objektum követő rendszer perzisztens ID-kkal"""
    
    def __init__(self, max_distance=0.5, timeout=2.0):
        """
        Args:
            max_distance: Max távolság objektumok közepességéhez (meter)
            timeout: Idő, amíg objektum ID megmarad eltűnés után (sec)
        """
        self.tracked_objects = {}  # {id: {'position': (x,y), 'last_seen': time}}
        self.next_id = 0
        self.max_distance = max_distance
        self.timeout = timeout
    
    def update(self, current_objects, current_time):
        """
        Frissíti a követett objektumokat
        
        Args:
            current_objects: list of (x, y) tuple - detektált objektumok
            current_time: float - jelenlegi idő (seconds)
            
        Returns:
            object_ids: list of int - objektumok ID-i
        """
        # Implement: Hungarian algorithm vagy nearest neighbor matching
        pass
    
    def get_visible_objects(self):
        """
        Visszaadja a látható objektumokat ID-val
        
        Returns:
            list of (id, x, y) tuple
        """
        pass
    
    def cleanup_old_objects(self, current_time):
        """Timeout-olt objektumok törlése"""
        pass
```

**Címke megjelenítés RViz-ben:**
- `visualization_msgs/MarkerArray` használata TEXT típusú markerekkel
- Marker ID = objektum ID
- Szöveg: f"OBJ_{id}"
- Pozíció: objektum felett (z = 0.5m)
- Csak akkor publikál, ha objektum látható

**Új topic:**
- `/object_labels` (visualization_msgs/MarkerArray) - szöveges címkék

**Fájlok módosítása:**
- `src/mgm_gyak/lidar_filter/lidar_filter/lidar_filter_node.py`
  - Új `ObjectTracker` osztály hozzáadása
  - `scan_callback()` módosítása tracker használatához
  - Új `create_label_markers()` metódus

**Tesztelés:**
- [ ] Objektum megjelenik → címke megjelenik
- [ ] Objektum eltakarva → címke eltűnik
- [ ] Objektum újra látható → UGYANAZ a címke
- [ ] Robot mozog → címkék megfelelően követik objektumokat
- [ ] Új objektum → új egyedi címke

---

### 🟠 2. PRIORITÁS: Új Szimulátor Pálya Integrálása

**Feladat:** További Gazebo world(ök) hozzáadása a tesztelés diverzifikálásához.

#### 2.1. Meglévő Pályák Felmérése

- [ ] TurtleBot3 hivatalos world-ök ellenőrzése:
  - `turtlebot3_world.launch.py` (jelenlegi)
  - `turtlebot3_house.launch.py`
  - `turtlebot3_stage_*.launch.py`
- [ ] Gazebo model database böngészése: http://models.gazebosim.org/
- [ ] Közösségi pályák keresése (GitHub, ROS2 community)

#### 2.2. Pálya Kiválasztása

**Kritériumok:**
- ✅ Változatos objektumok (dobozok, hengerek, falak)
- ✅ Különböző távolságok (0.5m - 5m)
- ✅ Dinamikus elemek (opcionális: mozgó objektumok)
- ✅ ROS2 Jazzy/Humble kompatibilis

**Javasolt opciók:**

1. **TurtleBot3 House** (könnyű)
   - Előny: Hivatalos, jól támogatott, komplex beltéri környezet
   - Használat: `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`

2. **Egyedi world készítése** (közepes)
   - SDF/World fájl írása
   - Különböző geometriák elhelyezése
   - Mentés: `worlds/custom_world.world`

3. **AWS RoboMaker worlds** (nehéz)
   - Nagy, realisztikus környezetek
   - Telepítés: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

#### 2.3. Integráció

- [ ] World fájl hozzáadása: `config/worlds/new_world.world`
- [ ] Új launch fájl készítése: `launch/complete_system_newworld.launch.py`
- [ ] Launch fájl módosítása:
```python
world_file = os.path.join(
    get_package_share_directory('lidar_filter'),
    'worlds',
    'new_world.world'
)

gazebo = Node(
    package='gazebo_ros',
    executable='gzserver',
    arguments=[world_file, '--verbose'],
    # ...
)
```
- [ ] Package.xml és CMakeLists.txt frissítése (world fájl telepítése)
- [ ] RViz konfig adaptálása új pályához (ha szükséges)
- [ ] README frissítése új launch paranccsal

#### 2.4. Dokumentáció

- [ ] Új pálya leírása `README.md`-ben
- [ ] Futtatási útmutató `FUTTATAS_UTMUTATO.md`-ben
- [ ] Screenshot hozzáadása: `tests/screenshots/new_world.png`

---

### 🟡 3. PRIORITÁS: Új Kiértékelési Metrikák

**Feladat:** Pontosság (accuracy) és további metrikák implementálása az objektum detektálás minőségének mérésére.

#### 3.1. Ground Truth Adatok

**Probléma:** Jelenleg nincs reference adat az objektumok valós pozíciójáról.

**Megoldási lehetőségek:**

1. **Gazebo model state topic használata:**
```python
# Gazebo model pozíciók lekérése
from gazebo_msgs.srv import GetModelState

def get_ground_truth_positions(self):
    """Valós objektum pozíciók Gazebo-ból"""
    # Implement: Service call gazebo/get_model_state
    pass
```

2. **Manuális annotálás:**
- RViz-ben kézzel megjelölni objektumok pozícióját
- YAML fájlba menteni
- Összehasonlítás detektált objektumokkal

#### 3.2. Metrikák Implementálása

**Új metrikák:**

| Metrika | Leírás | Számítás |
|---------|--------|----------|
| **Precision** | Detektált objektumok közül hány valós | TP / (TP + FP) |
| **Recall** | Valós objektumok közül hány detektálva | TP / (TP + FN) |
| **F1-Score** | Precision és Recall harmonikus átlaga | 2 × (P × R) / (P + R) |
| **IoU** | Intersection over Union (térbeli átfedés) | Átfedés / Unió |
| **Position Error** | Átlagos pozíciós hiba (meter) | mean(‖p_det - p_gt‖) |
| **Detection Rate** | Detektálások / frame | count / time |

**True Positive (TP):** Detektált objektum közel van ground truth-hoz (<0.3m)  
**False Positive (FP):** Detektált objektum nincs ground truth közelben  
**False Negative (FN):** Ground truth objektum nem lett detektálva

#### 3.3. Implementáció

**Új Python modul:**
```bash
tests/test_results/evaluate_metrics.py
```

**Struktúra:**
```python
class ObjectDetectionEvaluator:
    """Objektum detektálás kiértékelő"""
    
    def __init__(self, distance_threshold=0.3):
        self.distance_threshold = distance_threshold
        self.true_positives = []
        self.false_positives = []
        self.false_negatives = []
    
    def evaluate_frame(self, detected_objects, ground_truth_objects):
        """Egy frame kiértékelése"""
        # Hungarian algorithm: match detected ↔ ground truth
        # Távolság threshold alapján (TP/FP/FN)
        pass
    
    def calculate_metrics(self):
        """Metrikák számítása összes frame alapján"""
        precision = len(self.true_positives) / (len(self.true_positives) + len(self.false_positives))
        recall = len(self.true_positives) / (len(self.true_positives) + len(self.false_negatives))
        f1_score = 2 * (precision * recall) / (precision + recall)
        # ...
        return metrics_dict
    
    def generate_report(self, output_file):
        """Részletes report generálása (PDF/HTML)"""
        pass
```

**Vizualizáció bővítése:**
- `visualize_metrics.py` frissítése új metrikákkal
- Confusion matrix hozzáadása
- Precision-Recall görbe
- Position error heatmap

#### 3.4. Tesztelés

- [ ] Ground truth adatok összeállítása (legalább 100 frame)
- [ ] Evaluator futtatása meglévő rosbag-eken
- [ ] Metrikák számítása mindhárom teszt esetére (T1, T2, T3)
- [ ] Összehasonlító táblázat:

| Teszt | Precision | Recall | F1-Score | Pos. Error | Det. Rate |
|-------|-----------|--------|----------|------------|-----------|
| T1    | ?         | ?      | ?        | ? m        | ? Hz      |
| T2    | ?         | ?      | ?        | ? m        | ? Hz      |
| T3    | ?         | ?      | ?        | ? m        | ? Hz      |

- [ ] Eredmények dokumentálása `TESZT_OSSZEFOGLALO.md`-ben

---

### 🟢 4. PRIORITÁS: Teljes Rendszer Tesztelés és Videó Demó

**Feladat:** Minden implementált funkció integrált tesztelése mindkét pályán, majd demó videó készítése.

#### 4.1. Teljes Rendszer Teszt

**Előfeltételek:**
- ✅ DBSCAN implementálva
- ✅ Objektum címkézés működik
- ✅ Második pálya integrálva
- ✅ Új metrikák számíthatók

**Teszt forgatókönyv:**

**Pálya 1: TurtleBot3 World (eredeti)**
- [ ] T1-Static: Statikus objektumok (5 perc rosbag)
- [ ] T2-Moving: Robot mozog, objektumok statikusak (5 perc)
- [ ] T3-Dynamic: Komplex mozgás (5 perc)

**Pálya 2: Új pálya (pl. TurtleBot3 House)**
- [ ] T4-House-Static: Új környezet statikus teszt (5 perc)
- [ ] T5-House-Moving: Új környezet mozgásos teszt (5 perc)

**Minden tesztre:**
- [ ] Rosbag mentés
- [ ] Metrikák számítása (precision, recall, F1, stb.)
- [ ] Vizualizációk generálása
- [ ] Screenshot-ok (legalább 5/teszt)

#### 4.2. Teljesítmény Benchmarking

**CPU/RAM mérés:**
```bash
# Terminal 1: Rendszer indítása
ros2 launch lidar_filter complete_system.launch.py

# Terminal 2: Resource monitoring
top -p $(pgrep -f lidar_filter_node) -b -d 1 > cpu_usage.log

# Terminal 3: Memory tracking
ps aux | grep lidar_filter_node > memory_usage.log
```

**Latency mérés:**
```bash
ros2 topic delay /objects
ros2 topic hz /objects
```

**Eredmények táblázata:**
| Pálya | CPU (%) | RAM (MB) | Latency (ms) | Frequency (Hz) |
|-------|---------|----------|--------------|----------------|
| World | ?       | ?        | ?            | ?              |
| House | ?       | ?        | ?            | ?              |

#### 4.3. Demó Videó Készítése

**Cél:** 3-5 perces demó videó a rendszer működéséről

**Tartalmi elemek:**
1. **Intro (15 sec)**
   - Projekt név, cél
   - Használt technológiák (ROS2, Python, Gazebo)

2. **Rendszer Architektúra (30 sec)**
   - Diagram: Gazebo → LIDAR → Filter Node → RViz
   - Főbb komponensek ismertetése

3. **Objektum Detektálás Demó (90 sec)**
   - Gazebo + RViz egymás mellett (split screen)
   - Robot mozog a pályán
   - Objektumok detektálása valós időben
   - Címkék megjelenítése (OBJ_1, OBJ_2, stb.)
   - Objektum eltakarás → címke eltűnés → újramegjelenés ugyanazzal az ID-val

4. **Második Pálya (60 sec)**
   - Új környezet bemutatása
   - Működés hasonló körülmények között
   - Robusztusság demonstrálása

5. **Metrikák és Eredmények (45 sec)**
   - Grafikonok megjelenítése (matplotlib output)
   - Precision, Recall, F1-Score értékek
   - Összehasonlító táblázatok

6. **Outro (15 sec)**
   - Projekt státusz: Kész ✅
   - GitHub link
   - Köszönetnyilvánítás

**Eszközök:**
- **Screen recording:** SimpleScreenRecorder (Linux)
```bash
sudo apt install simplescreenrecorder
```
- **Video editing:** Kdenlive vagy OpenShot
```bash
sudo apt install kdenlive
```
- **Annotációk:** Screencast overlay (szöveg, nyilak)

**Felvétel parancsok:**
```bash
# Terminal 1: Rendszer indítása
ros2 launch lidar_filter complete_system.launch.py

# Terminal 2: Robot mozgatása (scriptelve)
ros2 run turtlebot3_teleop teleop_keyboard

# Vagy automatikus mozgás:
ros2 run turtlebot3_gazebo turtlebot3_drive
```

**Videó exportálás:**
- Formátum: MP4 (H.264 codec)
- Felbontás: 1920×1080 (Full HD)
- Framerate: 30 FPS
- Bitrate: 5-10 Mbps
- Fájl méret: ~50-150 MB (3-5 perc)

---

### 🔵 5. PRIORITÁS: Prezentáció Készítése

**Feladat:** Projekt prezentáció készítése a működés és eredmények bemutatására (demóval együtt).

#### 5.1. Prezentáció Struktúra

**Javasolt slide-ok (20-25 dia):**

1. **Címlap**
   - Projekt név: "LIDAR Alapú Objektum Detektálás és Követés"
   - Készítő(k)
   - Dátum
   - Intézmény/Kurzus

2. **Tartalomjegyzék** (1 dia)

3. **Bevezetés** (2-3 dia)
   - Probléma megfogalmazása
   - Miért fontos az objektum detektálás robotikában?
   - Projekt célkitűzései

4. **Technológiai Háttér** (3-4 dia)
   - ROS2 (Robot Operating System 2)
   - LIDAR szenzor működése
   - Gazebo szimulátor
   - Python + NumPy/sklearn

5. **Rendszer Architektúra** (2-3 dia)
   - Komponensdiagram
   - ROS2 node-ok és topic-ok
   - Adatfolyam (LIDAR → szűrés → clustering → vizualizáció)

6. **Implementáció** (4-5 dia)
   - LIDAR adatszűrés (min/max range)
   - Koordináta transzformáció (polár → descartes)
   - **DBSCAN clustering algoritmus**
   - **Objektum címkézés és követés**
   - RViz vizualizáció

7. **DBSCAN Algoritmus Részletesen** (2 dia)
   - Mi az a DBSCAN? (Density-Based Spatial Clustering)
   - Paraméterek: eps, min_samples
   - Miért jobb az egyszerű távolság alapú clustering-nél?
   - Előtt/után összehasonlítás

8. **Objektum Tracking** (2 dia)
   - Perzisztens ID rendszer
   - Címke megjelenítés logikája
   - Timeout kezelés

9. **Tesztelés** (3-4 dia)
   - Teszt forgatókönyvek (T1-T5)
   - Két különböző pálya
   - Rosbag alapú elemzés

10. **Eredmények** (4-5 dia)
    - **Metrikák táblázata** (Precision, Recall, F1-Score)
    - Grafikonok (matplotlib output)
    - Teljesítmény adatok (CPU, RAM, latency)
    - Összehasonlítás (pályák, algoritmusok)

11. **Demó Videó Beágyazása** (1 dia)
    - "Élő" működés bemutatása
    - Vagy videó lejátszás prezentáció közben

12. **Tapasztalatok és Kihívások** (2 dia)
    - Nehézségek (pl. LIDAR zajszűrés, objektum matching)
    - Megoldások
    - Lessons learned

13. **Jövőbeli Fejlesztések** (1 dia)
    - Kalman filter az objektum tracking-hez
    - Több szenzor fúziója (LIDAR + kamera)
    - Valós robot tesztelés (TurtleBot3 hardware)
    - Obstacle avoidance integráció

14. **Összefoglalás** (1 dia)
    - Projekt célok teljesítése ✅
    - Főbb eredmények
    - Tanulságok

15. **Köszönetnyilvánítás** (1 dia)
    - Konzulensek, tanárok
    - Használt open-source projektek
    - GitHub repository link

16. **Kérdések és Válaszok** (1 dia)

#### 5.2. Vizuális Elemek

**Diagram-ok készítése:**
- **draw.io:** https://app.diagrams.net/
- **PlantUML:** Text-based diagramok
- **Inkscape:** Vektorgrafika

**Példa diagram (ROS2 node graph):**
```
┌──────────┐       /scan        ┌─────────────────┐
│  Gazebo  │ ─────────────────► │ lidar_filter_   │
│TurtleBot3│                     │      node       │
└──────────┘                     └────────┬────────┘
                                          │
                     ┌────────────────────┼────────────────────┐
                     │                    │                    │
              /filtered_scan         /objects          /object_markers
                     │                    │                    │
                     ▼                    ▼                    ▼
                ┌─────────────────────────────────────────────┐
                │               RViz2                         │
                │  [3D Vizualizáció + Objektum Címkék]       │
                └─────────────────────────────────────────────┘
```

**Screenshot-ok:**
- Gazebo + RViz split screen
- RViz közelkép objektum címkékkel
- Metrika grafikonok (matplotlib)
- Terminal output (topic listázás, node info)

#### 5.3. Prezentációs Eszközök

**Lehetőségek:**

1. **LibreOffice Impress** (ingyenes, Linux natív)
```bash
sudo apt install libreoffice-impress
```

2. **Google Slides** (online, collaborative)

3. **LaTeX Beamer** (professzionális, verziókezelt)
```latex
\documentclass{beamer}
\usetheme{Madrid}
\title{LIDAR Objektum Detektálás}
% ...
```

4. **Reveal.js** (HTML alapú, programozói stílus)

**Ajánlás:** LibreOffice Impress vagy Google Slides (egyszerűség és kompatibilitás miatt)

#### 5.4. Elkészítési Lépések

- [ ] Vázlat írása (markdown vagy outline)
- [ ] Diagram-ok elkészítése (draw.io)
- [ ] Screenshot-ok kiválasztása és annotálása
- [ ] Grafikonok exportálása (PNG, 300 DPI)
- [ ] Slide-ok összeállítása
- [ ] Szöveges tartalom írása (rövid, lényegre törő)
- [ ] Videó beágyazása vagy lejátszás tesztelése
- [ ] Presenter notes írása (saját emlékeztető)
- [ ] Próba prezentáció (időzítés: 10-15 perc)
- [ ] Finalizálás és exportálás (PDF + PPTX formátumban)

#### 5.5. Prezentálási Tippek

- **Időbeosztás:** 1 dia ≈ 30-60 másodperc
- **Szöveg mennyiség:** Max 5-7 sor/dia, rövid mondatok
- **Vizuális elsőbbség:** Több kép/diagram, kevesebb szöveg
- **Demó:** Élő demó VAGY videó (videó megbízhatóbb)
- **Interakció:** Kérdések bátorítása
- **Backup:** PDF verzió, ha technikai probléma van

---

## 📊 Összesített Timeline

**Becsült időigény:**

| Feladat | Időigény | Függőségek |
|---------|----------|------------|
| 1. DBSCAN + Tracking | 15-20 óra | - |
| 2. Új pálya | 5-8 óra | - |
| 3. Új metrikák | 10-15 óra | Ground truth adatok |
| 4. Tesztelés + Videó | 8-12 óra | 1, 2, 3 kész |
| 5. Prezentáció | 6-10 óra | 4 kész |
| **Összesen** | **44-65 óra** | (~1-1.5 hét full-time) |

**Javasolt sorrend:** 1 → 2 → 3 → 4 → 5 (ahogy a prioritás mutatja)

---

## 🔧 Technikai Követelmények

**Új Python csomagok:**
```bash
pip3 install scikit-learn  # DBSCAN
pip3 install scipy         # Hungarian algorithm (objektum matching)
pip3 install pandas        # Metrika kezelés
```

**Vagy apt:**
```bash
sudo apt install python3-sklearn python3-scipy python3-pandas
```

**ROS2 csomagok:**
```bash
# Gazebo service client (ground truth)
sudo apt install ros-jazzy-gazebo-msgs

# További vizualizáció (opcionális)
sudo apt install ros-jazzy-rviz-visual-tools
```

---

## ✅ Definition of Done (DoD)

Egy feladat csak akkor tekinthető **KÉSZ**-nek, ha:

- [ ] Kód implementálva és működik
- [ ] Tesztekkel validálva (legalább manuális teszt)
- [ ] Dokumentáció frissítve (README, comments)
- [ ] Git commit + push (értelmes commit message)
- [ ] Nincs syntax error, warning (legalább a kritikus fájlokban)
- [ ] Review-zva (ha csapatban dolgozol)

---

## 📞 Kérdések és Segítség

Ha elakadsz valamelyik feladatnál:

1. **DBSCAN:** sklearn dokumentáció: https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
2. **Objektum Tracking:** SORT algoritmus referencia: https://github.com/abewley/sort
3. **Gazebo API:** http://docs.ros.org/en/jazzy/p/gazebo_msgs/
4. **RViz markerek:** http://wiki.ros.org/rviz/DisplayTypes/Marker

**GitHub Issues:** Ha bug-ot találsz vagy kérdésed van, nyiss issue-t a repo-ban.

---

## 🚀 Indulás!

Válaszd ki az első feladatot (DBSCAN integráció) és kezdj bele! Ne feledd:

> "A longest journey begins with a single step." - Lao Tzu

**Sok sikert a fejlesztéshez!** 💪

---

**Készítette:** Mitrenga Márk  
**Projekt:** MGM LIDAR Objektum Detektálás  
**Repository:** https://github.com/mitrengamark/project_mgm
