# 🚀 Fejlesztői Útmutató - MGM LIDAR Projekt

**Projekt:** LIDAR alapú objektum detektálás és követés  
**ROS2 Verzió:** Jazzy (de más verziókkal is kompatibilis)  
**Utolsó frissítés:** 2025-11-25

---

## 📋 Tartalomjegyzék

1. [Projekt Áttekintés](#projekt-áttekintés)
2. [Rendszerkövetelmények](#rendszerkövetelmények)
3. [Telepítés és Beállítás](#telepítés-és-beállítás)
4. [Projekt Struktúra](#projekt-struktúra)
5. [Rendszer Működése](#rendszer-működése)
6. [Futtatási Útmutatók](#futtatási-útmutatók)
7. [Fejlesztési Workflow](#fejlesztési-workflow)
8. [Tesztelés](#tesztelés)
9. [Hibaelhárítás](#hibaelhárítás)
10. [Hasznos Parancsok](#hasznos-parancsok)

---

## 🎯 Projekt Áttekintés

Ez a projekt egy **valós idejű LIDAR alapú objektum detektáló és követő rendszer** ROS2 környezetben. A rendszer TurtleBot3 robotra lett kifejlesztve, de bármilyen LIDAR szenzorral rendelkező roboton működik.

### Főbb Funkciók

- **LIDAR adat szűrés:** Távolság alapú zajszűrés
- **Objektum detektálás:** Clustering algoritmus (távolság alapú)
- **Valós idejű vizualizáció:** RViz2 integráció
- **Rosbag támogatás:** Teszteléshez és elemzéshez
- **Metrika vizualizáció:** Python alapú grafikonok és statisztikák

### Használt Technológiák

- **ROS2:** Robot Operating System 2 (Jazzy/Humble/Iron kompatibilis)
- **Python 3:** Fő programozási nyelv
- **Gazebo:** 3D robot szimulátor
- **RViz2:** 3D vizualizációs eszköz
- **NumPy:** Numerikus számítások
- **Matplotlib:** Grafikon generálás

---

## 💻 Rendszerkövetelmények

### Operációs Rendszer Támogatás

A projekt az alábbi Linux disztribúciókon tesztelt:

| Disztribúció | Verzió | ROS2 Verzió | Státusz |
|--------------|--------|-------------|---------|
| **Ubuntu** | 24.04 LTS (Noble) | Jazzy | ✅ Teljesen támogatott |
| **Ubuntu** | 22.04 LTS (Jammy) | Humble | ✅ Kompatibilis |
| **Ubuntu** | 24.04 LTS | Iron | ✅ Kompatibilis |
| **WSL2 Ubuntu** | 22.04+ | Jazzy/Humble | ✅ Működik (GUI-val) |
| **Debian** | 12+ | Jazzy/Humble | ⚠️ Teszteletlen, de működnie kell |

> **Megjegyzés:** WSL2-n teljes funkcionalitás elérhető, de némi teljesítmény-csökkenés várható. Natív Linux ajánlott éles használatra.

### Hardver Követelmények

**Minimum:**
- CPU: 4 mag, 2.0 GHz
- RAM: 8 GB
- GPU: Integrált (Gazebo alapvető működéshez)
- Tárhely: 10 GB szabad hely

**Ajánlott:**
- CPU: 6+ mag, 3.0+ GHz
- RAM: 16 GB
- GPU: Dedikált (Gazebo gördülékeny működéséhez)
- Tárhely: 20 GB szabad hely

### Szoftver Követelmények

- **ROS2:** Jazzy / Humble / Iron
- **Python:** 3.10+
- **pip csomagok:** numpy, matplotlib, pandas (vizualizációhoz)
- **Gazebo:** Harmonic / Garden (ROS2 verzióval kompatibilis)
- **TurtleBot3 csomagok:** turtlebot3, turtlebot3_simulations

---

## 🔧 Telepítés és Beállítás

### 1. ROS2 Telepítése

#### Ubuntu 24.04 + ROS2 Jazzy

```bash
# Locale beállítása
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 repository hozzáadása
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Jazzy telepítése
sudo apt update
sudo apt install -y ros-jazzy-desktop

# ROS2 development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

#### Ubuntu 22.04 + ROS2 Humble

```bash
# Humble esetén ugyanaz a folyamat, csak:
sudo apt install -y ros-humble-desktop
```

### 2. TurtleBot3 Csomagok Telepítése

```bash
# ROS2 Jazzy esetén
sudo apt install -y ros-jazzy-turtlebot3* ros-jazzy-gazebo-ros-pkgs

# ROS2 Humble esetén
sudo apt install -y ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
```

### 3. Python Függőségek Telepítése

```bash
# Rendszer Python használata (ne conda/virtualenv!)
sudo apt install -y python3-pip python3-numpy python3-matplotlib python3-pandas

# Vagy pip-pel (ha nem találja a rendszer):
pip3 install numpy matplotlib pandas
```

### 4. Projekt Klónozása és Build

```bash
# Projekt klónozása
cd ~
git clone https://github.com/mitrengamark/project_mgm.git
cd project_mgm

# Függőségek telepítése rosdep-pel
rosdep install --from-paths src --ignore-src -r -y

# Workspace build
colcon build --symlink-install

# Source setup fájl
source install/setup.bash
```

### 5. Környezeti Változók Beállítása

Ajánlott a `.bashrc` fájlba beírni:

```bash
# Szerkesztés
nano ~/.bashrc

# Hozzáadni a fájl végére:
# ROS2 alapértelmezett verzió
source /opt/ros/jazzy/setup.bash  # vagy humble/iron

# Projekt workspace (módosítsd a saját elérési utadra!)
source ~/project_mgm/install/setup.bash

# TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Domain ID (ha több ROS2 rendszer fut egyszerre)
export ROS_DOMAIN_ID=30

# Mentés és kilépés: Ctrl+O, Enter, Ctrl+X
```

Majd:
```bash
source ~/.bashrc
```

---

## 📁 Projekt Struktúra

```
project_mgm/
│
├── README.md                           # Projekt főoldal
├── PROJEKT_STATUS.md                   # Állapot összefoglaló
├── FEJLESZTOI_UTMUTATO.md             # Ez a fájl
│
├── src/                                # ROS2 csomagok forrása
│   └── mgm_gyak/
│       └── lidar_filter/               # Fő csomag
│           ├── lidar_filter/
│           │   ├── __init__.py
│           │   └── lidar_filter_node.py    # 🔥 FŐ NODE
│           │
│           ├── launch/
│           │   ├── lidar_filter.launch.py          # Egyszerű launch
│           │   ├── complete_system.launch.py       # Teljes rendszer
│           │   └── optimized_system.launch.py      # Optimalizált
│           │
│           ├── config/
│           │   ├── lidar_filter_rviz.rviz          # RViz konfig
│           │   └── lidar_filter_optimized.rviz     # RViz optimalizált
│           │
│           ├── setup.py                    # Python csomag setup
│           ├── package.xml                 # ROS2 package manifest
│           └── README.md                   # Csomag dokumentáció
│
├── tests/                              # Tesztek és eredmények
│   ├── test_cases.md                   # Teszt forgatókönyvek
│   ├── test_results/
│   │   ├── TESZT_OSSZEFOGLALO.md      # Eredmények összefoglalója
│   │   ├── visualize_metrics.py        # 🔥 Vizualizációs eszköz
│   │   ├── T1_static/                  # T1 teszt (statikus)
│   │   ├── T2_moving/                  # T2 teszt (mozgó)
│   │   └── T3_stress/                  # T3 teszt (stressz)
│   │       ├── analyze_objects.py      # Rosbag elemző
│   │       └── rosbag/                 # Mentett rosbag-ek
│   │
│   └── screenshots/                    # Rendszer képernyőképek
│
├── docs/                               # Dokumentáció
│   ├── FUTTATAS_UTMUTATO.md           # Futtatási útmutató
│   └── README_TESZTELESI_TERV.md      # Tesztelési terv
│
├── config/                             # Globális konfigurációk
├── build/                              # Build kimenetek (generált)
├── install/                            # Telepített fájlok (generált)
└── log/                                # Build logok (generált)
```

### Főbb Fájlok Magyarázata

| Fájl | Leírás | Módosítható? |
|------|--------|--------------|
| `lidar_filter_node.py` | Fő objektum detektáló node | ✅ Igen |
| `*.launch.py` | Launch fájlok (rendszer indítás) | ✅ Igen |
| `*.rviz` | RViz vizualizációs konfigurációk | ✅ Igen |
| `setup.py` | Python csomag telepítési konfig | ⚠️ Óvatosan |
| `package.xml` | ROS2 csomag metaadatok | ⚠️ Óvatosan |
| `visualize_metrics.py` | Teszt eredmények vizualizációja | ✅ Igen |
| `build/`, `install/`, `log/` | Generált fájlok | ❌ Ne módosítsd |

---

## ⚙️ Rendszer Működése

### Architektúra Áttekintés

```
┌─────────────┐
│   Gazebo    │  (Szimulátor)
│ TurtleBot3  │
└──────┬──────┘
       │ /scan (LaserScan)
       ▼
┌─────────────────────┐
│ lidar_filter_node   │
│                     │
│ 1. Szűrés           │  (min/max range)
│ 2. Pol→Cart transz │  (koordináta konverzió)
│ 3. Clustering       │  (távolság alapú)
│ 4. Centroid számít │  (objektum pozíció)
└──────┬──────────────┘
       │
       ├──► /filtered_scan (LaserScan)
       ├──► /objects (PoseArray)
       └──► /object_markers (MarkerArray)
              │
              ▼
       ┌──────────┐
       │  RViz2   │  (Vizualizáció)
       └──────────┘
```

### Node Input/Output

**Input Topic-ok:**
- `/scan` (sensor_msgs/LaserScan) - LIDAR nyers adatok

**Output Topic-ok:**
- `/filtered_scan` (sensor_msgs/LaserScan) - Szűrt LIDAR adatok
- `/objects` (geometry_msgs/PoseArray) - Detektált objektumok pozíciói
- `/object_markers` (visualization_msgs/MarkerArray) - RViz markerek

**Paraméterek:**
- `min_range`: Minimum érvényes távolság (default: 0.1m)
- `max_range`: Maximum érvényes távolság (default: 10.0m)
- `min_cluster_size`: Min pontok száma egy klaszterben (default: 3)
- `cluster_threshold`: Max távolság pontok között klaszterben (default: 0.2m)

### Objektum Detektálási Algoritmus

A `lidar_filter_node.py` az alábbi lépéseket hajtja végre:

1. **LIDAR Szűrés:**
   - Érvénytelen távolságok kiszűrése (< min_range vagy > max_range)
   - Inf értékek használata érvénytelen mérésekhez

2. **Koordináta Transzformáció:**
   - Polár (r, θ) → Descartes (x, y) konverzió
   - x = r × cos(θ), y = r × sin(θ)

3. **Clustering:**
   - Egyszerű távolság alapú algoritmus (DBSCAN-szerű)
   - Közeli pontok (<0.2m) egy klaszterbe kerülnek
   - Minimum 3 pont szükséges egy érvényes klaszterhez

4. **Centroid Számítás:**
   - Klaszter átlagos pozíciója = objektum középpontja
   - PoseArray üzenetként publikálás

5. **Vizualizáció:**
   - Piros hengerek (markerek) az objektumok helyén
   - 1 sec élettartam (automatikus törlődés)

---

## 🚀 Futtatási Útmutatók

### Gyors Indítás (Ajánlott)

**Teljes rendszer 1 paranccsal:**

```bash
cd ~/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter complete_system.launch.py
```

Ez elindítja:
- ✅ Gazebo szimulátort + TurtleBot3
- ✅ LIDAR filter node-ot
- ✅ RViz2-t előre konfigurálva

### Optimalizált Verzió (Alacsonyabb CPU)

```bash
ros2 launch lidar_filter optimized_system.launch.py
```

Vagy headless (GUI nélkül):
```bash
ros2 launch lidar_filter optimized_system.launch.py gui:=false
```

### Részletes Indítás (Debug Célra)

**3 külön terminálban:**

**Terminal 1 - Gazebo:**
```bash
cd ~/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 - LIDAR Filter Node:**
```bash
cd ~/project_mgm
source install/setup.bash
ros2 run lidar_filter lidar_filter_node
```

**Terminal 3 - RViz2:**
```bash
cd ~/project_mgm
source install/setup.bash
ros2 run rviz2 rviz2 -d install/lidar_filter/share/lidar_filter/config/lidar_filter_rviz.rviz
```

### Robot Mozgatása (Opcionális)

**Terminal 4 - Teleop:**
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

Vezérlés: W/A/S/D/X (előre/balra/stop/jobbra/hátra)

---

## 🛠️ Fejlesztési Workflow

### 1. Kód Módosítása

Ha módosítod a `lidar_filter_node.py` fájlt:

```bash
# Szerkesztés
nano src/mgm_gyak/lidar_filter/lidar_filter/lidar_filter_node.py

# VAGY VS Code-dal:
code src/mgm_gyak/lidar_filter/lidar_filter/lidar_filter_node.py
```

### 2. Build és Teszt

```bash
# Build (csak a módosított csomag)
cd ~/project_mgm
colcon build --packages-select lidar_filter --symlink-install

# Source
source install/setup.bash

# Teszt
ros2 launch lidar_filter complete_system.launch.py
```

> **Tipp:** A `--symlink-install` flag miatt Python kód módosításokhoz NEM kell újra build-elni! Csak újraindítani a node-ot.

### 3. Launch Fájl Módosítása

Ha módosítod a launch fájlt (pl. új paraméterek):

```bash
# Szerkesztés
nano src/mgm_gyak/lidar_filter/launch/complete_system.launch.py

# Újra build KELL (launch fájlok másolása)
colcon build --packages-select lidar_filter

# Source és teszt
source install/setup.bash
ros2 launch lidar_filter complete_system.launch.py
```

### 4. Új Python Szkript Hozzáadása

Ha új Python fájlt adsz hozzá (pl. új node):

```bash
# Fájl létrehozása
touch src/mgm_gyak/lidar_filter/lidar_filter/my_new_node.py

# Szerkesztés
nano src/mgm_gyak/lidar_filter/lidar_filter/my_new_node.py

# setup.py frissítése
nano src/mgm_gyak/lidar_filter/setup.py

# Hozzáadni az entry_points-hoz:
entry_points={
    'console_scripts': [
        'lidar_filter_node = lidar_filter.lidar_filter_node:main',
        'my_new_node = lidar_filter.my_new_node:main',  # ÚJ SOR
    ],
},

# Build
colcon build --packages-select lidar_filter

# Futtatás
source install/setup.bash
ros2 run lidar_filter my_new_node
```

### 5. Git Workflow

```bash
# Új branch létrehozása
git checkout -b feature/my-new-feature

# Módosítások hozzáadása
git add .
git commit -m "Add: új funkció leírása"

# Push távoli repo-ba
git push origin feature/my-new-feature

# Pull request létrehozása GitHub-on
```

---

## 🧪 Tesztelés

### ROS2 Topic Ellenőrzés

```bash
# Összes topic listázása
ros2 topic list

# Topic típus ellenőrzése
ros2 topic type /scan
ros2 topic type /objects

# Topic adatok megjelenítése (valós időben)
ros2 topic echo /scan
ros2 topic echo /objects

# Topic frekvencia mérése
ros2 topic hz /scan
ros2 topic hz /objects

# Topic bandwidth
ros2 topic bw /scan
```

### Node Információk

```bash
# Futó node-ok
ros2 node list

# Node információk
ros2 node info /lidar_filter_node

# Node paraméterek
ros2 param list /lidar_filter_node
ros2 param get /lidar_filter_node min_range

# Paraméter módosítása futás közben
ros2 param set /lidar_filter_node cluster_threshold 0.3
```

### Rosbag Felvétel és Lejátszás

**Felvétel:**
```bash
# Minden topic felvétele
ros2 bag record -a -o my_test_recording

# Csak specifikus topic-ok
ros2 bag record /scan /objects /tf -o my_test_recording
```

**Lejátszás:**
```bash
# Bag info
ros2 bag info my_test_recording

# Lejátszás
ros2 bag play my_test_recording

# Lejátszás félig sebességgel
ros2 bag play my_test_recording -r 0.5
```

**Elemzés Python-nal:**
```bash
cd ~/project_mgm/tests/test_results/T3_stress

# Terminal 1: Bag play
ros2 bag play path/to/rosbag

# Terminal 2: Elemző script
python3 analyze_objects.py

# Ctrl+C a statisztikákért
```

### Vizualizációs Teszt

```bash
cd ~/project_mgm/tests/test_results
python3 visualize_metrics.py

# Kimenetek: visualizations/ könyvtárban
ls visualizations/*.pdf
ls visualizations/*.png
```

### RViz Graph Vizualizáció

```bash
# Node gráf megjelenítése
ros2 run rqt_graph rqt_graph

# TF tree
ros2 run rqt_tf_tree rqt_tf_tree
```

---

## 🔍 Hibaelhárítás

### Gyakori Problémák és Megoldások

#### 1. "Package 'lidar_filter' not found"

**Probléma:** A workspace nincs source-olva.

**Megoldás:**
```bash
cd ~/project_mgm
source install/setup.bash
```

#### 2. "No executable found"

**Probléma:** A csomag nincs build-elve vagy a setup.py hibás.

**Megoldás:**
```bash
colcon build --packages-select lidar_filter --symlink-install
source install/setup.bash
```

#### 3. Gazebo nem indul / fekete képernyő

**Probléma:** GPU driver vagy WSL2 X11 probléma.

**WSL2 Megoldás:**
```bash
# .bashrc-be:
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=0

# Windows-on: VcXsrv vagy X410 futtatása
```

**Native Linux Megoldás:**
```bash
# Nvidia driver frissítés
sudo ubuntu-drivers autoinstall
```

#### 4. "Could not contact Gazebo master"

**Probléma:** Gazebo még nem indult el teljesen.

**Megoldás:**
- Várj 10-15 másodpercet a Gazebo teljes elindulásáig
- VAGY terminálban indítsd először a Gazebo-t külön

#### 5. RViz "Fixed Frame does not exist"

**Probléma:** TF frame nem található.

**Megoldás:**
```bash
# Ellenőrizd a TF-eket
ros2 run tf2_ros tf2_echo odom base_scan

# RViz-ben állítsd át a Fixed Frame-et:
# Global Options > Fixed Frame > "odom" vagy "base_link"
```

#### 6. Python import error (numpy, matplotlib)

**Probléma:** Conda/virtualenv interferál a ROS2-vel.

**Megoldás:**
```bash
# Conda deaktiválása
conda deactivate

# VAGY .bashrc-ből kivegye a conda init részt
# VAGY rendszer Python használata:
sudo apt install python3-numpy python3-matplotlib
```

#### 7. Build hiba: "Multiple packages found"

**Probléma:** Duplikált csomagok (build/ és src/ könyvtárakban).

**Megoldás:**
```bash
# Tisztítás
cd ~/project_mgm
rm -rf build/ install/ log/

# Újra build
colcon build --symlink-install
```

#### 8. Alacsony scan rate / Lassú működés

**Probléma:** Túl sok CPU terhelés (főleg WSL2-n).

**Megoldás:**
```bash
# Optimalizált verzió használata
ros2 launch lidar_filter optimized_system.launch.py

# VAGY headless Gazebo
ros2 launch lidar_filter optimized_system.launch.py gui:=false
```

### Debug Módok

**Részletes logolás:**
```bash
ros2 launch lidar_filter complete_system.launch.py --log-level debug
```

**Node külön-külön indítása:**
- Könnyebb hibakeresés
- Lásd: [Részletes Indítás](#részletes-indítás-debug-célra)

**Python debugger (pdb):**
```python
# lidar_filter_node.py-ban:
import pdb; pdb.set_trace()

# Futtatás terminálból
ros2 run lidar_filter lidar_filter_node
```

---

## 📝 Hasznos Parancsok

### ROS2 Általános

```bash
# Környezet info
printenv | grep ROS

# ROS2 verzió
ros2 --version

# Installed packages
ros2 pkg list

# Package info
ros2 pkg prefix lidar_filter
```

### Colcon Build

```bash
# Teljes workspace build
colcon build

# Csak egy csomag
colcon build --packages-select lidar_filter

# Symlink install (Python módosításokhoz)
colcon build --symlink-install

# Tisztítás és újra build
rm -rf build/ install/ log/
colcon build
```

### Dependency Check

```bash
# Hiányzó függőségek telepítése
rosdep install --from-paths src --ignore-src -r -y

# Package.xml ellenőrzése
ros2 pkg xml lidar_filter
```

### Performance Monitoring

```bash
# CPU/RAM használat
htop

# ROS2 node resource usage
ros2 node info /lidar_filter_node

# Topic bandwidth
ros2 topic bw /scan

# Latency mérés
ros2 topic delay /scan
```

### Dokumentáció Generálás

```bash
# Python docstring-ek exportálása
pydoc3 lidar_filter.lidar_filter_node > node_docs.txt

# Package dokumentáció
rosdoc2 build --package-path src/mgm_gyak/lidar_filter
```

---

## 🤝 Hozzájárulás (Contributing)

Ha szeretnél hozzájárulni a projekthez:

1. **Fork-old** a repót GitHub-on
2. **Klónozd** a saját fork-odat
3. **Hozz létre** egy új branch-et (`git checkout -b feature/amazing-feature`)
4. **Commit-old** a módosításokat (`git commit -m 'Add: amazing feature'`)
5. **Push-old** a branch-et (`git push origin feature/amazing-feature`)
6. **Nyiss** egy Pull Request-et

### Kód Stílus

- **Python:** PEP 8 szabványok
- **Kommentek:** Magyar vagy angol (konzisztens legyen)
- **Docstring-ek:** Google style vagy NumPy style
- **Launch fájlok:** Kommentekkel ellátva
- **Commit üzenetek:** Értelmes, leíró üzenetek

### Tesztelés Követelmények

Minden új funkció esetén:
- ✅ Unit teszt (ha lehetséges)
- ✅ Integrációs teszt (rosbag vagy szimuláció)
- ✅ Dokumentáció frissítése
- ✅ Működik Jazzy + Humble verziókon

---

## 📚 További Források

### Hivatalos Dokumentációk

- **ROS2 Jazzy:** https://docs.ros.org/en/jazzy/
- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **TurtleBot3:** https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- **Gazebo:** https://gazebosim.org/docs

### Tutorial-ok

- **ROS2 alapok:** https://docs.ros.org/en/jazzy/Tutorials.html
- **Python ROS2:** https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- **Launch fájlok:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html

### Projekt Specifikus Dokumentáció

- **[README.md](README.md)** - Projekt áttekintés
- **[PROJEKT_STATUS.md](PROJEKT_STATUS.md)** - Jelenlegi állapot
- **[docs/FUTTATAS_UTMUTATO.md](docs/FUTTATAS_UTMUTATO.md)** - Részletes futtatási útmutató
- **[tests/test_results/TESZT_OSSZEFOGLALO.md](tests/test_results/TESZT_OSSZEFOGLALO.md)** - Teszt eredmények

---

## 📞 Kapcsolat és Támogatás

**Eredeti fejlesztő:** Mitrenga Márk  
**Repository:** https://github.com/mitrengamark/project_mgm

**Kérdések, problémák esetén:**
1. Nézd át ezt az útmutatót
2. Ellenőrizd a [Hibaelhárítás](#hibaelhárítás) szekciót
3. Keress hasonló issue-kat a GitHub-on
4. Nyiss egy új issue-t részletes leírással

---

## 🔄 Verziókezelés

**Jelenlegi verzió:** 1.0.0  
**Utolsó frissítés:** 2025-11-25

### Változások követése

```bash
# Git log
git log --oneline --graph --all

# Verzió különbségek
git diff main..your-branch

# Tag-ek (release-ek)
git tag -l
```

---

**Jó kódolást!** 🚀

Ha bármilyen kérdésed van, ne habozz issue-t nyitni vagy Pull Request-et küldeni!
