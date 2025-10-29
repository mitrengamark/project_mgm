# T2 Teszt - Mozgó Robot

**Dátum:** 2025-10-29  
**Teszt típus:** Mozgó robot objektum detektálás

## Teszt leírás

A T2 teszt célja, hogy **mozgó robot** közben tesztelje az objektum detektálási rendszert. A robot köröket írva le fog mozogni a világban, miközben a LIDAR filter node detektálja az objektumokat.

## Teszt lépések

### 1. Rendszer indítása

Terminál 1 - Teljes rendszer (Gazebo + lidar_filter + RViz):
```bash
cd /home/mark/codes/mgm/project_mgm
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch lidar_filter complete_system.launch.py
```

### 2. Rosbag rögzítés indítása

Terminál 2 - Rosbag record:
```bash
cd /home/mark/codes/mgm/project_mgm/tests/test_results/T2_moving/rosbag
source /home/mark/codes/mgm/project_mgm/install/setup.bash
ros2 bag record -o test_run_moving /scan /scan_filtered /detected_objects /markers /odom /tf /cmd_vel
```

### 3. Robot mozgatása

Terminál 3 - TurtleBot3 teleop:
```bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mozgási szekvencia:**
- 20 sec: Egyenes mozgás előre (w)
- 10 sec: Fordulás jobbra (d)
- 20 sec: Egyenes mozgás előre (w)
- 10 sec: Fordulás balra (a)
- Ismételd körpályán ~2 percig

### 4. Metrikák gyűjtése

A teszt alatt figyelendő:
- Detektált objektumok száma
- FPS (node kimenet)
- Látható markerek RVizben
- Robot odometry változása

### 5. Teszt leállítása

1. Terminál 3: Stop robot (Ctrl+C)
2. Terminál 2: Stop rosbag (Ctrl+C)
3. Terminál 1: Stop launch (Ctrl+C)

## Eredmények

### Rosbag fájl:
`tests/test_results/T2_moving/rosbag/test_run_moving/`

### Metrikák CSV:
`tests/test_results/T2_moving/metrics_t2.csv`

### Jegyzetek:
`tests/test_results/T2_moving/notes_t2.md`

## Screenshot-ok készítése

- Gazebo: Robot mozgás közben
- RViz: Detected markers + scan vizualizáció
- rqt_graph: Node kapcsolatok
