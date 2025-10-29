# T3 Teszt Jegyzetek - Stresszteszt

**Dátum:** 2025-10-29  
**Tesztelő:** Mitrenga Márk  
**Verzió:** Stresszteszt (több objektum)

---

## Teszt Konfiguráció

- **Robot model:** TurtleBot3 Waffle
- **Világ:** turtlebot3_world.world
- **Node:** lidar_filter_node
- **Launch:** optimized_system.launch.py
- **Gazebo mode:** [ ] GUI  [ ] Headless (gui:=false)
- **Paraméterek:**
  - min_range: 0.1m
  - max_range: 10.0m
  - min_cluster_size: 3
  - cluster_threshold: 0.2

---

## Spawning Objektumok

### Létrehozott objektumok:
| ID | Típus | Pozíció (x, y, z) | Jegyzet |
|----|-------|-------------------|---------|
| 1  | Box/Cylinder | (__, __, __) | |
| 2  | Box/Cylinder | (__, __, __) | |
| 3  | Box/Cylinder | (__, __, __) | |
| 4  | Box/Cylinder | (__, __, __) | |
| 5  | Box/Cylinder | (__, __, __) | |
| 6  | Box/Cylinder | (__, __, __) | |
| 7  | Box/Cylinder | (__, __, __) | |

**Összes objektum:** _____ db

**Spawning módszer:**
- [ ] Gazebo GUI Insert tab
- [ ] `ros2 run gazebo_ros spawn_entity.py` parancsok
- [ ] Előre definiált world fájl

---

## Megfigyelések

### Inicializáció
- [ ] Gazebo elindult sikeresen
- [ ] Objektumok spawning sikeres
- [ ] LIDAR Filter Node inicializálva
- [ ] RViz megnyílt optimalizált config-gal
- [ ] Scan vizualizáció látható (PIROS)
- [ ] Filtered scan látható (ZÖLD)
- [ ] Markerek megjelennek (CYLINDEREK)

### Detektálás (statikus robot)
- [ ] Minden objektum látható a scan-ben
- [ ] Objektumok detektálása sikeres
- [ ] Markerek minden objektumnál megjelennek
- [ ] Detektálás stabil (nem villog)
- [ ] Nincs false positive

### Detektálás (mozgó robot - opcionális)
- [ ] Objektumok követése mozgás közben
- [ ] Markerek frissülnek a pozíció változásával
- [ ] Nincs objektum elvesztés
- [ ] Detektálási siker stabil marad

### Teljesítmény
- Átlagos FPS: _____
- Detektált objektumok száma/scan: _____
- CPU használat: _____% 
- Memory használat: _____
- Gazebo RTF (Real Time Factor): _____

### RViz Megjelenés
- [ ] Tiszta vizualizáció
- [ ] Minden marker látható
- [ ] Scan és filtered scan jól elkülöníthető
- [ ] Nincs túlterhelés/lag

---

## Rosbag Ellenőrzés (teszt után)

```bash
ros2 bag info test_run_stress
```

**Teszt időtartam:** 61.08 sec

**Topic-ok üzenetszáma:**
- /scan: 44
- /filtered_scan: 43
- /objects: 42
- /object_markers: 42
- /odom: 218
- /tf: 8077
- /cmd_vel: 0 (statikus robot)

**Bag méret:** 2.7 MiB

**Scan rate:** ~0.72 Hz (44 scan / 61 sec)

---

## Összehasonlítás T2 vs T3

| Metrika | T2 (mozgó, 1-3 obj) | T3 (stresszteszt, 0 obj) | Változás |
|---------|---------------------|---------------------------|----------|
| **Objektumok száma** | 1-3 | 0 (spawn sikertelen) | N/A |
| **Teszt időtartam** | 276.7 sec | 61.08 sec | -78% |
| **Bag méret** | 15.2 MiB | 2.7 MiB | -82% |
| **Scan rate** | 0.86 Hz | 0.72 Hz | -16% |
| **Detektálási siker** | 99.6% | 100%* | *0 obj esetén |
| **Det. obj/scan** | ~1-3 | 0 | N/A |
| **CPU használat** | ~100% | Valószínűleg alacsonyabb | N/A |
| **Memory** | - | Nem mérve | - |

**Megjegyzés:** T3 teszt nem valós stresszteszt volt, mivel a spawning sikertelen volt.

---

## Problémák / Hibák

A spawn_objects.sh lefutott de nem láttam semmit gazebon vagy rvizen.
Valamint annak nem kéne végig futnia? Mert lefutott de aztán mire elindítottam a rosbag-be való felvételt már nem futott a kód


### Spawning problémák:
Nem láttam semmit lespawnolni!

### Detektálási problémák:


### Teljesítmény problémák:


### Egyéb:
Elküldöm a 2. -spawn terminál ablakot.
mark@DESKTOP-4ADON8D:~/codes/mgm/project_mgm/tests/test_results/T2_moving/rosbag $ cd /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress
parse_git_branch: command not found
mark@DESKTOP-4ADON8D:~/codes/mgm/project_mgm/tests/test_results/T3_stress $ ./spawn_objects.sh 7
============================================
  T3 Stresszteszt - Objektum Spawning
============================================

Spawning 7 objektum a Gazebo világban...

Várakozás 5 másodperc a Gazebo inicializálására...
Spawning objektumok...

[1/7] Box spawning (2.0, 1.5, 0.5)...
Package 'gazebo_ros' not found
[2/7] Cylinder spawning (2.0, -1.5, 0.5)...
Package 'gazebo_ros' not found
[3/7] Box spawning (-2.0, 1.5, 0.5)...
Package 'gazebo_ros' not found
[4/7] Cylinder spawning (-2.0, -1.5, 0.5)...
Package 'gazebo_ros' not found
[5/7] Box spawning (3.0, 0.0, 0.5)...
Package 'gazebo_ros' not found
[6/7] Cylinder spawning (-3.0, 0.0, 0.5)...
Package 'gazebo_ros' not found
[7/7] Box spawning (2.5, 0.0, 0.5)...
Package 'gazebo_ros' not found

Várakozás az objektumok spawning-jának befejeződésére...

============================================
  Spawning befejezve!
  Létrehozott objektumok: 7
============================================

Ellenőrizd a Gazebo-ban és az RViz-ben!
Ha minden rendben, indítsd a rosbag rögzítést.

Spawning objektumok ellenőrzése:
The passed service type is invalid
parse_git_branch: command not found
mark@DESKTOP-4ADON8D:~/codes/mgm/project_mgm/tests/test_results/T3_stress $ source /home/mark/codes/mgm/project_mgm/install/setup.bash
parse_git_branch: command not found
mark@DESKTOP-4ADON8D:~/codes/mgm/project_mgm/tests/test_results/T3_stress $ ros2 run gazebo_ros spawn_entity.py -entity box1 -database unit_box -x 2.0 -y 1.5 -z 0.5
Package 'gazebo_ros' not found
parse_git_branch: command not found
mark@DESKTOP-4ADON8D:~/codes/mgm/project_mgm/tests/test_results/T3_stress $ ^C
parse_git_branch: command not found

---

## Következtetések

### Pozitívumok:
Elindult a rendszer és mentette is azadatokat.

### Negatívumok:
Nem volt szemmel látható spawning és a ros bag recordnál már nem is futott a program.

### Rendszer Stabilitása:
- [ ] Nincs crash
- [ ] Nincs node restart
- [ ] Nincs ERROR üzenet
- [ ] Stabil működés végig

### Skálázhatóság:
- Hány objektumig működik jól a rendszer? _____
- Melyik a bottleneck? (CPU/Memory/Scan rate/Clustering) _____

### Javaslatok:
A random objektum spawning legyen folyamatos amíg le nem állítom. Példál egy objektum pár másodpercig látható aztán eltűnik és lesz egy újabb mindaddig amíg le nem állítom.^C

---

## Következő Lépések

- [ ] Rosbag → CSV export
- [ ] Grafikonok készítése (objektumszám vs teljesítmény)
- [ ] Összehasonlító elemzés (T1 vs T2 vs T3)
- [ ] Screenshot-ok (RViz, Gazebo, rqt_graph)
- [ ] Dokumentáció frissítése

---

**Teszt végrehajtva:** _____ (dátum, idő)  
**Összesített értékelés:** ⭐ (1-5)
