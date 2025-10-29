# T2 Teszt Jegyzetek - Optimalizált Verzió (v2)

**Dátum:** 2025-10-29  
**Tesztelő:** Mitrenga Márk  
**Verzió:** 2.0 (Optimalizált)

## Teszt konfiguráció

- **Robot model:** TurtleBot3 Waffle
- **Világ:** turtlebot3_world.world
- **Node:** lidar_filter_node
- **Launch:** optimized_system.launch.py
- **RViz config:** lidar_filter_optimized.rviz
- **Paraméterek:**
  - min_range: 0.1m
  - max_range: 10.0m
  - min_cluster_size: 3
  - cluster_threshold: 0.2

## Optimalizációk

### Javított beállítások:
- ✅ RViz: Csak 3 TF frame (odom, base_link, base_scan)
- ✅ Map display eltávolítva (warning megszűnés)
- ✅ Helyes topic nevek: /filtered_scan, /objects, /object_markers
- ✅ Optimalizált marker megjelenítés

## Megfigyelések

### Inicializáció
- [ ] Gazebo elindult sikeresen
- [ ] LIDAR Filter Node inicializálva
- [ ] RViz megnyílt optimalizált config-gal
- [ ] Scan vizualizáció látható (PIROS)
- [ ] Filtered scan látható (ZÖLD)
- [ ] Markerek megjelennek (CYLINDEREK)

### Mozgás közben
- [ ] Objektumok detektálása mozgás közben
- [ ] Markerek követik az objektumokat
- [ ] FPS stabil marad
- [ ] Nincs késleltetés
- [ ] Csak 3 TF frame látható

### Teljesítmény (v2)
- Átlagos FPS: _____
- Detektált objektumok száma: _____
- CPU használat: _____% (előző: 100%)
- Memory használat: _____
- Gazebo RTF (Real Time Factor): _____

### RViz megjelenés
- [ ] Tiszta, nem "ocsmány"
- [ ] Kevesebb koordináta rendszer
- [ ] Nincs Map warning
- [ ] Markerek jól láthatók

## Rosbag ellenőrzés (teszt után)

```bash
ros2 bag info test_run_moving_v2
```

**Topic-ok üzenetszáma:**
- /scan: _____
- /filtered_scan: _____
- /objects: _____
- /object_markers: _____
- /odom: _____
- /tf: _____
- /cmd_vel: _____

**Összehasonlítás v1-el:**
- v1: 0 üzenet /filtered_scan, /objects, /markers témákban ❌
- v2: _____ üzenet várható ✅

## Problémák / Hibák (v2)

_(Töltsd ki a teszt közben észlelt problémákkal - javultak-e?)_

## Összehasonlítás v1 vs v2

| Tulajdonság | v1 (eredeti) | v2 (optimalizált) |
|-------------|--------------|-------------------|
| CPU használat | 100% (8 core) | _____% |
| RViz TF frames | Összes (~12+) | 3 (odom, base_link, base_scan) |
| Map warning | Igen ⚠️ | Nem ✅ |
| Rosbag topic-ok | Hiányos ❌ | Teljes ✅ |
| Gazebo RTF | Lassú | _____ |

## Következtetések

_(Töltsd ki a teszt után - javult-e a helyzet?)_

### Pozitívumok:

### Negatívumok / További optimalizálási lehetőségek:

### Javaslatok T3 teszthez:
