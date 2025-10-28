# Tesztesetek - LIDAR Objektum Detektálás

**Projekt:** MGM - LIDAR alapú objektum detektálás és követés  
**Tesztelő:** Mitrenga Márk  
**Dátum:** 2025. október 28.

---

## 🎯 Tesztelési célok

1. LIDAR szűrés működésének validálása
2. Objektum detektálás pontosságának mérése
3. Rendszer stabilitásának ellenőrzése
4. Performance metrikák gyűjtése

---

## 📋 Tesztszcenáriók

### T1: Statikus környezet - Alap detektálás

**Cél:** Statikus objektumok (falak, akadályok) detektálása  
**Környezet:** TurtleBot3 World (Gazebo)  
**Előfeltétel:** Robot áll, nem mozog

**Lépések:**
1. Gazebo + lidar_filter_node + RViz indítása
2. 10 másodperc adatrögzítés állva
3. Topic frekvencia mérése
4. Detektált objektumok számának mérése

**Elvárt eredmény:**
- `/scan` topic: ~10 Hz
- `/filtered_scan` publikálva
- Statikus objektumok konzisztensen detektálva
- CPU használat < 50%

**Rosbag:** `test_run1_static.bag`

---

### T2: Mozgó robot - Dinamikus detektálás

**Cél:** Robot mozgás közben objektumok folyamatos detektálása  
**Környezet:** TurtleBot3 World (Gazebo)  
**Előfeltétel:** Teleop irányítás aktív

**Lépések:**
1. Robot mozgatása előre (w billentyű)
2. Robot forgatása (a/d billentyűk)
3. 30 másodperc adatrögzítés mozgás közben
4. Objektum követés megfigyelése

**Elvárt eredmény:**
- Folyamatos objektum detektálás mozgás közben
- Objektumok pozíciójának frissítése
- Nincs jelentős késleltetés
- Stabil működés

**Rosbag:** `test_run2_moving.bag`

---

### T3: Komplex manőverek - Stresszteszt

**Cél:** Gyors mozgások és irányváltások kezelése  
**Környezet:** TurtleBot3 World (Gazebo)  
**Előfeltétel:** Teleop irányítás aktív

**Lépések:**
1. Gyors előre-hátra mozgás (w/x)
2. Folyamatos forgás (a/d)
3. Hirtelen megállások (s)
4. 20 másodperc vegyes manőverek
5. Performance monitoring (htop)

**Elvárt eredmény:**
- Node nem crashel
- Objektum detektálás konzisztens marad
- CPU spike-ok kezelve
- Memory leak nincs

**Rosbag:** `test_run3_stress.bag`

---

## 📊 Mérési metrikák

### 1. Topic frekvencia
```bash
ros2 topic hz /scan
ros2 topic hz /filtered_scan
ros2 topic hz /objects
```

**Cél:** 10 Hz ± 1 Hz

---

### 2. Késleltetés (Latency)
```bash
ros2 topic echo /scan --field header.stamp
ros2 topic echo /objects --field header.stamp
```

**Számítás:** Timestamp különbség  
**Cél:** < 100 ms

---

### 3. CPU használat
```bash
htop  # lidar_filter_node folyamat
```

**Cél:** < 50% átlagosan, < 80% peak

---

### 4. Objektum detektálás pontossága

**Módszer:** Manuális számlálás vs. `/objects` topic  
**Számítás:**
- Precision = TP / (TP + FP)
- Recall = TP / (TP + FN)

**Cél:** 
- Precision > 90%
- Recall > 85%

---

### 5. Pozíciós hiba

**Módszer:** Ground truth (Gazebo) vs. detektált pozíció  
**Cél:** < 0.1 m átlagos hiba

---

## 🗂️ Tesztadatok struktúra

```
tests/
├── test_cases.md                    # Ez a fájl
├── test_results/
│   ├── T1_static/
│   │   ├── rosbag/
│   │   │   └── test_run1_static/
│   │   ├── screenshots/
│   │   │   ├── rviz_t1.png
│   │   │   ├── gazebo_t1.png
│   │   │   └── rqt_graph_t1.png
│   │   ├── metrics_t1.csv
│   │   └── notes_t1.txt
│   ├── T2_moving/
│   │   ├── rosbag/
│   │   │   └── test_run2_moving/
│   │   ├── screenshots/
│   │   ├── metrics_t2.csv
│   │   └── notes_t2.txt
│   └── T3_stress/
│       ├── rosbag/
│       │   └── test_run3_stress/
│       ├── screenshots/
│       ├── metrics_t3.csv
│       └── notes_t3.txt
└── summary_report.md
```

---

## ✅ Teszt checklist

### T1: Statikus környezet
- [ ] Rendszer indítva
- [ ] Rosbag rögzítés indítva
- [ ] 10 sec várakozás
- [ ] Screenshot - RViz
- [ ] Screenshot - Gazebo
- [ ] Screenshot - rqt_graph
- [ ] Topic frekvencia mérés
- [ ] CPU mérés
- [ ] Rosbag mentve
- [ ] Jegyzet készítése

### T2: Mozgó robot
- [ ] Rendszer indítva
- [ ] Teleop indítva
- [ ] Rosbag rögzítés indítva
- [ ] Robot mozgatása (30 sec)
- [ ] Screenshot - RViz (mozgás közben)
- [ ] Topic frekvencia mérés
- [ ] Rosbag mentve
- [ ] Jegyzet készítése

### T3: Stresszteszt
- [ ] Rendszer indítva
- [ ] Teleop indítva
- [ ] htop monitoring
- [ ] Rosbag rögzítés indítva
- [ ] Komplex manőverek (20 sec)
- [ ] CPU peak mérés
- [ ] Memory mérés
- [ ] Rosbag mentve
- [ ] Jegyzet készítése

---

## 📝 Jegyzet sablon

```markdown
# Teszt jegyzet - [T1/T2/T3]

**Dátum:** 2025.10.28  
**Időpont:** HH:MM  
**Tesztelő:** Mitrenga Márk

## Körülmények
- ROS_DISTRO: jazzy
- Python verzió: 3.12
- Gazebo verzió: ...

## Megfigyelések
- [Észrevétel 1]
- [Észrevétel 2]

## Mért értékek
- Scan Hz: ...
- Objects Hz: ...
- CPU: ...

## Problémák
- [Ha voltak]

## Következtetés
- [Összegzés]
```

---

**Következő lépés:** T1 teszteset futtatása és rosbag rögzítése
