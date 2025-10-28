# Teszt jegyzet - T1 (Statikus környezet)

**Dátum:** 2025.10.28  
**Időpont:** 23:50  
**Tesztelő:** Mitrenga Márk  
**Teszt ID:** T1_static

---

## 🎯 Tesztcél
Statikus objektumok (falak, akadályok) detektálása állva

---

## ⚙️ Körülmények

### Rendszer
- **ROS_DISTRO:** jazzy
- **Python verzió:** 3.12  
- **Gazebo:** Sim 8.x
- **TurtleBot3 Model:** Waffle

### Konfiguráció
- **LIDAR filter paraméterek:**
  - min_range: 0.1 m
  - max_range: 10.0 m
  - min_cluster_size: 3
  - cluster_threshold: 0.2 m

---

## 📊 Tesztfolyamat

1. ✅ Rendszer indítva (Gazebo + lidar_filter_node + RViz2)
2. ✅ Robot statikus pozícióban
3. ✅ 10 másodperc rosbag rögzítés
4. ✅ Topic frekvencia mérés

**Rögzített topicok:**
- `/scan`
- `/filtered_scan`
- `/objects`
- `/object_markers`
- `/map`
- `/odom`
- `/tf`

---

## 📈 Mért értékek

### Topic frekvenciák
- **`/scan`**: ~0.9 Hz (alacsonyabb mint elvárt!)
  - Min: 0.129s
  - Max: 1.460s
  - Std dev: 0.34s
  - **PROBLÉMA:** Kellene ~10 Hz

### Megfigyelések
- ✅ `/filtered_scan` publikálva
- ✅ `/objects` publikálva  
- ✅ `/object_markers` vizualizáció működik
- ✅ Node nem crashelt
- ⚠️ Scan frekvencia alacsonyabb mint várt

---

## 🐛 Problémák

### 1. Alacsony LIDAR frekvencia
**Tünet:** `/scan` topic ~0.9 Hz helyett ~10 Hz  
**Lehetséges okok:**
- Gazebo szimuláció lassú (CPU)
- LIDAR szenzor konfiguráció
- ROS 2 QoS beállítások

**Megoldási javaslat:**
- Gazebo real-time factor ellenőrzése
- LIDAR update rate növelése a modellben

---

## ✅ Sikeres elemek

1. **Objektum detektálás működik** - Falak és akadályok detektálva
2. **Rosbag sikeresen rögzítve** - ~6 másodperc adat
3. **Node stabilan fut** - Nincs crash
4. **Vizualizáció OK** - RViz2-ben látható minden topic

---

## 📁 Rögzített fájlok

- **Rosbag:** `tests/test_results/T1_static/rosbag/test_run1_static/`
  - `metadata.yaml`
  - `test_run1_static_0.mcap`
- **Időtartam:** ~6 másodperc
- **Fájlméret:** ~több MB (ellenőrizendő)

---

## 📝 Következtetés

**Státusz:** ✅ Részben sikeres

**Pozitívumok:**
- Objektum detektálás alapvetően működik
- Rendszer stabil
- Rosbag rögzítés sikeres

**Javítandók:**
- LIDAR frekvencia növelése (Gazebo konfiguráció)
- Real-time performance optimalizálás

**Következő lépés:**
- T2 teszt (mozgó robot) vagy
- Gazebo LIDAR konfiguráció javítása

---

**Jegyzet készítette:** Mitrenga Márk  
**Verzió:** 1.0
