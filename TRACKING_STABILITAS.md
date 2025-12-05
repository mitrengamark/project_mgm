# 🔧 Objektum ID Stabilitás - Javítási Dokumentáció

**Probléma:** Az objektumok gyorsan változó ID-kat kaptak (pl. OBJ_4 → OBJ_34), mely azt jelezte, hogy túl sok új objektum jött létre az ID-megőrzés helyett.

**Gyökeroka:** 
1. `max_distance` (0.6m) túl szigorú volt
2. A LIDAR mérési zaj + robot gyors mozgása miatt az objektumok pozíciói nagyobb mértékben változtak
3. Hiányzott az eltakarodott objektumok kezelése az új detektáláskor

---

## 🎯 Megoldás: Háromszintű Párosítási Logika

### 1. **Elsőszintű Párosítás - Hungarian Algorithm**
```
Régi objektumok ↔ Új detektálások (max_distance küszöb alapján)
```
- Ha távolság < 1.5m → PÁROSÍTÁS (ID megmarad)
- Ha távolság > 1.5m → NINCS PÁROSÍTÁS

### 2. **Másodszintű Párosítás - Eltakarodott Objektumok**
```
Nem párosított új objektum ↔ Eltakarodott (invisible) régi objektumok
```
- Ha új objektum közel van (< 1.5m) egy eltakarodott objektumhoz:
  - **Azt az eltakarodott objektumot reaktiváljuk**
  - Az eredeti ID-t KAPJA VISSZA ✅
  - `visible` flag = True

### 3. **Harmadszintű Párosítás - Új ID**
```
Ha nem párosított az objektum 1. vagy 2. szinten → ÚJ ID jön létre
```

---

## 📊 Konkrét Algoritmus

```python
for új_obj in nem_párosított_új_objektumok:
    found = False
    
    # Keresés eltakarodott objektumok között
    for régi_obj_id in régi_objektumok:
        if régi_obj_id nincs párosítva AND régi_obj[visible] == False:
            dist = távolság(új_obj, régi_obj[position])
            
            if dist < 1.5m:  # max_distance küszöb
                # REAKTIVÁCIÓ: adunk az eltakarodott objektumnak
                régi_obj[position] = új_obj
                régi_obj[visible] = True
                régi_obj[last_seen] = jelenlegi_idő
                found = True
                break
    
    if not found:
        # Valóban új objektum → új ID
        self.next_id += 1
```

---

## 🔧 Paramétermódosítások

| Paraméter | Régi érték | Új érték | Indoklás |
|-----------|------------|----------|----------|
| `max_distance` | 0.6m | 1.5m | Robot gyors mozgása, LIDAR szóródás |
| `timeout` | 2 sec | 5 sec | Eltakarodás > 2 sec lehet |

---

## ✅ Végeredmény

**Előtte (nem jó):**
```
OBJ_0 → OBJ_1 → OBJ_2 → OBJ_10 → OBJ_34 → OBJ_48
        (ID-k folyamatosan változnak!)
```

**Utána (jó):**
```
OBJ_0 → OBJ_0 → OBJ_0 → OBJ_0 → OBJ_0 → OBJ_0
        (ID megmarad, még takarás után is!)
```

---

## 🧪 Tesztelés

```bash
# Rendszer indítása
ros2 launch lidar_filter optimized_system.launch.py

# Külön terminálban: nyomkövetés debug
python3 test_tracking.py

# Harmadik terminálban: robot mozgatása
ros2 run turtlebot3_teleop teleop_keyboard
```

**Expectáció:**
- ✅ ID-k stabil maradnak
- ✅ Eltakarás után ugyanaz az ID
- ✅ Csak valóban új objektumok kapnak új ID-t

---

## 🐛 Finomhangolás (ha szükséges)

Ha még mindig túl sok új ID jön létre:
```python
# lidar_filter_node.py, 145. sor:
self.tracker = ObjectTracker(
    max_distance=2.0,  # Még nagyobb tolerancia
    timeout=10.0       # Még hosszabb timeout
)
```

Ha túl sok hamis párosítás (rossz objektumok összeolvadnak):
```python
# Csökkentsd a max_distance értéket
max_distance=1.2  # Visszatérés szigorúbb hozzárendelésre
```
