#!/bin/bash

# T3 Stresszteszt - Folyamatos Random Objektum Spawning
# Objektumok pár másodpercenként spawn-olnak és eltűnnek

echo "============================================"
echo "  T3 Folyamatos Objektum Spawning"
echo "============================================"
echo ""
echo "Objektumok folyamatosan spawn-olnak és tűnnek el."
echo "Nyomj Ctrl+C a leállításhoz!"
echo ""

# Ellenőrizzük, hogy fut-e a Gazebo (Gazebo Harmonic)
if ! pgrep -f "gz sim" > /dev/null; then
    echo "❌ HIBA: Gazebo nem fut!"
    echo "Indítsd el először: ros2 launch lidar_filter optimized_system.launch.py"
    exit 1
fi

echo "✅ Gazebo (Harmonic) fut, spawning indítása..."
echo ""

# Számláló az egyedi entity ID-khez
counter=1

# Objektum típusok
models=("unit_box" "unit_cylinder")

# Pozíciók a robot körül (2-4m távolság)
positions=(
    "2.0 1.5"
    "2.0 -1.5"
    "-2.0 1.5"
    "-2.0 -1.5"
    "3.0 0.0"
    "-3.0 0.0"
    "2.5 2.5"
    "-2.5 2.5"
    "2.5 -2.5"
    "-2.5 -2.5"
    "1.5 3.0"
    "-1.5 3.0"
)

# Végtelen ciklus - Ctrl+C-vel lehet leállítani
while true; do
    # Random objektum típus kiválasztása
    model_idx=$((RANDOM % ${#models[@]}))
    model=${models[$model_idx]}
    
    # Random pozíció kiválasztása
    pos_idx=$((RANDOM % ${#positions[@]}))
    pos=${positions[$pos_idx]}
    x=$(echo $pos | cut -d' ' -f1)
    y=$(echo $pos | cut -d' ' -f2)
    z=0.5
    
    # Objektum neve típus alapján
    if [ "$model" == "unit_box" ]; then
        type_name="Box"
    else
        type_name="Cyl"
    fi
    
    entity_name="stress_${type_name}_${counter}"
    
    # Spawning (Gazebo Harmonic SDF format)
    echo "[$(date +%H:%M:%S)] Spawning: $entity_name at ($x, $y, $z)"
    
    # SDF model különbözik box vs cylinder esetén
    if [ "$model" == "unit_box" ]; then
        sdf_geometry="<box><size>0.5 0.5 0.5</size></box>"
    else
        sdf_geometry="<cylinder><radius>0.25</radius><length>0.5</length></cylinder>"
    fi
    
    gz service -s /world/default/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf: '<model name=\"$entity_name\"><static>false</static><pose>$x $y $z 0 0 0</pose><link name=\"link\"><inertial><mass>1.0</mass></inertial><collision name=\"collision\"><geometry>$sdf_geometry</geometry></collision><visual name=\"visual\"><geometry>$sdf_geometry</geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model>'" \
        > /dev/null 2>&1 &
    
    spawn_pid=$!
    
    # Rövid várakozás hogy a spawn befejeződjön
    sleep 0.5
    
    # Objektum élettartama: 5-10 másodperc random
    lifetime=$((5 + RANDOM % 6))
    echo "   ↳ Élettartam: ${lifetime}s"
    
    # Várakozás
    sleep $lifetime
    
    # Objektum törlése (Gazebo Harmonic)
    echo "   ↳ Törlés: $entity_name"
    gz service -s /world/default/remove \
        --reqtype gz.msgs.Entity \
        --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req "name: '$entity_name', type: MODEL" \
        > /dev/null 2>&1 &
    
    # Számláló növelése
    counter=$((counter + 1))
    
    # Kis szünet a következő spawn előtt (1-3 másodperc)
    pause=$((1 + RANDOM % 3))
    sleep $pause
    
    echo ""
done
