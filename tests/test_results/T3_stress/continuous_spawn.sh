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

# Ellenőrizzük, hogy fut-e a Gazebo
if ! ros2 service list | grep -q "/gazebo"; then
    echo "❌ HIBA: Gazebo nem fut!"
    echo "Indítsd el először: ros2 launch lidar_filter optimized_system.launch.py"
    exit 1
fi

echo "✅ Gazebo fut, spawning indítása..."
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
    
    # Spawning
    echo "[$(date +%H:%M:%S)] Spawning: $entity_name at ($x, $y, $z)"
    ros2 run gazebo_ros spawn_entity.py \
        -entity "$entity_name" \
        -database "$model" \
        -x "$x" -y "$y" -z "$z" \
        > /dev/null 2>&1 &
    
    spawn_pid=$!
    
    # Rövid várakozás hogy a spawn befejeződjön
    sleep 0.5
    
    # Objektum élettartama: 5-10 másodperc random
    lifetime=$((5 + RANDOM % 6))
    echo "   ↳ Élettartam: ${lifetime}s"
    
    # Várakozás
    sleep $lifetime
    
    # Objektum törlése
    echo "   ↳ Törlés: $entity_name"
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: '$entity_name'}" > /dev/null 2>&1 &
    
    # Számláló növelése
    counter=$((counter + 1))
    
    # Kis szünet a következő spawn előtt (1-3 másodperc)
    pause=$((1 + RANDOM % 3))
    sleep $pause
    
    echo ""
done
