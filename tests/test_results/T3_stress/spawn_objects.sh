#!/bin/bash

# T3 Teszt - Objektum Spawning Script (Batch Mode)
# Használat: ./spawn_objects.sh [objektumok_száma]
# Példa: ./spawn_objects.sh 7
# Megjegyzés: Folyamatos spawning-hoz használd a continuous_spawn.sh-t!

echo "============================================"
echo "  T3 Stresszteszt - Objektum Spawning"
echo "============================================"
echo ""

# Objektumok száma (alapértelmezett: 7)
NUM_OBJECTS=${1:-7}

echo "Spawning $NUM_OBJECTS objektum a Gazebo világban..."
echo ""

# ROS 2 setup - KRITIKUS!
source /opt/ros/jazzy/setup.bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

# Ellenőrzés: Fut-e a Gazebo?
if ! ros2 service list | grep -q "/gazebo"; then
    echo "❌ HIBA: Gazebo nem fut!"
    echo "Indítsd el először: ros2 launch lidar_filter optimized_system.launch.py"
    exit 1
fi

echo "✅ Gazebo fut, folytatás..."
echo ""

# Várakozás a Gazebo inicializálására
echo "Várakozás 5 másodperc a Gazebo inicializálására..."
sleep 5

# Objektumok spawning-ja körkörösen a robot körül
# Robot pozíció: ~(0, 0, 0)
# Objektumok: 2-4 méter távolságban, egyenletes eloszlással

echo "Spawning objektumok..."
echo ""

# Objektum 1: Box előre-jobbra
echo "[1/$NUM_OBJECTS] Box spawning (2.0, 1.5, 0.5)..."
ros2 run gazebo_ros spawn_entity.py -entity stress_box1 -database unit_box -x 2.0 -y 1.5 -z 0.5 &
sleep 2

# Objektum 2: Cylinder előre-balra
if [ $NUM_OBJECTS -ge 2 ]; then
    echo "[2/$NUM_OBJECTS] Cylinder spawning (2.0, -1.5, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_cyl1 -database unit_cylinder -x 2.0 -y -1.5 -z 0.5 &
    sleep 2
fi

# Objektum 3: Box hátra-jobbra
if [ $NUM_OBJECTS -ge 3 ]; then
    echo "[3/$NUM_OBJECTS] Box spawning (-2.0, 1.5, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_box2 -database unit_box -x -2.0 -y 1.5 -z 0.5 &
    sleep 2
fi

# Objektum 4: Cylinder hátra-balra
if [ $NUM_OBJECTS -ge 4 ]; then
    echo "[4/$NUM_OBJECTS] Cylinder spawning (-2.0, -1.5, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_cyl2 -database unit_cylinder -x -2.0 -y -1.5 -z 0.5 &
    sleep 2
fi

# Objektum 5: Box jobbra
if [ $NUM_OBJECTS -ge 5 ]; then
    echo "[5/$NUM_OBJECTS] Box spawning (3.0, 0.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_box3 -database unit_box -x 3.0 -y 0.0 -z 0.5 &
    sleep 2
fi

# Objektum 6: Cylinder balra
if [ $NUM_OBJECTS -ge 6 ]; then
    echo "[6/$NUM_OBJECTS] Cylinder spawning (-3.0, 0.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_cyl3 -database unit_cylinder -x -3.0 -y 0.0 -z 0.5 &
    sleep 2
fi

# Objektum 7: Box előre középen
if [ $NUM_OBJECTS -ge 7 ]; then
    echo "[7/$NUM_OBJECTS] Box spawning (2.5, 0.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_box4 -database unit_box -x 2.5 -y 0.0 -z 0.5 &
    sleep 2
fi

# Objektum 8: Cylinder előre jobbra (közelebb)
if [ $NUM_OBJECTS -ge 8 ]; then
    echo "[8/$NUM_OBJECTS] Cylinder spawning (1.5, 2.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_cyl4 -database unit_cylinder -x 1.5 -y 2.0 -z 0.5 &
    sleep 2
fi

# Objektum 9: Box előre balra (közelebb)
if [ $NUM_OBJECTS -ge 9 ]; then
    echo "[9/$NUM_OBJECTS] Box spawning (1.5, -2.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_box5 -database unit_box -x 1.5 -y -2.0 -z 0.5 &
    sleep 2
fi

# Objektum 10: Cylinder hátra középen
if [ $NUM_OBJECTS -ge 10 ]; then
    echo "[10/$NUM_OBJECTS] Cylinder spawning (-2.5, 0.0, 0.5)..."
    ros2 run gazebo_ros spawn_entity.py -entity stress_cyl5 -database unit_cylinder -x -2.5 -y 0.0 -z 0.5 &
    sleep 2
fi

# Várakozás az összes spawning befejeződésére
echo ""
echo "Várakozás az objektumok spawning-jának befejeződésére..."
wait

echo ""
echo "============================================"
echo "  Spawning befejezve!"
echo "  Létrehozott objektumok: $NUM_OBJECTS"
echo "============================================"
echo ""
echo "✅ Ellenőrizd a Gazebo-ban és az RViz-ben!"
echo "✅ Ha látszanak az objektumok, indítsd a rosbag rögzítést."
echo ""
echo "⚠️  FONTOS: Ez a script azonnal kilép!"
echo "   Objektumok a világban maradnak a teszt végéig."
echo ""
echo "   Objektumok törlése (teszt után):"
echo "   ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \"{name: 'stress_box1'}\""
echo ""
echo "   💡 Folyamatos spawning-hoz használd: ./continuous_spawn.sh"
echo ""
