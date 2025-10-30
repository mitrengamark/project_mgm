#!/bin/bash
# Wrapper script rosbag elemzéshez
# Source-olja a ROS környezetet és futtatja az analyze_objects.py-t

echo "🔧 ROS környezet beállítása..."
source /opt/ros/jazzy/setup.bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

echo "📊 Analyze Objects indítása..."
echo "⚠️ Nyomd meg Ctrl+C az elemzés befejezéséhez és statisztikák kiírásához"
echo ""

python3 /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/analyze_objects.py
