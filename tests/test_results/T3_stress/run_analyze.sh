#!/bin/bash
# Wrapper script rosbag elemzéshez
# Source-olja a ROS környezetet és futtatja az analyze_objects.py-t

echo "🔧 Conda környezet deaktiválása..."
# Conda deaktiválás ha aktív
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "   Conda env found: $CONDA_DEFAULT_ENV - deaktiválás..."
    conda deactivate 2>/dev/null || true
fi

# Biztosítjuk hogy system Python-t használjunk
export PATH="/usr/bin:$PATH"
unset PYTHONPATH

echo "🔧 ROS környezet beállítása..."
source /opt/ros/jazzy/setup.bash
source /home/mark/codes/mgm/project_mgm/install/setup.bash

echo "📊 Analyze Objects indítása..."
echo "⚠️ Nyomd meg Ctrl+C az elemzés befejezéséhez és statisztikák kiírásához"
echo ""

/usr/bin/python3 /home/mark/codes/mgm/project_mgm/tests/test_results/T3_stress/analyze_objects.py
