"""
ROS2 Python csomag setup fájl - lidar_filter csomag.

Ez a fájl definiálja a lidar_filter ROS2 csomag telepítési
konfigurációját. A setuptools és ament_python használatával
telepíti a csomagot és a szükséges fájlokat.

Főbb elemek:
- Package metaadatok (név, verzió, szerző)
- Data fájlok (launch scripts, RViz config)
- Entry points (futtatható node-ok)

ROS2 build rendszer (colcon) ezt a fájlt használja a csomag
build-eléséhez és telepítéséhez.
"""

from setuptools import setup
import os
from glob import glob

package_name = 'lidar_filter'

setup(
    # Csomag alapinformációk
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Python csomagok listája
    
    # Data fájlok telepítése - launch scripts, config fájlok, stb.
    # Ezek a fájlok a ROS2 share könyvtárba kerülnek
    data_files=[
        # Package resource index - ROS2 package discovery-hez
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package.xml - csomag metaadatok és függőségek
        ('share/' + package_name, ['package.xml']),
        # Launch fájlok - .launch.py scriptek
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # RViz konfiguráció fájlok - .rviz fájlok
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    
    # Függőségek és követelmények
    install_requires=['setuptools', 'numpy', 'scikit-learn', 'scipy'],  # Build dependency
    zip_safe=True,  # Csomag zip-elhető (nem tartalmaz C extensionöket)
    
    # Szerző és license információk
    maintainer='Mitrenga Márk',
    maintainer_email='mitrenga.mark@example.com',
    description='LIDAR-based object detection and tracking for MGM project',
    license='Apache-2.0',
    
    # Tesztelési követelmények
    tests_require=['pytest'],
    
    # Entry points - ROS2 node-ok definíciója
    # Ezek a 'ros2 run' paranccsal futtathatók
    entry_points={
        'console_scripts': [
            # Format: 'futtatható_név = csomag.modul:függvény'
            # Példa: ros2 run lidar_filter lidar_filter_node
            'lidar_filter_node = lidar_filter.lidar_filter_node:main',
        ],
    },
)
