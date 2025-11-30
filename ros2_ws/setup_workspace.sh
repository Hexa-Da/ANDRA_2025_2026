#!/bin/bash
# setup_workspace.sh - Script d'initialisation du workspace ROS2

source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
source install/setup.bash

echo "Initialisation du workspace ROS2"
echo "Packages disponibles:"
ros2 pkg list | grep -E "(image_transfer|slam)" || echo "  (aucun package trouvé)"