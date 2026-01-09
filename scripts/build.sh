#!/bin/bash
# Script de compilation des workspaces
# Usage: ./scripts/build.sh [ydlidar|ros2_ws|scout_base|zed]

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Initialiser ROS2
source /opt/ros/humble/setup.bash

# Définir la cible de compilation
# 1 correspond au premier argument passé au script ($1) donc ici ydlidar ou ros2_ws 
# sinon on prend par default all
BUILD_TARGET="${1:-all}" 

case "$BUILD_TARGET" in
    ydlidar)
        echo "Compilation du workspace YDLidar..."
        cd "$PROJECT_DIR/dependencies/ydlidar_ros2_ws"
        colcon build
        ;;
    ros2_ws)
        echo "Compilation du workspace ros2_ws..."
        cd "$PROJECT_DIR/ros2_ws"
        colcon build
        ;;
    scout_base)
        echo "Compilation du workspace scout_base..."
        cd "$PROJECT_DIR/dependencies/scout_base"
        colcon build
        ;;
    zed)
        echo "Compilation du workspace ZED Wrapper..."
        cd "$PROJECT_DIR/dependencies/zed-ros2-wrapper"
        colcon build
        ;;
    all)
        echo "Compilation de tous les workspaces..."
        
        # YDLidar
        echo "YDLidar..."
        cd "$PROJECT_DIR/dependencies/ydlidar_ros2_ws"
        colcon build

        # scout_base
        echo "scout base..."
        cd "$PROJECT_DIR/dependencies/scout_base"
        colcon build

        # Zed wrapper
        echo "ZED Wrapper..."
        cd "$PROJECT_DIR/dependencies/zed-ros2-wrapper"
        colcon build
        
        # ros2_ws
        echo "ros2_ws..."
        cd "$PROJECT_DIR/ros2_ws"
        colcon build
        ;;
    *)
        exit 1
        ;;
esac

echo ""
echo "✅ Compilation terminée!"
echo ""
echo "--> Réexecutez la commande: source scripts/setup.sh"
echo ""
