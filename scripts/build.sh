#!/bin/bash
# Script de compilation des workspaces
# Usage: ./scripts/build.sh [ydlidar|andra]

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Initialiser ROS2
source /opt/ros/humble/setup.bash

# Définir la cible de compilation
# 1 correspond au premier argument passé au script ($1) donc ici ydlidar ou andra 
# sinon on prend par default all
BUILD_TARGET="${1:-all}" 

case "$BUILD_TARGET" in
    ydlidar)
        echo "Compilation du workspace YDLidar..."
        cd "$PROJECT_DIR/dependencies/ydlidar_ros2_ws"
        colcon build
        ;;
    andra)
        echo "Compilation du workspace ANDRA..."
        cd "$PROJECT_DIR/ros2_ws"
        colcon build
        ;;
    scout_base)
        echo "Compilation du workspace scout_base..."
        cd "$PROJECT_DIR/dependencies/scout_base"
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
        
        # ANDRA
        echo "ANDRA..."
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
