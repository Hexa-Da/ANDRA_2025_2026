#!/bin/bash
# Script d'initialisation complet pour le projet ANDRA
# Usage: source scripts/setup.sh

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Initialisation du projet ANDRA..."
echo "Répertoire projet: $PROJECT_DIR"

# 1. ROS2 de base
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble initialisé"
else
    echo "❌ Erreur: ROS2 Humble non trouvé"
    return 1
fi

# 2. Workspace YDLidar 
YDLIDAR_WS="$PROJECT_DIR/dependencies/ydlidar_ros2_ws"
if [ -f "$YDLIDAR_WS/install/setup.bash" ]; then
    source "$YDLIDAR_WS/install/setup.bash"
    echo "✅ Workspace YDLidar sourcé"
elif [ -d "u$YDLIDAR_WS" ]; then
    echo "❌ Workspace YDLidar trouvé mais non compilé (exécutez: scripts/build.sh)"
else
    echo "❌ Workspace YDLidar non trouvé dans dependencies/"
fi

# 3. Workspace scout_base 
SCOUT_WS="$PROJECT_DIR/dependencies/scout_base"
if [ -f "$SCOUT_WS/install/setup.bash" ]; then
    source "$SCOUT_WS/install/setup.bash"
    echo "✅ Workspace scout_base sourcé"
else
    echo "❌ Workspace scout_base non trouvé dans dependencies/"
fi

# 4. Worksapace ZED Wrapper
ZED_WS="$PROJECT_DIR/dependencies/zed-ros2-wrapper"
if [ -f "$ZED_WS/install/setup.bash" ]; then
    source "$ZED_WS/install/setup.bash"
    echo "✅ Workspace ZED Wrapper sourcé"
elif [ -d "$ZED_WS" ]; then
    echo "❌ Workspace ZED Wrapper trouvé mais non compilé (exécutez: scripts/build.sh zed)"
else
    echo "❌  Workspace ZED Wrapper non trouvé dans dependencies/"
fi

# 5. Workspace principal ANDRA
ANDRA_WS="$PROJECT_DIR/ros2_ws"
if [ -f "$ANDRA_WS/install/setup.bash" ]; then
    cd "$ANDRA_WS"
    source install/setup.bash
    echo "✅ Workspace ANDRA sourcé"
elif [ -d "$ANDRA_WS" ]; then
    echo "❌ Workspace ANDRA trouvé mais non compilé (exécutez: scripts/build.sh)"
else
    echo "❌ Erreur: Workspace ANDRA non trouvé"
    return 1
fi

# 6. Configuration middleware DDS ROS2 (pour éviter les problèmes avec scout_base)
if [ -z "$RMW_IMPLEMENTATION" ]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    echo "✅ Middleware DDS configuré: $RMW_IMPLEMENTATION"
fi

# Retrour à la racine du projet
cd ..

# Vérification finale
echo ""
echo "✅ Initialisation terminée!"
echo ""
