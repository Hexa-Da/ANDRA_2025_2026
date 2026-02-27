#!/bin/bash

# Script pour lancer le nœud image_subscriber avec PyTorch dans Docker GPU
# Usage: ./scripts/image_subscriber_gpu.sh
# Note: Ce script utilise maintenant ros2_docker/launch_jetson.sh pour une infrastructure unifiée

# Ce script délègue maintenant à ros2_docker/launch_jetson.sh pour une infrastructure unifiée
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS_DOCKER_SCRIPT="$PROJECT_DIR/ros2_docker/launch_jetson.sh"

if [ -f "$ROS_DOCKER_SCRIPT" ]; then
    echo "Utilisation de l'infrastructure Docker unifiée..."
    exec "$ROS_DOCKER_SCRIPT"
else
    echo "❌ Erreur: Script ros2_docker/launch_jetson.sh non trouvé!"
    echo "   Vérifiez que le fichier existe: $ROS_DOCKER_SCRIPT"
    exit 1
fi