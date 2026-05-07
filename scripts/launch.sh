#!/bin/bash
# Script de lancement du système complet
# Usage: ./scripts/launch.sh [slam|amcl] [options...]

set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Aller dans le répertoire du projet
cd "$PROJECT_DIR"

# Lancer selon le mode (par default slam)
MODE="${1:-slam}"

case "$MODE" in
    slam)
        echo "Lancement en mode SLAM..."
        echo ""
        echo "Options disponibles:"
        echo "  enable_lidar:=false           Désactiver le LIDAR"
        echo "  enable_scout:=false           Désactiver Scout Base"
        echo "  enable_zed:=false             Désactiver la caméra ZED"
        echo "  enable_ptz:=false             Désactiver la caméra PTZ"
        echo "  enable_image_transfer:=false  Désactiver les nœuds de capture et de traitement d'images"
        echo ""
        echo "Repère LiDAR (TF base_link → laser_frame) :"
        echo "  (défaut) π/2 rad (~1.570796327) — usage courant sans argument"
        echo "  laser_mount_yaw:=0.0            Création de carte / SLAM"
        echo ""
        echo ""
        echo "--> pour sauvegarder la carte, avant de couper les nœuds, executer :"
        echo "ros2 run nav2_map_server map_saver_cli -f ros2_ws/src/ros_launcher/map_results/ma_carte"
        echo ""
        
        # Passer les arguments supplémentaires à ros2 launch
        shift  # Retirer le premier argument (slam)
        ros2 launch ros_launcher navigation_stack.launch.py use_slam:=true "$@"
        ;;

    amcl)
        echo "Lancement en mode AMCL..."
        echo "Navigation Nav2 activée par défaut (use_nav:=true)."
        echo "Pour localisation seule: ajouter use_nav:=false"
        echo ""
        echo "Options disponibles:"
        echo ""
        echo "  enable_lidar:=false           Désactiver le LIDAR"
        echo "  enable_scout:=false           Désactiver Scout Base"
        echo "  enable_zed:=false             Désactiver la caméra ZED"
        echo "  enable_ptz:=false             Désactiver la caméra PTZ"
        echo "  enable_image_transfer:=false  Désactiver les nœuds de capture et de traitement d'images"
        echo "  enable_video_publisher:=true  Réactiver le scan périodique des vidéos (désactivé par défaut)"
        echo ""
        echo "Repère LiDAR (TF base_link → laser_frame) :"
        echo "  (défaut) π/2 rad (~1.570796327) — usage courant sans argument"
        echo "  laser_mount_yaw:=0.0            Création de carte / SLAM"
        echo ""
        # si pas de carte spécifiée ou chemin invalide, on quitte le script
        if [ -z "${2:-}" ]; then
            echo "❌ Carte non spécifiée."
            echo "   Usage: ./scripts/launch.sh amcl <chemin/vers/ma_carte.yaml>"
            exit 1
        fi
        MAP_PATH="$2"
        if [ ! -f "$MAP_PATH" ]; then
            echo "❌ Fichier carte introuvable: $MAP_PATH"
            exit 1
        fi
        shift 2  # Retirer le mode (amcl) et le chemin de carte
        ros2 launch ros_launcher navigation_stack.launch.py use_slam:=false use_amcl:=true use_nav:=true map_path:="$MAP_PATH" "$@"
        ;;
    *)
        echo "Option non reconnue. Choisissez slam ou amcl. Par défaut : slam"
        exit 1
        ;;
esac
