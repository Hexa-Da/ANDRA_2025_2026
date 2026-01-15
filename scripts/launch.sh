#!/bin/bash
# Script de lancement du système complet
# Usage: ./scripts/launch.sh [slam|amcl] [options...]

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Aller dans le répertoire de lancement
cd "$PROJECT_DIR/ros_launcher"

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
        echo "Options PTZ (ajustement d'image):"
        echo "  ptz_brightness:=(1.0=normal, >1.0=plus lumineux)           Multiplicateur de luminosité"
        echo "  ptz_contrast:=(1.0=normal, >1.0=plus de contraste)         Multiplicateur de contraste"
        echo "  ptz_gamma:=(1.0=normal, <1.0=plus lumineux)                Correction gamma"
        echo ""
        
        # Passer les arguments supplémentaires à ros2 launch
        shift  # Retirer le premier argument (slam)
        ros2 launch navigation_stack.launch.py use_slam:=true "$@"

        echo ""
        echo "--> executé : ros2 run nav2_map_server map_saver  -f ~/Documents/ANDRA_2025-2026/ros_launcher/ma_carte"
        echo "pour sauvegarder la carte créer"
        echo ""
        ;;

    amcl)
        echo "Lancement en mode AMCL..."
        # si pas de carte specifier, on affiche un message d'erreur et on quitte le script
        if [ -z "$2" ]; then
            echo "❌ Carte non specifier ou erreur sur le chemin de la carte"
            exit 1
        fi
        ros2 launch navigation_stack.launch.py use_slam:=false use_amcl:=true map_path:="$2"
        ;;
    *)
        exit 1
        ;;
esac
