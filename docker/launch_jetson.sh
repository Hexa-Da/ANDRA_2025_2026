#!/bin/bash
# Script pour lancer le nœud image_subscriber avec PyTorch sur Jetson
# Usage: ./docker/launch_jetson.sh

set -eo pipefail

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS_DIR="$PROJECT_DIR/ros2_ws"

# Image Docker pour Jetson (override possible via JETSON_IMAGE)
IMAGE_JETSON="${JETSON_IMAGE:-ros2-humble-pytorch-jetson:r36.2.0}"

# Verifier si l'image Jetson personnalisee existe (sinon abort, sinon ROS serait
# reinstalle a chaque run, ce qui est lent et incoherent avec Dockerfile.jetson)
if ! docker images -q "$IMAGE_JETSON" 2>/dev/null | grep -q .; then
    echo "[ERROR] Image Docker '$IMAGE_JETSON' introuvable."
    echo "        Construisez-la avec :"
    echo "          cd $SCRIPT_DIR && sudo docker build -t $IMAGE_JETSON -f Dockerfile.jetson ."
    echo "        Ou definissez JETSON_IMAGE=<autre tag> avant de relancer ce script."
    exit 1
fi
IMAGE="$IMAGE_JETSON"
echo "[OK] Image Jetson: $IMAGE"

echo "Demarrage du noeud IA Docker GPU..."
echo "Workspace: $ROS2_WS_DIR"
echo ""

# Verifier que le workspace est compile
if [ ! -f "$ROS2_WS_DIR/install/setup.bash" ]; then
    echo "[ERROR] Workspace ROS2 non compile. Executez: cd $ROS2_WS_DIR && colcon build"
    exit 1
fi

# Corriger les permissions des modeles si necessaire
MODEL_DIR="$ROS2_WS_DIR/models"
if [ -d "$MODEL_DIR" ]; then
    for model in best.engine best.pt; do
        if [ -f "$MODEL_DIR/$model" ] && [ ! -r "$MODEL_DIR/$model" ]; then
            sudo chmod 644 "$MODEL_DIR/$model" 2>/dev/null
        fi
    done
fi

# Lancement du conteneur Docker pour Jetson
sudo docker run -it --rm \
    --runtime nvidia \
    --network host \
    --ipc host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -v "$ROS2_WS_DIR:/ros2_ws" \
    -v "$PROJECT_DIR:/project" \
    -w /ros2_ws \
    "$IMAGE" \
    /bin/bash -c "
    set -e

    # Sanity check: ROS 2 Humble doit etre dans l'image (Dockerfile.jetson l'installe)
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        echo '[ERROR] ROS 2 Humble absent de l image. Reconstruisez Dockerfile.jetson.'
        exit 1
    fi

    # Verifier modele YOLO
    if [ -f /ros2_ws/models/best.engine ] && [ -r /ros2_ws/models/best.engine ]; then
        echo '[OK] Modele TensorRT: best.engine'
    elif [ -f /ros2_ws/models/best.pt ] && [ -r /ros2_ws/models/best.pt ]; then
        echo '[OK] Modele PyTorch: best.pt'
    else
        echo '[ERROR] Modele non trouve dans /ros2_ws/models/'
        exit 1
    fi

    # Sourcing ROS 2 + workspace
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash

    echo 'ROS_DOMAIN_ID:' \$ROS_DOMAIN_ID '| RMW:' \$RMW_IMPLEMENTATION
    echo 'Lancement image_subscriber...'
    exec ros2 run image_transfer image_subscriber
    "
