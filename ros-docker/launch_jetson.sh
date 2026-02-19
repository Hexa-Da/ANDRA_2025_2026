#!/bin/bash
# Script pour lancer le nœud image_subscriber avec PyTorch sur Jetson
# Usage: ./ros-docker/launch_jetson.sh

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS_DIR="$PROJECT_DIR/ros2_ws"

# Image Docker pour Jetson
IMAGE_JETSON="ros2-humble-pytorch-jetson:r36.2.0"
BASE_IMAGE="dustynv/l4t-pytorch:r36.2.0"

# Vérifier si l'image Jetson existe
if docker images -q "$IMAGE_JETSON" 2>/dev/null | grep -q .; then
    IMAGE="$IMAGE_JETSON"
    echo "✅ Utilisation de l'image Jetson personnalisée: $IMAGE"
else
    IMAGE="$BASE_IMAGE"
    echo "⚠️ Image Jetson personnalisée non trouvée, utilisation de: $IMAGE"
    echo "   💡 Pour créer l'image Jetson (une seule fois, ~20-30 min):"
    echo "      cd $SCRIPT_DIR && docker build -t $IMAGE_JETSON -f Dockerfile.jetson ."
fi

echo "🤖 Démarrage du Nœud IA sous Docker GPU sur Jetson..."
echo "📁 Workspace ROS2: $ROS2_WS_DIR"
echo ""

# Vérifier que le workspace est compilé
if [ ! -f "$ROS2_WS_DIR/install/setup.bash" ]; then
    echo "❌ Erreur: Le workspace ROS2 n'est pas compilé!"
    echo "   Exécutez d'abord: cd $ROS2_WS_DIR && colcon build"
    exit 1
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
    $IMAGE \
    /bin/bash -c "
    # Vérifier si ROS 2 Humble est déjà installé
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        echo '⚠️ ROS 2 Humble non trouvé, installation...' && \
        export DEBIAN_FRONTEND=noninteractive && \
        apt-get update -qq && \
        apt-get install -y -qq curl gnupg2 lsb-release software-properties-common && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        ARCH=\$(dpkg --print-architecture) && \
        CODENAME=\$(lsb_release -cs) && \
        printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \"\${ARCH}\" \"\${CODENAME}\" > /etc/apt/sources.list.d/ros2-latest.list && \
        apt-get update -qq && \
        apt-get install -y -qq ros-humble-desktop python3-rosdep python3-colcon-common-extensions && \
        rosdep init || true && \
        rosdep update || true && \
        echo '✅ ROS 2 Humble installé'
    else
        echo '✅ ROS 2 Humble déjà disponible'
    fi && \
    
    echo '🔧 Vérification des dépendances YOLO...' && \
    python3 -c \"import ultralytics, cv2, torch\" >/dev/null 2>&1 || \
      pip install --no-cache-dir --index-url https://pypi.org/simple 'numpy<2.0' 'opencv-python<4.10' ultralytics >/dev/null 2>&1 && \
    
    echo '🔍 Vérification du modèle YOLO...' && \
    if [ ! -f /ros2_ws/models/best.engine ] && [ ! -f /ros2_ws/models/best.pt ]; then
        echo '❌ Modèle non trouvé (ni best.engine ni best.pt dans /ros2_ws/models/)'
        exit 1
    fi && \
    
    echo '🐍 Sourcing de ROS 2 et du workspace...' && \
    source /opt/ros/humble/setup.bash && \
    source install/local_setup.bash && \
    
    echo '🔍 Contexte ROS2 conteneur...' && \
    echo '   ROS_DOMAIN_ID:' \$ROS_DOMAIN_ID && \
    echo '   RMW_IMPLEMENTATION:' \$RMW_IMPLEMENTATION && \
    echo '   FASTDDS_BUILTIN_TRANSPORTS:' \$FASTDDS_BUILTIN_TRANSPORTS && \
    echo '' && \
    echo '🚀 Lancement du nœud image_subscriber...' && \
    ros2 run image_transfer image_subscriber
    "
