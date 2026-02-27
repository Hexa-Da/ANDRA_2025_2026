#!/bin/bash
# Script pour lancer le nœud image_subscriber avec PyTorch sur Jetson
# Usage: ./docker/launch_jetson.sh

# Obtenir le répertoire du script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS_DIR="$PROJECT_DIR/ros2_ws"

# Image Docker pour Jetson
IMAGE_JETSON="ros2-humble-pytorch-jetson:r36.2.0"
BASE_IMAGE="dustynv/l4t-pytorch:r36.2.0"

# Verifier si l'image Jetson existe
if docker images -q "$IMAGE_JETSON" 2>/dev/null | grep -q .; then
    IMAGE="$IMAGE_JETSON"
    echo "[OK] Image Jetson: $IMAGE"
else
    IMAGE="$BASE_IMAGE"
    echo "[WARN] Image personnalisee non trouvee, utilisation de: $IMAGE"
    echo "       Pour la creer: cd $SCRIPT_DIR && docker build -t $IMAGE_JETSON -f Dockerfile.jetson ."
fi

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
    $IMAGE \
    /bin/bash -c "
    # Installer ROS 2 Humble si necessaire
    if [ ! -f /opt/ros/humble/setup.bash ]; then
        echo '[WARN] ROS 2 Humble non trouve, installation...' && \
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
        echo '[OK] ROS 2 Humble installe'
    else
        echo '[OK] ROS 2 Humble disponible'
    fi && \
    
    # Verifier dependances YOLO
    python3 -c \"import ultralytics, cv2, torch\" >/dev/null 2>&1 || \
      pip install --no-cache-dir --index-url https://pypi.org/simple 'numpy<2.0' 'opencv-python<4.10' ultralytics >/dev/null 2>&1 && \
    
    # Verifier modele YOLO
    if [ -f /ros2_ws/models/best.engine ] && [ -r /ros2_ws/models/best.engine ]; then
        echo '[OK] Modele TensorRT: best.engine'
    elif [ -f /ros2_ws/models/best.pt ] && [ -r /ros2_ws/models/best.pt ]; then
        echo '[OK] Modele PyTorch: best.pt'
    else
        echo '[ERROR] Modele non trouve dans /ros2_ws/models/'
        exit 1
    fi && \
    
    # Sourcing ROS 2
    source /opt/ros/humble/setup.bash && \
    source install/local_setup.bash && \
    
    echo 'ROS_DOMAIN_ID:' \$ROS_DOMAIN_ID '| RMW:' \$RMW_IMPLEMENTATION && \
    echo 'Lancement image_subscriber...' && \
    ros2 run image_transfer image_subscriber
    "
