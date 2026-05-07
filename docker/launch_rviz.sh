#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="ros2-humble-custom"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$PROJECT_ROOT/ros2_ws"

# Autoriser l'accès X11 local
xhost +local:

# Construire automatiquement l'image si absente
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "Image '$IMAGE_NAME' absente. Construction en cours..."
  docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile.rviz" "$SCRIPT_DIR"
fi

# Activer GPU uniquement si NVIDIA est detecte
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

# Propage ROS_DOMAIN_ID / RMW_IMPLEMENTATION depuis l'environnement hôte (avec
# defauts cohérents avec scripts/setup.sh et docker/launch_jetson.sh).
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

docker run -it --rm \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  "${GPU_ARGS[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$WORKSPACE_DIR:/workspace/ros2_ws" \
  "$IMAGE_NAME" /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    [ -f install/setup.bash ] && source install/setup.bash || echo 'Workspace non compile: colcon build' && \
    cd /workspace/ && \
    exec /bin/bash
  "