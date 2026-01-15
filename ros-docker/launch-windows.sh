#!/bin/bash

# Script de lancement Docker pour Windows (WSL ou Git Bash)
# Prérequis: Docker Desktop démarré, VcXsrv/X410 démarré, image ros2-humble-custom construite

# Configuration X11 pour Windows
if [ -n "$WSL_DISTRO_NAME" ]; then
    # Dans WSL, utiliser l'IP de la machine Windows
    DISPLAY_HOST=$(ip route show | grep -i default | awk '{ print $3}'):0.0
else
    # Dans Git Bash ou autre, utiliser host.docker.internal
    DISPLAY_HOST="host.docker.internal:0.0"
fi

# Lancer le conteneur avec montage du workspace
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY_HOST \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/../ros2_ws:/workspace/ros2_ws \
    -v $(pwd)/../ros_launcher:/workspace/ros_launcher \
    ros2-humble-custom /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        cd /workspace/ros2_ws && \
        [ -f install/setup.bash ] && source install/setup.bash || true && \
        export ROS_DOMAIN_ID=0 && \
        exec /bin/bash
    "
