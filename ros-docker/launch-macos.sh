#!/bin/bash

# Script de lancement Docker pour macOS avec XQuartz
# Prérequis: XQuartz doit être installé et démarré

# Vérifier que XQuartz est en cours d'exécution
if ! pgrep -x "Xquartz" > /dev/null; then
    echo "⚠️  XQuartz n'est pas démarré. Veuillez le lancer depuis Applications > Utilitaires > XQuartz"
    echo "   Ou installer XQuartz depuis: https://www.xquartz.org/"
    exit 1
fi

# Obtenir l'adresse IP de la machine macOS pour X11
# Sur macOS avec Docker Desktop, on utilise host.docker.internal
if [[ "$OSTYPE" == "darwin"* ]]; then
    # Obtenir l'IP de la machine (pour XQuartz)
    IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || echo "127.0.0.1")
    
    # Sur macOS, XQuartz écoute sur localhost par défaut
    # Docker Desktop peut accéder via host.docker.internal
    DISPLAY_HOST="host.docker.internal:0"
    
    # Autoriser les connexions X11 depuis Docker (via xhost)
    # Note: xhost peut ne pas être dans le PATH, mais on essaie quand même
    if command -v xhost >/dev/null 2>&1; then
        xhost + 127.0.0.1 2>/dev/null
        xhost + "$IP" 2>/dev/null || true
    else
        echo "⚠️  xhost non trouvé dans le PATH. Assurez-vous que XQuartz est correctement installé."
        echo "   Vous pouvez essayer: export PATH=\"/opt/X11/bin:\$PATH\""
    fi
else
    # Pour Linux, utiliser le comportement standard
    xhost +local:
    DISPLAY_HOST="${DISPLAY:-:0}"
fi

echo "🚀 Lancement du conteneur Docker ROS2..."
echo "📺 Affichage X11 configuré pour: $DISPLAY_HOST"

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
        [ -f install/setup.bash ] && source install/setup.bash || echo '⚠️  Workspace non compilé: exécutez colcon build' && \
        export ROS_DOMAIN_ID=0 && \
        echo '✅ Environnement ROS2 initialisé' && \
        echo '📁 Workspaces montés:' && \
        echo '   - /workspace/ros2_ws' && \
        echo '   - /workspace/ros_launcher' && \
        echo '' && \
        echo '💡 Pour lancer RViz2 avec votre configuration:' && \
        echo '   rviz2 -d /workspace/ros_launcher/config.rviz' && \
        echo '' && \
        exec /bin/bash
    "
