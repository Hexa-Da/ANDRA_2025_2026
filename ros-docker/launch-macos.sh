#!/bin/bash

# Script de lancement Docker pour macOS avec XQuartz
# Prérequis: Docker Desktop démarré, XQuartz démarré, image ros2-humble-custom construite

# Configuration X11 pour macOS
DISPLAY_HOST="host.docker.internal:0"

# Autoriser les connexions X11
xhost + 127.0.0.1 2>/dev/null || true

# Lancer le conteneur avec montage du workspace
# Note: Configuration OpenGL logiciel pour macOS/XQuartz
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY_HOST \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=0 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e GALLIUM_DRIVER=llvmpipe \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    -e MESA_GLSL_VERSION_OVERRIDE=330 \
    -e __GLX_VENDOR_LIBRARY_NAME=mesa \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/../ros2_ws:/workspace/ros2_ws \
    -v $(pwd)/../ros_launcher:/workspace/ros_launcher \
    ros2-humble-custom /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        cd /workspace/ros2_ws && \
        [ -f install/setup.bash ] && source install/setup.bash || true && \
        export ROS_DOMAIN_ID=0 && \
        export LIBGL_ALWAYS_SOFTWARE=1 && \
        export GALLIUM_DRIVER=llvmpipe && \
        export MESA_GL_VERSION_OVERRIDE=3.3 && \
        export MESA_GLSL_VERSION_OVERRIDE=330 && \
        export __GLX_VENDOR_LIBRARY_NAME=mesa && \
        exec /bin/bash
    "
