# autoriser l'accès x11
xhost +local:

# lancer le conteneur avec montage du workspace
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./config.rviz:/tmp/config.rviz \
    -v $(pwd)/../ros2_ws:/workspace/ros2_ws \
    -v $(pwd)/../ros_launcher:/workspace/ros_launcher \
    ros2-humble-custom /bin/bash
    
# a ajouter pour les ordis du techlab
# NVIDIA_DRIVER_CAPABILITIES=all \
# --gpus all \
