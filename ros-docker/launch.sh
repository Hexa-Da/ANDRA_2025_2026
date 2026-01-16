# autoriser l'accès x11
xhost +local:

# lancer le conteneur avec montage du workspace
# ajouter/retirer NVIDIA_DRIVER_CAPABILITIES=all et --gpus all si vous avez un GPU
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/../ros2_ws:/workspace/ros2_ws \
    -v $(pwd)/../ros_launcher:/workspace/ros_launcher \
    ros2-humble-custom /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        cd /workspace/ros2_ws && \
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
        [ -f install/setup.bash ] && source install/setup.bash || echo 'Workspace non compilé: colcon build' && \
        cd /workspace/ && \
        export ROS_DOMAIN_ID=0 && \
        exec /bin/bash
    "