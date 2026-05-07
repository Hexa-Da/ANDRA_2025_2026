## Commandes de débogage ROS2

### Vérifier les nœuds actifs

```bash
# Lister tous les nœuds
ros2 node list

# Voir les détails d'un nœud
ros2 node info <nom_du_noeud>
```

### Vérifier les topics actifs

```bash
ros2 topic list
```

### Vérifier les messages publiés

#### Topics de détection et traitement d'images

```bash
# Images capturées depuis la caméra PTZ ou extraites de vidéos (sensor_msgs/Image)
# Publié par : image_publisher (caméra PTZ) ou video_publisher (vidéos enregistrées)
# Souscrit par : image_subscriber
ros2 topic echo /photo_topic

# Vérifier la fréquence de publication
ros2 topic hz /photo_topic

# Positions où des fissures ont été détectées (geometry_msgs/Point)
# Publié par : image_subscriber
# Souscrit par : report_fissures
ros2 topic echo /position_detectee
```

#### Vérifier l'utilisation GPU

```bash
# Sur Jetson
tegrastats

# Ou avec jtop
jtop
```

#### Topics d'odométrie et localisation

```bash
# Odométrie brute du robot Scout (nav_msgs/Odometry)
# Publié par : scout_base
# Utilisé par : EKF (filtre de Kalman étendu)
ros2 topic echo /odom_robot

# Odométrie filtrée par le filtre EKF (nav_msgs/Odometry)
# Publié par : EKF
# Souscrit par : image_subscriber, odom_to_path, sequence_photo, sequence_video
# (position_publisher et show_pos sont WIP, voir docs/STRUCTURE.md)
ros2 topic echo /odometry/filtered

# Trajet du robot (nav_msgs/Path)
# Publié par : odom_to_path 
# Visualisé dans RViz2 via le display Path
ros2 topic echo /robot_path
```

#### Topics de contrôle PTZ

```bash
# Commande de vitesse pour la caméra PTZ (geometry_msgs/Twist)
# Souscrit par : ptz_controller
ros2 topic echo /ptz/cmd_vel

# Commande de preset pour la caméra PTZ (std_msgs/Int32)
# Souscrit par : ptz_controller
ros2 topic echo /ptz/preset
```

## Vérifier les transforms TF

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 run tf2_ros tf2_echo map odom
```

## Vérifier la configuration DDS et la communication réseau

### Vérifier ROS_DOMAIN_ID

```bash
# Sur le robot
echo $ROS_DOMAIN_ID  # Doit afficher 0

# Dans le conteneur Docker (si utilisé)
echo $ROS_DOMAIN_ID  # Doit également afficher 0
```

### Vérifier le middleware DDS (RMW_IMPLEMENTATION)

```bash
# Sur le robot
echo $RMW_IMPLEMENTATION  # Doit afficher rmw_fastrtps_cpp

# Dans le conteneur Docker (si utilisé)
echo $RMW_IMPLEMENTATION  # Doit également afficher rmw_fastrtps_cpp (ou vide = FastRTPS par défaut)
```

**Important** : Si les middlewares diffèrent (FastRTPS vs CycloneDDS), les topics ne seront pas visibles même si le réseau fonctionne.

### Prendre le contrôle manuel

Pour controler les roues :
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Pour visioner les informations de la zed2i dans RViz :
```bash
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```