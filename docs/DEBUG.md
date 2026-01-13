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
# Images capturées depuis la caméra PTZ (sensor_msgs/Image)
# Publié par : image_publisher
# Souscrit par : image_subscriber
ros2 topic echo /photo_topic

# Positions où des fissures ont été détectées (geometry_msgs/Point)
# Publié par : image_subscriber
# Souscrit par : report_fissures
ros2 topic echo /position_detectee
```

#### Topics d'odométrie et localisation

```bash
# Odométrie brute du robot Scout (nav_msgs/Odometry)
# Publié par : scout_base
# Utilisé par : EKF (filtre de Kalman étendu)
ros2 topic echo /odom_robot

# Odométrie filtrée par le filtre EKF (nav_msgs/Odometry)
# Publié par : EKF
# Souscrit par : image_subscriber, position_publisher
ros2 topic echo /odometry/filtered
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

### Explication des transformations TF

Les transformations TF (Transform) dans ROS2 représentent les relations géométriques entre les repères de coordonnées du robot. Elles permettent de convertir les coordonnées d'un capteur à un autre et de localiser le robot dans la carte.

**Structure de l'arbre TF :**
```
map → odom → base_link → {laser_frame, zed_camera_link}
```

**Frames utilisés :**
- **`map`** : Repère de la carte globale (publié par SLAM/AMCL)
- **`odom`** : Repère d'odométrie (publié par EKF, fusion des capteurs)
- **`base_link`** : Centre du robot (repère de référence)
- **`laser_frame`** : Repère du LIDAR (rotation de 90° par rapport à `base_link`)
- **`zed_camera_link`** : Repère de la caméra ZED2 (coïncide avec `base_link`)

**Transformations statiques** (fixes, position physique des capteurs) :
- `base_link` → `zed_camera_link` : (0, 0, 0, 0, 0, 0)
- `base_link` → `laser_frame` : (0, 0, 0, 0, 0, 1.57) - rotation de 90°

**Transformations dynamiques** (changent avec le mouvement) :
- `map` → `odom` : publiée par SLAM/AMCL
- `odom` → `base_link` : publiée par EKF
