# Arbre TF

## Vue d'ensemble

L'arbre TF décrit les relations spatiales entre les repères (frames) du robot. Chaque transformation relie un repère parent à un repère enfant, avec une translation (x, y, z) et une rotation (roll, pitch, yaw).

```
map
  └── odom                                [SLAM ou AMCL — exclusivement]
        └── base_link                     [Driver scout_base via /odom_robot]
              ├── base_footprint          [URDF Scout via robot_state_publisher]
              ├── zed_camera_link         [static : mesures terrain]
              └── laser_frame             [static : yaw au choix via launch]
```

Le nœud ZED est lancé avec **`publish_tf:=false`** : il ne publie **pas** `odom -> zed_camera_link` ; la pose mécanique de la caméra est uniquement **`base_link -> zed_camera_link`** (statique).

## Détail des transformations

### 1. `map` → `odom` (dynamique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | Dynamique |
| **Publié par** | `slam_toolbox` (mode SLAM) ou `nav2_amcl` (mode AMCL) |

Corrige la dérive d'odométrie en recadrant le robot sur la carte. **Aucun static identité de secours n'est publié** : si ni SLAM ni AMCL ne tourne, la TF `map -> odom` n'existe simplement pas (et `map` n'a pas vocation à être utilisé en mode SLAM tant que la carte n'a pas démarré).

### 2. `odom` → `base_link` (dynamique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | Dynamique |
| **Publié par** | `scout_base` (driver des roues) |
| **Topic odométrie associé** | `/odom_robot` |

C'est bien le **driver scout** qui publie cette TF, pas l'EKF. L'EKF est configuré avec `publish_tf: false` (`configs/ekf_config.yaml`) et n'alimente que le **topic** `/odometry/filtered` (consommé par `image_subscriber`, `position_publisher`, `odom_to_path`, `sequence_*`). Cela évite tout conflit de TF sur l'arête `odom -> base_link`.

**Sources fusionnées par l'EKF** (voir `configs/ekf_config.yaml`) :

| Source | Topic | Données |
|--------|-------|---------|
| Odométrie roues (Scout) | `/odom_robot` | X, Y, yaw + vitesses (référence principale) |
| IMU (ZED2i) | `/zed/zed_node/imu/data` | Vit. angulaire lacet (`vyaw`) uniquement |

L'odométrie visuelle ZED n'est plus fusionnée dans l'EKF (pour éviter les conflits avec les roues). La ZED reste tout de même utilisée pour l'image et surtout les données IMU. Si la ZED est désactivée (`enable_zed:=false`), `imu0` est coupé par `sensor_timeout` et l'EKF tourne avec `/odom_robot` seul ; le nœud EKF lui-même est conditionné à `enable_scout`.

### 3. `base_link` → `base_footprint` (statique via URDF Scout)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | `robot_state_publisher` (à partir de l'URDF Scout Mini) |
| **Translation** | (0, 0, -0.178) m |
| **Rotation** | (0, 0, 0) |
| **Publié par** | `robot_state_publisher` (`scout_mini.urdf`) |

Ce lien vient de l'URDF officiel Agilex (`scout_description`) et fournit une frame de référence au sol utile en navigation 2D.

### 4. `base_link` → `zed_camera_link` (statique)

| Paramètre | Valeur actuelle |
|-----------|-----------------|
| **Type** | `static_transform_publisher` |
| **Translation** | **(0,185, 0, 0,192)** m — soit **x ≈ 18,5 cm, z ≈ 19,2 cm** |
| **Rotation** | yaw (Z) = 0, pitch (Y) = 0, roll (X) = 0 |
| **Ordre** | `static_transform_publisher` : x y z yaw pitch roll |
| **Publié par** | `navigation_stack.launch.py` (`base_to_zed_tf`) |

**ZED wrapper** : `publish_tf:=false` — pas de TF `odom -> zed_camera_link` ; seul le montage rigide sur `base_link` s’applique.

### 5. `base_link` → `laser_frame` (statique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | `static_transform_publisher` |
| **Translation** | **(0.085, 0, 0.222)** m — soit **x ≈ 8,5 cm, z ≈ 22,2 cm** |
| **Rotation** | `yaw` = argument ROS `laser_mount_yaw` (par défaut `1.570796327` pour π/2 rad, sinon `0.0` pour la création de carte), pitch et roll = **0** |
| **Publié par** | `navigation_stack.launch.py` (`base_to_laser_tf`) |

Exemples : `laser_mount_yaw:=0.0` (avant du robot), `laser_mount_yaw:=1.570796327` (LiDAR tourné de 90° autour de Z).