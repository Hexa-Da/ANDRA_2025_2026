# Arbre TF

## Vue d'ensemble

L'arbre TF décrit les relations spatiales entre les repères (frames) du robot. Chaque transformation relie un repère parent à un repère enfant, avec une translation (x, y, z) et une rotation (roll, pitch, yaw).

```
map
  └── odom                                [SLAM ou AMCL]
        └── base_link                     [EKF publie odom → base_link]
              ├── base_footprint          [URDF Scout via robot_state_publisher]
              ├── zed_camera_link         [static : mesures terrain]
              └── laser_frame             [static : mesures terrain + yaw 90°]
```

Le nœud ZED est lancé avec **`publish_tf:=false`** : il ne publie **pas** `odom -> zed_camera_link` ; la pose mécanique de la caméra est uniquement **`base_link -> zed_camera_link`** (statique).

## Détail des transformations

### 1. `map` → `odom` (dynamique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | Dynamique |
| **Publié par** | `slam_toolbox` (SLAM) ou `nav2_amcl` (localisation) |
| **Fallback** | `static_transform_publisher` identité si inactifs |

Corrige la dérive d'odométrie en recadrant le robot sur la carte. Fallback identité si aucune map disponible.

### 2. `odom` → `base_link` (dynamique, EKF)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | Dynamique |
| **Publié par** | `robot_localization` (`ekf_filter_node`) |
| **Config** | `configs/ekf_config.yaml` |

**Sources fusionnées par l'EKF :**

| Source | Topic | Données |
|--------|-------|---------|
| Odométrie roues (Scout) | `/odom_robot` | X, Y + vit. linéaire |
| Odométrie visuelle (ZED2i) | `/zed/zed_node/odom` | X, Y + vit. linéaire |
| IMU (ZED2i) | `/zed/zed_node/imu/data` | Yaw + vit. angulaire |

`publish_tf: true` dans `ekf_config.yaml`. Un seul nœud doit publier cette TF (conflit sinon).

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
| **Translation** | **(0,085, 0, 0,222)** m — soit **x ≈ 8,5 cm, z ≈ 22,2 cm** |
| **Rotation** | yaw (Z) = **1,57 rad** (~90°) |
| **Publié par** | `navigation_stack.launch.py` (`base_to_laser_tf`) |

LIDAR YDLidar TG15 modélisé tourné de 90° par rapport à l'avant. 