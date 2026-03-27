# Arbre TF

## Vue d'ensemble

L'arbre TF décrit les relations spatiales entre les repères (frames) du robot. Chaque transformation relie un repère parent à un repère enfant, avec une translation (x, y, z) et une rotation (roll, pitch, yaw).

```
world
  └── map                                 [static_transform_publisher : identité]
        └── odom                          [EKF ou AMCL]
              └── base_link               [EKF publie odom → base_link]
                    ├── zed_camera_link   [static : à mesurer]
                    └── laser_frame       [static : 90° yaw]
```

## Détail des transformations

### 1. `world` → `map` (statique, identité)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | `static_transform_publisher` |
| **Translation** | (0, 0, 0) |
| **Rotation** | (0, 0, 0) |
| **Publié par** | `navigation_stack.launch.py` (`world_to_map_tf`) |

Repère racine `world` ≡ `map`. Ancrage de la carte.

### 2. `map` → `odom` (dynamique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | Dynamique |
| **Publié par** | `slam_toolbox` (SLAM) ou `nav2_amcl` (localisation) |
| **Fallback** | `static_transform_publisher` identité si inactifs |

Corrige la dérive d'odométrie en recadrant le robot sur la carte. Fallback identité si aucune map disponible.

### 3. `odom` → `base_link` (dynamique, EKF)

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

### 4. `base_link` → `zed_camera_link` (statique)

| Paramètre | Valeur actuelle |
|-----------|-----------------|
| **Type** | `static_transform_publisher` |
| **Translation** | (0, 0, 0) |
| **Rotation** | (0, 0, 0) |
| **Publié par** | `navigation_stack.launch.py` (`base_to_zed_tf`) |

**Problème** : identité → ZED 2i considérée au centre du robot (faux) → erreurs EKF.
**Action** : mesurer position/orientation de la ZED2i par rapport à `base_link`, renseigner en m et rad.

### 5. `base_link` → `laser_frame` (statique)

| Paramètre | Valeur |
|-----------|--------|
| **Type** | `static_transform_publisher` |
| **Translation** | (0, 0, 0) |
| **Rotation** | (1.57, 0, 0) → 90° yaw |
| **Publié par** | `navigation_stack.launch.py` (`base_to_laser_tf`) |

LIDAR YDLidar TG15 modélisé tourné de 90° par rapport à l'avant. **À vérifier** 