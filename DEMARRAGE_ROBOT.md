# Guide de démarrage des nœuds sur le robot

Ce document explique comment démarrer les nœuds ROS2 sur le robot Agilex Scout Mini.

## Connexion au robot

```bash
ssh techlab@192.168.40.99
```

## 1. Configuration préalable

### 1.1. Initialiser ROS2

```bash
source /opt/ros/humble/setup.bash
```

### 1.2. Configurer l'interface CAN pour les roues

**Important** : Cette commande doit être exécutée avant de lancer les nœuds du robot.

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 1.3. Aller dans le workspace

```bash
cd ~/Documents/ANDRA_2025-2026/ros2_ws
```

### 1.4. Compiler le workspace

```bash
colcon build
```

Si vous avez modifié uniquement le package `image_transfer`, vous pouvez compiler uniquement ce package :

```bash
colcon build --packages-select image_transfer
```

### 1.5. Sourcer le workspace compilé

```bash
source install/setup.bash
```
les deux "not found" sont des warning (donc a regler plus tard)

## 2. Démarrage des nœuds

Le fichier `navigation_stack.launch.py` lance tous les nœuds nécessaires en une seule commande.

```bash
cd ~/Documents/ANDRA_2025-2026/ros_launcher
ros2 launch navigation_stack.launch.py
```

**Ce fichier lance automatiquement :**
- Les drivers matériels (LIDAR, robot Scout, caméra ZED2)
- Les nœuds de traitement d'images (`image_publisher`, `image_subscriber`, `position_publisher`, `report_fissures`)
- Les transforms statiques (base_link → zed_camera_link, base_link → laser_frame)
- Le filtre EKF pour la localisation
- Le SLAM Toolbox (par défaut) ou AMCL (si spécifié)

**Paramètres disponibles :**
- **SLAM** (par défaut) : `ros2 launch navigation_stack.launch.py use_slam:=true`
- **AMCL avec carte** : `ros2 launch navigation_stack.launch.py use_slam:=false use_amcl:=true map_path:=wall_techlab.yaml`

### Option B : Lancement manuel des nœuds individuels

Si vous souhaitez lancer les nœuds individuellement pour le débogage :

#### Terminal 1 : LIDAR
```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/ANDRA_2025-2026/ros2_ws
source install/setup.bash

# Lancer le driver LIDAR
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# Dans un autre terminal : Rotation du LIDAR (si nécessaire)
ros2 run tf2_ros static_transform_publisher 0 0 0 1.57 0 0 base_link laser_frame
```

#### Terminal 2 : Robot Scout
```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/ANDRA_2025-2026/ros2_ws
source install/setup.bash

ros2 launch scout_base scout_mini_base.launch.py
```

#### Terminal 3 : Caméra ZED2
```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/ANDRA_2025-2026/ros2_ws
source install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

#### Terminal 4 : Nœuds de traitement d'images

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/ANDRA_2025-2026/ros2_ws
source install/setup.bash

# Publisher d'images (capture depuis la caméra PTZ toutes les 20 secondes)
ros2 run image_transfer image_publisher

# Subscriber d'images (détection YOLO des fissures)
ros2 run image_transfer image_subscriber

# Publisher de position
ros2 run image_transfer position_publisher

# Rapport des fissures (trace les positions sur la carte)
ros2 run image_transfer report_fissures
```

## 3. Vérification du système

### Vérifier les topics actifs

```bash
ros2 topic list
```

### Vérifier les messages publiés

```bash
# Images publiées
ros2 topic echo photo_topic

# Positions détectées
ros2 topic echo position_detectee

# Odométrie filtrée
ros2 topic echo /odometry/filtered
```

### Vérifier les transforms TF

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

## 4. Fichiers importants

- **Modèle YOLO** : `/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/models/best.pt`
- **Images détectées** : Sauvegardées dans `ros2_ws/images_detectees/`
- **Carte** : `ros_launcher/andra.yaml` (pour `report_fissures`)
- **Configurations** : `ros_launcher/ekf_config.yaml`, `slam_config.yaml`, `amcl_config.yaml`



