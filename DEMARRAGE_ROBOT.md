# Guide de démarrage des nœuds sur le robot

Ce document explique comment démarrer les nœuds ROS2 sur le robot Agilex Scout Mini.

## Démarage rapide

### 1. Connexion au robot

```bash
ssh techlab@192.168.40.99
```

### 2. Initialiser l'environnement

```bash
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
```

### 3. Compiler les workspaces (si nécessaire)

**Quand utiliser `build.sh` ?**
- **Première utilisation** : Après avoir cloné le projet ou récupéré le code
- **Après modification du code** : Si vous avez modifié des fichiers Python/C++ dans `ros2_ws/src/`
- **Après ajout de packages** : Si vous avez ajouté de nouveaux packages ROS2
- **Après mise à jour des dépendances** : Si les dépendances ont changé

**Compilation :**
```bash
# Compiler tous les workspaces
./scripts/build.sh

# Ou compiler uniquement un workspace spécifique
./scripts/build.sh ydlidar    # Uniquement YDLidar
./scripts/build.sh andra      # Uniquement ANDRA
```

**Après compilation, re-sourcer l'environnement :**
```bash
source scripts/setup.sh
```

### 4. Configurer CAN (une seule fois par session)

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 4. Lancer le système

**Mode SLAM** (pour créer une carte) :
```bash
./scripts/launch.sh slam
```

**Mode AMCL** (pour utiliser une carte existante) :
```bash
./scripts/launch.sh amcl ros_launcher/wall_techlab.yaml
```

## Structure du projet

```
~/Documents/ANDRA_2025-2026/
├── ros-doker/                  # Ce dossier contient l'environnement Docker ROS2 
├── ros2_ws/                    # Workspace principal (code robot, navigation, etc)
├── ros_launcher/               # Fichiers de lancement ROS2 (scripts .launch.py et cartes)
├── dependencies/               # Dépendances externes
│   ├── ydlidar_ros2_ws/        # Workspace ROS2 pour le lidar YDLidar
│   └── YDLidar-sdk/            # SDK C++ officiel du lidar YDLidar
├── script_video/               # Script d'enregistrement vidéo depuis la caméra du robot 
└── scripts/                    # Scripts d'initialisation et gestion du projet
    ├── setup.sh                # Préparation de tout l'environnement 
    ├── build.sh                # Compilation des workspaces
    └── launch.sh               # Lancement du système (navigation SLAM/AMCL)
```

## Scripts disponibles

### `scripts/setup.sh` - Initialisation de l'environnement

Sourcer tous les workspaces nécessaires :

```bash
source scripts/setup.sh
```

Ce script :
- ✅ Initialise ROS2 Humble
- ✅ Source le workspace YDLidar (si présent dans `dependencies/`)
- ✅ Source le workspace ANDRA
- ✅ Affiche les packages disponibles

### `scripts/build.sh` - Compilation des workspaces

Compiler les workspaces :

```bash
# Compiler tout
./scripts/build.sh 

# Compiler uniquement YDLidar
./scripts/build.sh ydlidar

# Compiler uniquement ANDRA
./scripts/build.sh andra
```

### `scripts/launch.sh` - Lancement du système

Lancer le système complet :

```bash
# Mode SLAM (cartographie)
./scripts/launch.sh slam

# Mode AMCL (localisation sur carte existante)
./scripts/launch.sh amcl ros_launcher/wall_techlab.yaml
```

## Vérifier les topics actifs

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

# Position AMCL
ros2 topic echo /amcl_pose
```

## Vérifier les transforms TF

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link zed_camera_link
ros2 run tf2_ros tf2_echo map odom
```

## Nœuds lancés automatiquement

Le fichier `navigation_stack.launch.py` lance automatiquement :

### Drivers matériels
- **LIDAR** : `ydlidar_ros2_driver` (scans laser) 
- **Robot Scout** : `scout_base` (odométrie des roues) 
- **Caméra ZED2** : `zed_wrapper` (images et données de profondeur) 

### Nœuds de traitement d'images
- **`image_publisher`** : Capture des images depuis la caméra PTZ toutes les 20 secondes
- **`image_subscriber`** : Détection YOLO des fissures, sauvegarde des images détectées
- **`position_publisher`** : Affichage de la position du robot
- **`report_fissures`** : Trace les positions détectées sur la carte

### Localisation et cartographie
- **EKF** : Filtre de Kalman étendu pour fusionner les données des capteurs
- **SLAM Toolbox** : En mode SLAM, construit la carte
- **AMCL** : En mode AMCL, localise le robot sur la carte

### Transforms statiques
- `base_link` → `zed_camera_link` (caméra ZED2)
- `base_link` → `laser_frame` (LIDAR, rotation de 90°)

## Lancement manuel des nœuds (débogage)

### Terminal 1 : LIDAR [en cours de test]

```bash
source scripts/setup.sh
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### Terminal 2 : Robot Scout [package manquant]

```bash
source scripts/setup.sh
ros2 launch scout_base scout_mini_base.launch.py
```

### Terminal 3 : Caméra ZED2 [package manquant]

```bash
source scripts/setup.sh
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### Terminal 4 : Nœuds de traitement d'images [fonctionnel]

```bash
source scripts/setup.sh

# Publisher d'images
ros2 run image_transfer image_publisher

# Subscriber d'images (détection YOLO)
ros2 run image_transfer image_subscriber

# Publisher de position
ros2 run image_transfer position_publisher

# Rapport des fissures
ros2 run image_transfer report_fissures
```


## Fichiers importants

- **Modèle YOLO** : `ros2_ws/models/best.pt`
- **Images détectées** : Sauvegardées dans `ros2_ws/images_detectees/`
- **Cartes** : `ros_launcher/*.yaml` et `ros_launcher/*.pgm`
- **Configurations** :
  - `ros_launcher/ekf_config.yaml` (filtre EKF)
  - `ros_launcher/slam_config.yaml` (SLAM)
  - `ros_launcher/amcl_config.yaml` (AMCL)

