# Guide de démarrage des nœuds sur le robot

Ce document explique comment démarrer les nœuds ROS2 sur le robot Agilex Scout Mini.

## Démarrage rapide

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
./scripts/build.sh ydlidar        # Uniquement YDLidar
./scripts/build.sh ros2_ws        # Uniquement ros2_ws
./scripts/build.sh scrout_base    # Uniquement scout_base
./scripts/build.sh zed            # Uniquement scout_base
```

**Après compilation, re-sourcer l'environnement :**
```bash
source scripts/setup.sh
```

### 4. Configurer le réseau PTZ (une seule fois par session)

**Important** : Après chaque redémarrage du robot, il faut reconfigurer le réseau pour accéder à la caméra PTZ.

```bash
# Configurer l'interface Ethernet pour la caméra PTZ
sudo bash scripts/ptz-network-setup.sh

# Vérifier la configuration
ip addr show enP8p1s0
# Doit afficher : inet 192.168.5.100/24

# Vérifier que la route passe par Ethernet (pas WiFi)
ip route show | grep 192.168.5
# Doit afficher : 192.168.5.0/24 dev enP8p1s0 ...

# Tester la connectivité
ping -c 3 192.168.5.163
```

**Note** : Ce script configure l'interface Ethernet `enP8p1s0` avec l'adresse IP statique `192.168.5.100/24` et supprime les routes WiFi conflictuelles. 

### 5. Configurer CAN (une seule fois par session)

Le TechLab utilise un service systemd pour configurer automatiquement l'interface CAN `agilex` :

```bash
# Vérifier que l'interface 'agilex' est UP
ip link show agilex
# Doit afficher : state UP

# Si DOWN, Activer et démarrer le service 
sudo systemctl enable agilex-handler.service
sudo systemctl start agilex-handler.service

# Vérifier la réception de messages CAN (si le robot est allumé)
candump agilex -n 5 -T 2000
```

**Note :** Vérifiez toujours que l'interface CAN est active avant de lancer le système. Si elle n'est pas active, scout_base ne pourra pas communiquer avec le robot.

### 6. Lancer le système

**Mode SLAM** (pour créer une carte) :
```bash
./scripts/launch.sh slam

# Lancer sans LIDAR
./scripts/launch.sh slam enable_lidar:=false

# Lancer sans Scout Base 
./scripts/launch.sh slam enable_scout:=false

# Lancer sans ZEDs
./scripts/launch.sh slam enable_zed:=false

# Lancer sans caméra PTZ
./scripts/launch.sh slam enable_ptz:=false

# Lancer avec mode automatique pour la correction d'image PTZ
./scripts/launch.sh slam ptz_auto_adjustment:=true

# Combinaisons possibles
./scripts/launch.sh slam enable_lidar:=false enable_scout:=false
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
│   ├── scout_base/             # Workspace ROS2 pour le robot Scout
│   └── zed-ros2-wrapper/       # Workspace ROS2 pour la caméra ZED2
├── video_launcher/             # Script d'enregistrement vidéo depuis la caméra PTZ
│   ├── script.sh              # Script interactif d'enregistrement avec retouches d'image
│   └── video_output/          # Dossier de sauvegarde des vidéos enregistrées
├── script_video/              # Script d'enregistrement vidéo depuis la caméra du robot (ancien)
└── scripts/                    # Scripts d'initialisation et gestion du projet
    ├── setup.sh                # Préparation de tout l'environnement 
    ├── build.sh                # Compilation des workspaces
    ├── launch.sh               # Lancement du système (navigation SLAM/AMCL)
    └── ptz-network-setup.sh    # Configuration réseau pour la caméra PTZ
```

## Scripts disponibles

### `scripts/setup.sh` - Initialisation de l'environnement

Sourcer tous les workspaces nécessaires :

```bash
source scripts/setup.sh
```

Ce script :
- ✅ Initialise ROS2 Humble
- ✅ Source le workspace YDLidar
- ✅ Source le workspace scout_base 
- ✅ Source le workspace ZED Wrapper 
- ✅ Source le workspace ANDRA
- ✅ Configure automatiquement le middleware DDS

### `scripts/build.sh` - Compilation des workspaces

Compiler les workspaces :

```bash
# Compiler tout
./scripts/build.sh 

# Compiler uniquement YDLidar
./scripts/build.sh ydlidar

# Compiler uniquement scout_base
./scripts/build.sh scout_base

# Compiler uniquement ZED Wrapper
./scripts/build.sh zed

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

### `scripts/ptz-network-setup.sh` - Configuration réseau PTZ

Configure automatiquement l'interface Ethernet pour accéder à la caméra PTZ :

```bash
# Exécuter le script (nécessite les droits sudo)
sudo bash scripts/ptz-network-setup.sh
```

Ce script :
- Configure l'adresse IP statique `192.168.5.100/24` sur `enP8p1s0`
- Supprime les routes WiFi conflictuelles vers `192.168.5.0/24`
- Vérifie que la route passe bien par Ethernet

**Note** : À exécuter après chaque redémarrage du robot pour accéder à la caméra PTZ.

### `video_launcher/script.sh` - Enregistrement vidéo depuis la caméra PTZ

Script interactif pour enregistrer des vidéos depuis le flux RTSP de la caméra PTZ avec retouches d'image :

```bash
cd ~/Documents/ANDRA_2025-2026
./video_launcher/script.sh
```

**Contrôles** :
- `r` : Démarrer/arrêter l'enregistrement (toggle)
- `s` : Arrêter l'enregistrement en cours
- `q` : Quitter le script (arrête aussi l'enregistrement en cours)

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
- **`image_publisher`** : Capture des images depuis la caméra PTZ
  - Intervalle de capture configurable (défaut: 10 secondes)
  - Ajustement automatique de luminosité/contraste/gamma
  - Sauvegarde de toutes les images dans `ros2_ws/images_capturees/`
  - Publication sur le topic `/photo_topic`
  - Paramètres configurables : `brightness`, `contrast`, `gamma`, `capture_interval`, `auto_adjustment_mode`
- **`image_subscriber`** : Détection YOLO des fissures, sauvegarde des images détectées
  - Reçoit les images depuis `/photo_topic`
  - Applique la détection YOLO avec le modèle `ros2_ws/models/best.pt`
  - Sauvegarde uniquement les images avec détection dans `ros2_ws/images_detectees/`
- **`position_publisher`** : Affichage de la position du robot
- **`report_fissures`** : Trace les positions détectées sur la carte
  - Reçoit les positions depuis le topic `/position_detectee`
  - Trace les points détectés sur la carte en utilisant le fichier YAML de la carte
  - Paramètre ROS2 configurable : `map_yaml_path` (défaut: `ros_launcher/map_results/andra.yaml`)
  - Sauvegarde les images avec timestamp : `map_with_point_YYYY-MM-DD_HH-MM-SS.png`
- **`ptz_controller`** : Contrôle PTZ de la caméra Marshall CV-605 via VISCA over IP

### Localisation et cartographie
- **EKF** : Filtre de Kalman étendu pour fusionner les données des capteurs
- **SLAM Toolbox** : En mode SLAM, construit la carte
- **AMCL** : En mode AMCL, localise le robot sur la carte

### Transforms statiques
- `base_link` → `zed_camera_link` (caméra ZED2)
- `base_link` → `laser_frame` (LIDAR, rotation de 90°)

## Lancement manuel des nœuds (débogage)

### Terminal 1 : LIDAR

```bash
source scripts/setup.sh
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### Terminal 2 : Robot Scout

```bash
source scripts/setup.sh
# Utiliser l'interface CAN 'agilex' (configuration TechLab)
# Note : Le code source a été modifié pour accepter "agilex" comme nom de port CAN valide
ros2 launch scout_base scout_mini_base.launch.py port_name:=agilex is_scout_mini:=True odom_topic_name:=odom_robot
```

### Terminal 3 : Caméra ZED2 

```bash
source scripts/setup.sh
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### Terminal 4 : Nœuds de traitement d'images

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

# Contrôleur PTZ
ros2 run image_transfer ptz_controller
```

## Notes techniques

### Configuration Middleware DDS

**Qu'est-ce que le middleware DDS ?**

Le middleware DDS (Data Distribution Service) est la couche de communication sous-jacente utilisée par ROS2 pour permettre aux nœuds de communiquer entre eux. C'est lui qui gère :
- La publication et la souscription aux topics ROS2
- La découverte automatique des nœuds sur le réseau
- La transmission des messages entre les nœuds
- La gestion de la qualité de service (QoS)

## Fichiers importants

- **Modèle YOLO** : `ros2_ws/models/best.pt`
- **Images détectées** : Sauvegardées dans `ros2_ws/images_detectees/`
- **Cartes** : `ros_launcher/*.yaml` et `ros_launcher/*.pgm`
- **Configurations** :
  - `ros_launcher/configs/ekf_config.yaml` (filtre EKF)
  - `ros_launcher/configs/slam_config.yaml` (SLAM)
  - `ros_launcher/configs/amcl_config.yaml` (AMCL)
  - `ros_launcher/navigation_stack.launch.py` (fichier de lancement principal)
- **Scripts** :
  - `scripts/setup.sh` (initialisation avec configuration automatique DDS et vérification CAN)
  - `scripts/build.sh` (compilation des workspaces)
  - `scripts/launch.sh` (lancement du système)
  - `scripts/ptz-network-setup.sh` (configuration réseau pour la caméra PTZ)
  - `video_launcher/script.sh` (enregistrement vidéo avec retouches d'image)

