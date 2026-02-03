# Guide de démarrage des nœuds sur le robot

Ce document explique comment démarrer les nœuds ROS2 sur le robot Agilex Scout Mini.

## Démarrage rapide

### 1. Connexion au robot

Pour vous connecter au robot, vous devez être sur le même réseau WIFI que lui.

## Au techlab
Si l'adresse ip n'a pas changé, vous pouvez vous connecter avec via le réseau wifi : Techlab-wifi

```bash
ssh techlab@192.168.40.101
```

**Note :** Vérifier avec nmap -sn 192.168.40.0/24 et chercher orin2 ou nmap -Pn -p 22 192.168.40.0/24 pour forcer la reponse de tous les host

## En dehors du techlab

Vous devez avoir le service avahi installé sur votre ordinateur.
Vous pouvez alors effectuer la commande :

```bash
ssh techlab@orin2.local
```

**Note :** Si l'adresse IP fonctionne mais pas le nom du robot, vérifiez qu'Avahi est aussi installé sur le robot.


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
./scripts/build.sh zed            # Uniquement zed
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

**Note** : Ce script configure l'interface Ethernet `enP8p1s0` de la orin2 avec l'adresse IP statique `192.168.5.100/24` et supprime les routes WiFi conflictuelles. 

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

**Note :** Vérifiez toujours que l'interface CAN est active avant de lancer le système. Si elle n'est pas active, le neoud s'arrête sans faire d'erreur.

### 6. Lancer le système

Il existe deux façons de lancer le système :

#### `scripts/launch.sh` (recommandé pour l'usage normal)

**Avantages :**
- Syntaxe simplifiée avec deux modes (`slam` / `amcl`)
- Vérifications automatiques (carte obligatoire pour AMCL)
- Messages d'aide affichés
- Gestion automatique du répertoire de travail

```bash
# Mode SLAM simple (par défault)
./scripts/launch.sh slam

# Mode SLAM avec options
./scripts/launch.sh slam enable_lidar:=false enable_zed:=false

# Mode AMCL (vérifie automatiquement que la carte est fournie)
./scripts/launch.sh amcl ros_launcher/andra.yaml
```

#### `navigation_stack.launch.py` (pour configuration avancée)

**Avantages :**
- Contrôle total sur tous les paramètres ROS2
- Configuration avancée possible (ex: paramètres PTZ personnalisés)
- Utile pour le débogage et les tests

```bash
# Depuis le répertoire ros_launcher
cd ros_launcher

# Mode SLAM avec paramètres PTZ personnalisés
ros2 launch navigation_stack.launch.py \
  use_slam:=true \
  enable_ptz:=true \
  ptz_brightness:=2.0 \
  ptz_contrast:=1.5 \
  ptz_gamma:=0.8 \
  enable_image_transfer:=true

# Mode AMCL avec configuration personnalisée
ros2 launch navigation_stack.launch.py \
  use_slam:=false \
  use_amcl:=true \
  map_path:=ros_launcher/andra.yaml 
```

## Nœuds lancés automatiquement

Que vous utilisiez `scripts/launch.sh` ou `navigation_stack.launch.py` directement, les mêmes nœuds sont lancés (car `launch.sh` appelle `navigation_stack.launch.py` en interne).

Le fichier `navigation_stack.launch.py` lance automatiquement :

### Drivers matériels
- **LIDAR** : `ydlidar_ros2_driver` (scans laser) 
- **Robot Scout** : `scout_base` (odométrie des roues) 
- **Caméra ZED2** : `zed_wrapper` (images et données de profondeur) 

### Nœuds de traitement d'images (image_transfer)
- **`image_publisher`** : Capture des images depuis la caméra PTZ
  - Active le mode auto exposure au démarrage (via VISCA)
  - Capture automatique optionnelle (désactivée par défaut)
  - Capture à la demande via le topic `/trigger_capture` (String)
  - Ajustement manuel de luminosité/contraste/gamma (activé avec `enable_image_adjustment:=true`)
  - Sauvegarde de toutes les images dans `ros2_ws/images_capturees/`
  - Publication sur le topic `/photo_topic`
- **`ptz_controller`** : Contrôle de la caméra PTZ Marshall CV-605
  - Gère les commandes de mouvement via `/ptz/cmd_vel` (geometry_msgs/Twist)
  - Gère le preset Home (-1) via `/ptz/preset` (std_msgs/Int32)
  - Communication VISCA avec la caméra (192.168.5.163:1259)
- **`image_subscriber`** : Détection YOLO des fissures, sauvegarde des images détectées
  - Reçoit les images depuis `/photo_topic`
  - Applique la détection YOLO avec le modèle `ros2_ws/models/best.pt`
  - Sauvegarde uniquement les images avec détection dans `ros2_ws/images_detectees/`
- **`position_publisher`** : Affichage de la position du robot lors des détections
  - S'abonne au topic `/odometry/filtered` pour obtenir la position du robot (publiée par EKF)
  - S'abonne au topic `detection_status` (Bool) pour être notifié lorsqu'une fissure est détectée
  - Affiche dans les logs la position (x, y, z) du robot au moment où une détection se produit

### Nœuds de navigation et visualisation (navigation_utils)
- **`odom_to_path`** : Visualisation du trajet du robot
  - S'abonne au topic `/odometry/filtered` pour obtenir la position du robot (publiée par EKF)
  - Publie le Path sur `/robot_path` pour visualiser le trajet parcouru dans RViz2
  - Publie la transformation TF `odom` → `base_link` pour compléter l'arbre de transformations
  - Ajoute un point au Path tous les 10 cm minimum (`min_distance = 0.1`)
- **`report_fissures`** : Trace les positions détectées sur la carte
  - Reçoit les positions depuis le topic `/position_detectee`
  - Trace les points détectés sur la carte en utilisant le fichier YAML de la carte
  - Paramètre ROS2 configurable : `map_yaml_path` (défaut: `ros_launcher/map_results/andra.yaml`)
  - Sauvegarde les images avec timestamp : `map_with_point_YYYY-MM-DD_HH-MM-SS.png`

**Note** : Le nœud `sequence_robot` n'est **pas** lancé automatiquement. Il doit être lancé manuellement pour automatiser la séquence de mouvement et captures PTZ.

### Localisation et cartographie

- **EKF** (`ekf_filter_node`) : Filtre de Kalman étendu pour fusionner les données des capteurs
  - Publie la transformation `odom` → `base_link`
  - Fusionne : odométrie des roues (`/odom_robot`), odométrie ZED2 (`/zed/zed_node/odom`), IMU ZED2 (`/zed/zed_node/imu/data`)
  
- **SLAM Toolbox** (`slam_toolbox`) : En mode SLAM, construit la carte
  - Publie la transformation `map` → `odom`
  - Génère la carte à partir des scans LIDAR

- **AMCL** (uniquement en mode AMCL) : Localise le robot sur une carte existante
  - **`map_server`** (`nav2_map_server`) : Charge la carte depuis le fichier YAML spécifié
  - **`amcl`** (`nav2_amcl`) : Localise le robot sur la carte en utilisant les scans LIDAR
    - Publie la transformation `map` → `odom` (remplace `map_to_odom_fallback`)
  - **`lifecycle_manager_localization`** (`nav2_lifecycle_manager`) : Gère le cycle de vie des nœuds AMCL et map_server
    - Assure que les nœuds démarrent dans le bon ordre

### Transforms statiques

Les transformations statiques sont publiées par des nœuds `static_transform_publisher` :

- **`base_to_zed_tf`** : Publie `base_link` → `zed_camera_link` (caméra ZED2)
  - Transformation : (0, 0, 0, 0, 0, 0) - pas de translation ni rotation
  
- **`base_to_laser_tf`** : Publie `base_link` → `laser_frame` (LIDAR)
  - Transformation : (0, 0, 0, 0, 0, 1.57) - rotation de 90° autour de l'axe Z
  
- **`map_to_odom_fallback`** : Publie `map` → `odom` (uniquement en mode AMCL)
  - Transformation temporaire (0, 0, 0, 0, 0, 0) remplacée par AMCL une fois actif

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

### Terminal 4 : Nœuds de traitement d'images ou de navigation

```bash
source scripts/setup.sh

# Publisher d'images
ros2 run image_transfer image_publisher

# Subscriber d'images (détection YOLO)
ros2 run image_transfer image_subscriber

# Publisher de position
ros2 run image_transfer position_publisher

# Controle de la PTZ
ros2 run image_transfer ptz_controller

# Séquence de base
ros2 run navigation_utils sequence_robot

# Rapport des fissures (trace sur la carte)
ros2 run navigation_utils report_fissures

# Afficher la position du robot
ros2 run navigation_utils show_pos

# Test de la carte
ros2 run navigation_utils test
```

