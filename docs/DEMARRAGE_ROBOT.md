# Guide de démarrage des nœuds sur le robot

Ce document explique comment démarrer les nœuds ROS2 sur le robot Agilex Scout Mini.

## Démarrage rapide

### 1. Connexion au robot

Pour vous connecter au robot, vous devez être sur le même réseau que lui (soit via le routeur du Techlab, soit via le Hotspot du robot) le mot de passe de la connexion ssh est toujours `depinfonancy`.

## Au techlab (Techlab-wifi)

```bash
ssh techlab@192.168.40.101
# OU 
ssh techlab@orin2.local

```

**Note :** Si vous ne trouvez pas l'IP, scannez le réseau :
`nmap -sn 192.168.40.0/24` (Chercher "orin2") ou `nmap -Pn -p 22 192.168.40.0/24` pour forcer la reponse de tous les host 

## En dehors du techlab (HotspotRobot)

S'il ne capte pas le réseau du Techlab, le robot crée automatiquement son propre réseau WiFi.

Connectez votre ordinateur au WiFi du robot : **JetsonWIF** (Mdp : depinfonancy)

```bash
ssh techlab@orin2.local

```

**Requis :** Sur **macOS**, cela fonctionne nativement (Bonjour). Sur **Linux**, assurez-vous que `avahi-daemon` est installé.

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
./scripts/launch.sh amcl ros2_ws/src/ros_launcher/map_results/andra.yaml
```

#### `navigation_stack.launch.py` (pour configuration avancée)

**Avantages :**
- Contrôle total sur tous les paramètres ROS2
- Configuration avancée possible (ex: paramètres PTZ personnalisés)
- Utile pour le débogage et les tests

```bash
# Mode SLAM avec paramètres PTZ personnalisés
ros2 launch ros_launcher navigation_stack.launch.py \
  use_slam:=true \
  enable_ptz:=true \
  ptz_brightness:=2.0 \
  ptz_contrast:=1.5 \
  ptz_gamma:=0.8 \
  enable_image_transfer:=true

# Mode AMCL avec configuration personnalisée
ros2 launch ros_launcher navigation_stack.launch.py \
  use_slam:=false \
  use_amcl:=true \
  map_path:=ros2_ws/src/ros_launcher/map_results/andra.yaml 
```

### 7. Lancer le nœud image_subscriber avec GPU (dans un terminal séparé)

```bash
# Dans un nouveau terminal (après avoir lancé le système principal)
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
./scripts/image_subscriber_gpu.sh
```

**Ordre de lancement recommandé** :
1. **Terminal 1** : Lancer le système principal (`./scripts/launch.sh slam` ou `amcl`)
2. **Terminal 2** : Lancer `image_subscriber` avec GPU (`./scripts/image_subscriber_gpu.sh`)

### 8. Lancer le nœud sequence_photo (dans un terminal séparé)

```bash
# Dans un nouveau terminal (après avoir lancé le système principal)
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
ros2 run navigation_utils sequence_photo
```

**Ordre de lancement recommandé** :
1. **Terminal 1** : Lancer le système principal (`./scripts/launch.sh slam` ou `amcl`)
2. **Terminal 2** : Lancer `sequence_photo` (`ros2 run navigation_utils sequence_photo`)

### 9. Lancer le nœud sequence_video (dans un terminal séparé)

```bash
# Dans un nouveau terminal (après avoir lancé le système principal)
cd ~/Documents/ANDRA_2025-2026
source scripts/setup.sh
ros2 run navigation_utils sequence_video
```

**Ordre de lancement recommandé** :
1. **Terminal 1** : Lancer le système principal (`./scripts/launch.sh slam` ou `amcl`)
2. **Terminal 2** : Lancer `sequence_video` (`ros2 run navigation_utils sequence_video`)

## Nœuds lancés automatiquement

Que vous utilisiez `scripts/launch.sh` ou `navigation_stack.launch.py` directement, les mêmes nœuds sont lancés (car `launch.sh` appelle `navigation_stack.launch.py` en interne).

Le fichier `navigation_stack.launch.py` lance automatiquement :

### Drivers matériels
- **LIDAR** : `ydlidar_ros2_driver` (scans laser) 
- **Robot Scout** : `scout_base` (odométrie des roues) 
- **Caméra ZED2i** : `zed_wrapper` (images et données de profondeur) 

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
- **`video_publisher`** : Traitement des vidéos enregistrées et extraction d'images
  - Surveille le dossier `video/video_output/` toutes les 5 secondes pour détecter de nouvelles vidéos `.mp4`
  - Vérifie que les vidéos sont stables (pas en cours d'écriture) avant traitement
  - Valide les vidéos avant traitement (vérifie FPS et capacité de lecture)
  - Extrait des images à un taux configurable (`extract_rate`, défaut: 30 images/seconde)
  - Sauvegarde les images extraites dans `ros2_ws/images_capturees/` avec le préfixe `from_vid_`
  - Publie chaque image extraite sur le topic `/photo_topic` pour traitement par `image_subscriber`
  - Déplace les vidéos traitées vers `video/video_output/processed/` après traitement réussi
  - Déplace les vidéos corrompues vers `video/video_output/failed/` pour éviter les traitements répétés
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
  - Paramètre ROS2 configurable : `map_yaml_path` (défaut: `ros2_ws/src/ros_launcher/map_results/andra.yaml`)
  - Sauvegarde les images avec timestamp : `map_with_point_YYYY-MM-DD_HH-MM-SS.png`

### Localisation et cartographie

- **EKF** (`ekf_filter_node`) : Filtre de Kalman étendu pour fusionner les données des capteurs
  - Publie la transformation `odom` → `base_link`
  - Fusionne : odométrie des roues (`/odom_robot`), odométrie ZED2i (`/zed/zed_node/odom`), IMU ZED2i (`/zed/zed_node/imu/data`)
  
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

- **`base_to_zed_tf`** : Publie `base_link` → `zed_camera_link` (caméra ZED2i)
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

### Terminal 3 : Caméra ZED2i

```bash
source scripts/setup.sh
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### Terminal 4 : Nœuds de traitement d'images ou de navigation

```bash
source scripts/setup.sh

# Publisher d'images depuis la caméra PTZ
ros2 run image_transfer image_publisher

# Publisher de vidéos (traitement des vidéos enregistrées)
ros2 run image_transfer video_publisher

# Publisher de position
ros2 run image_transfer position_publisher

# Controle de la PTZ
ros2 run image_transfer ptz_controller

# Séquence photo (Step-and-Go, 5 captures PTZ)
ros2 run navigation_utils sequence_photo

# Séquence vidéo (enregistrement continu + balayage PTZ)
ros2 run navigation_utils sequence_video

# Rapport des fissures (trace sur la carte)
ros2 run navigation_utils report_fissures

# Afficher la position du robot
ros2 run navigation_utils show_pos

# Test de la carte
ros2 run navigation_utils test
```

