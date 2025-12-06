# ROS2 Docker - Guide d'utilisation

## Objectif

Ce dossier (`ros-docker/`) permet de travailler et tester votre code ROS2 sans avoir accès au robot physique. 

C'est l'environnement de développement local qui vous permet de :
- **Développer et tester vos nœuds ROS2** sur votre machine personnelle
- **Compiler votre code** avant de le déployer sur le robot
- **Visualiser avec RViz2** les données du robot (si connecté au même réseau)
- **Travailler sur n'importe quelle distribution Linux** (alors que ROS2 Humble est uniquement sur Ubuntu 22.04)

### Workflow recommandé

- **Développement local** (Docker) : Éditez et testez votre code dans Docker
- **Déploiement sur robot** : Utilisez les scripts dans `scripts/` pour compiler et lancer sur le robot

Pour lancer le système complet sur le robot (avec tous les drivers matériels), utilisez les scripts dans `scripts/` (voir [`DEMARRAGE_ROBOT.md`](../DEMARRAGE_ROBOT.md)).

## Prérequis

- **Docker** installé et fonctionnel
- **Environnement X11** (pour RViz2)
- **Pas besoin d'installer ROS2 nativement** sur votre système

## Installation (une seule fois)

```bash
cd ros-docker
docker build -t ros2-humble-custom .
```

Cette commande va :
- Télécharger l'image de base `ros:humble-ros-core-jammy` (Ubuntu 22.04)
- Installer tous les outils de développement ROS2
- Installer les dépendances Python (numpy<2, ultralytics, pillow, matplotlib)
- Installer les paquets ROS2 nécessaires (cv-bridge, image-geometry, rviz2)

## Structure des fichiers

### `Dockerfile`
Crée une image Docker complète avec :
- **Image de base** : `ros:humble-ros-core-jammy` (Ubuntu 22.04 + ROS2 Humble core)
- **Outils de développement** : build-essential, git, colcon, rosdep, vcstool
- **Paquets ROS2** : ros-base, cv-bridge, image-geometry, rviz2
- **Dépendances Python** : numpy<2 (compatibilité cv_bridge), ultralytics (YOLOv11), pillow, matplotlib
- **Outils multimédias** : opencv, ffmpeg

### `launch.sh`
Script de lancement du conteneur Docker avec :
- **Accès X11** : pour afficher RViz2 (`xhost +local:`)
- **Réseau hôte** : `--net=host` pour communiquer avec le robot (si sur le même réseau)
- **Volumes montés** :
  - `/tmp/.X11-unix` → accès à l'affichage graphique
  - `./config.rviz` → configuration RViz2
  - `../ros2_ws` → workspace ROS2 (monté en `/workspace/ros2_ws`)
  - `../ros_launcher` → fichiers de lancement (monté en `/workspace/ros_launcher`)

### `config.rviz`
Configuration pré-définie pour RViz2 qui affiche :
- **Grille** : référence visuelle
- **TF (Transform)** : arbre de coordonnées (map, odom, base_link, laser_frame, zed_camera_link)
- **Carte** : `/map` - carte générée par le SLAM
- **LaserScan** : `/scan` - données du LIDAR
- **PoseArray** : `/particlecloud` - particules de localisation (AMCL)
- **Polygon** : `/local_costmap/published_footprint` - empreinte du robot
- **Pose** : `/amcl_pose` - position estimée du robot
- **Outils de navigation** : SetInitialPose, SetGoal

## Utilisation

### Lancer le conteneur

```bash
cd ros-docker
./launch.sh
```

Cela va :
1. Autoriser l'accès X11
2. Lancer un conteneur interactif avec bash
3. Monter vos workspaces en volumes

### Dans le conteneur

**Workflow de développement :**

```bash
# 1. Initialiser ROS2
source /opt/ros/humble/setup.bash

# 2. Aller dans le workspace
cd /workspace/ros2_ws

# 3. Compiler le workspace (si nécessaire)
# Équivalent à ./scripts/build.sh andra sur le robot
colcon build

# 4. Sourcer le workspace compilé
source install/setup.bash

# 5. Tester vos nodes ROS2 individuellement
ros2 run image_transfer image_publisher
ros2 run image_transfer image_subscriber
# etc.
```

**Note** : Dans Docker, vous devez compiler manuellement avec `colcon build`. Les scripts `scripts/build.sh` et `scripts/setup.sh` sont conçus pour fonctionner directement sur le robot (voir [`DEMARRAGE_ROBOT.md`](../DEMARRAGE_ROBOT.md) pour le workflow complet sur le robot).

### Lancer RViz2

```bash
# Lancer directement avec la config
rviz2 -d /tmp/config.rviz
```

## Workflow de développement

### Édition des fichiers

**Vous pouvez éditer vos fichiers SANS lancer le conteneur** :
- Modifier les fichiers Python dans `ros2_ws/src/`
- Modifier les fichiers YAML dans `ros_launcher/`
- Les changements sont immédiatement visibles dans le conteneur (grâce aux volumes montés)

### Compilation et exécution

**Lancez le conteneur uniquement pour** :
- Compiler le projet (`colcon build`)
- Lancer des nodes ROS2 (`ros2 launch`, `ros2 run`)
- Visualiser avec RViz2
- Tester votre code localement

### Connexion au robot (optionnel)

Si le robot est sur le même réseau et que vous voulez visualiser ses données :
- Le conteneur utilise `--net=host` pour accéder au réseau
- Vous pouvez lancer RViz2 et vous connecter aux topics du robot
- Voir `DEMARRAGE_ROBOT.md` pour savoir comment lancer le système sur le robot

### Exécutables disponibles

Dans le package `image_transfer`, les exécutables sont :
- `image_publisher` (pas `publisher`)
- `image_subscriber`
- `test`
- `show_pos`
- `position_publisher`
- `report_fissures`

## À quoi sert RViz ?

RViz (Robot Visualization) est l'outil de visualisation 3D de ROS2 qui permet de :
- **Voir la carte** générée par le SLAM en temps réel
- **Visualiser les données des capteurs** (LIDAR, caméras)
- **Observer la position du robot** sur la carte
- **Définir des objectifs de navigation** (Set Goal)
- **Déboguer visuellement** le système robotique

## Liens utiles

- [Documentation ROS2 Humble](https://docs.ros.org/en/humble/)
- [Documentation Docker](https://docs.docker.com/)
- [Documentation RViz2](https://github.com/ros2/rviz)
- [Guide démarrage robot](../DEMARRAGE_ROBOT.md) 

