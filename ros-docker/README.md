# ROS2 Docker - Guide d'utilisation

Ce dossier contient la configuration Docker pour exécuter ROS2 Humble dans un conteneur isolé, permettant de travailler avec ROS2 sur n'importe quelle distribution Linux (alors que ROS2 Humble est officiellement supporté uniquement sur Ubuntu 22.04).

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
- **Réseau hôte** : `--net=host` pour communiquer directement avec le robot
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

```bash
# 1. Initialiser ROS2
source /opt/ros/humble/setup.bash

# 2. Aller dans le workspace
cd /workspace/ros2_ws

# 3. Compiler le workspace (si nécessaire)
colcon build

# 4. Sourcer le workspace compilé
source install/setup.bash

# 5. Lancer vos nodes ROS2
ros2 run image_transfer image_publisher
ros2 run image_transfer image_subscriber
# etc.
```

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
- Tester votre code

### Exécutables disponibles

Dans le package `image_transfer`, les exécutables sont :
- `image_publisher` (pas `publisher`)
- `image_subscriber`
- `test`
- `show_pos`
- `position_publisher`
- `report_fissures`

Pour lister tous les exécutables :
```bash
ros2 pkg executables image_transfer
```

## 🔍 À quoi sert RViz ?

RViz (Robot Visualization) est l'outil de visualisation 3D de ROS2 qui permet de :
- **Voir la carte** générée par le SLAM en temps réel
- **Visualiser les données des capteurs** (LIDAR, caméras)
- **Observer la position du robot** sur la carte
- **Définir des objectifs de navigation** (Set Goal)
- **Déboguer visuellement** le système robotique

C'est essentiel pour observer le robot en action et vérifier que tout fonctionne correctement.

## 🐛 Dépannage

### Erreur "No executable found"
- Vérifiez que vous avez compilé le workspace : `colcon build`
- Vérifiez que vous avez sourcé le workspace : `source install/setup.bash`
- Vérifiez le nom de l'exécutable : `ros2 pkg executables <package_name>`

### Erreur avec sudo
- Retirez `sudo` du script `launch.sh`
- Ajoutez votre utilisateur au groupe docker si nécessaire

### RViz2 ne s'affiche pas
- Vérifiez les permissions X11 : `xhost +local:`
- Vérifiez que `$DISPLAY` est défini : `echo $DISPLAY`
- Vérifiez que le volume X11 est monté dans `launch.sh`

## 📝 Notes importantes

- **Pas besoin d'installer ROS2 sur Debian 13** : tout fonctionne dans Docker
- **Les volumes montés** permettent d'éditer les fichiers localement sans relancer le conteneur
- **Le réseau hôte** (`--net=host`) permet la communication ROS2 directe avec le robot
- **NumPy < 2** est requis pour la compatibilité avec cv_bridge compilé avec NumPy 1.x

## 🔗 Liens utiles

- [Documentation ROS2 Humble](https://docs.ros.org/en/humble/)
- [Documentation Docker](https://docs.docker.com/)
- [Documentation RViz2](https://github.com/ros2/rviz)

