# ROS2 Docker - Guide d'utilisation

## Objectif

**Avantage principal de ce dossier : Utiliser RViz2 et ROS2 sans installation native !**

C'est l'environnement de développement local qui vous permet de :
- **Visualiser avec RViz2** les cartes SLAM et les données du robot **sans installer ROS2** sur votre système
- **Développer et tester vos nœuds ROS2** sur votre machine personnelle 
- **Vérifier que votre code compile** avant de le déployer sur le robot
- **Travailler sur macOS, Windows ou Linux** (alors que ROS2 Humble n'est disponible nativement que sur Ubuntu 22.04)

## Prérequis

- **Docker** installé et fonctionnel
- **Environnement X11** (pour RViz2)
  - **Linux** : X11 natif (déjà installé)
  - **macOS** : XQuartz (voir section macOS ci-dessous)
  - **Windows** : VcXsrv ou X410 (voir section Windows ci-dessous)
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

### Scripts de lancement

#### `launch.sh` (Linux)
Script de lancement du conteneur Docker pour Linux avec :
- **Accès X11** : pour afficher RViz2 (`xhost +local:`)
- **Réseau hôte** : `--net=host` pour communiquer avec le robot (si sur le même réseau)
- **Volumes montés** :
  - `/tmp/.X11-unix` → accès à l'affichage graphique
  - `../ros2_ws` → workspace ROS2 (monté en `/workspace/ros2_ws`)
  - `../ros_launcher` → fichiers de lancement (monté en `/workspace/ros_launcher`)

#### `launch-macos.sh` (macOS)
Script adapté pour macOS avec :
- **Configuration XQuartz** : configure automatiquement l'accès X11 pour macOS
- **Même configuration réseau et volumes** que `launch.sh`

#### `launch-windows.sh` (Windows)
Script adapté pour Windows (WSL ou Git Bash) avec :
- **Configuration X11** : détecte automatiquement WSL et configure l'affichage
- **Support VcXsrv et X410** : configuration interactive pour le serveur X11
- **Même configuration réseau et volumes** que `launch.sh`

## Utilisation

### macOS

**Configuration requise :**

1. **Installer Docker Desktop** : [Télécharger Docker Desktop pour Mac](https://www.docker.com/products/docker-desktop/)
2. **Installer XQuartz** :
   ```bash
   brew install --cask xquartz
   # Ou télécharger depuis: https://www.xquartz.org/
   ```
3. **Configurer XQuartz** :
   - Ouvrir **Applications > Utilitaires > XQuartz**
   - **Préférences > Sécurité** → Cocher **"Autoriser les connexions depuis le réseau"**
   - Redémarrer XQuartz

**Lancer le conteneur :**
```bash
cd ros-docker
./launch-macos.sh
```

### Windows

**Configuration requise :**

1. **Installer Docker Desktop** : [Télécharger Docker Desktop pour Windows](https://www.docker.com/products/docker-desktop/)
2. **Installer un serveur X11** (choisir l'un des deux) :
   - **VcXsrv** : [Télécharger VcXsrv](https://sourceforge.net/projects/vcxsrv/)
   - **X410** : Disponible sur le Microsoft Store (payant mais plus stable)
3. **Configurer le serveur X11** :
   - **VcXsrv** : Lors du lancement, cocher "Disable access control"
   - **X410** : Suivre les instructions de configuration réseau

**Lancer le conteneur :**
```bash
# Dans WSL ou Git Bash
cd ros-docker
./launch-windows.sh
```

Le script détecte automatiquement si vous êtes dans WSL et configure l'affichage X11 en conséquence.

### Linux

**Lancer le conteneur :**
```bash
cd ros-docker
./launch.sh
```

### Dans le conteneur

**Workflow de développement :**

```bash
# 1. Initialiser ROS2 (déjà fait automatiquement par le script)
source /opt/ros/humble/setup.bash

# 2. Aller dans le workspace
cd /workspace/ros2_ws

# 3. Compiler le workspace (si nécessaire)
# Équivalent à ./scripts/build.sh andra sur le robot
colcon build

# 4. Sourcer le workspace compilé
source install/setup.bash

# 5. Lancer RViz2 pour visualiser les cartes SLAM
rviz2 -d /workspace/ros_launcher/config.rviz

# 6. Tester vos nodes ROS2 individuellement
ros2 run image_transfer image_publisher
ros2 run image_transfer image_subscriber
# etc.
```

**Utilisation principale : Visualiser les cartes SLAM avec RViz2**

Le fichier `config.rviz` est automatiquement monté depuis `ros_launcher/config.rviz` et contient toute la configuration nécessaire pour visualiser :
- La carte (`/map`) générée par SLAM
- Les scans LIDAR (`/scan`)
- Les transformations TF
- La position du robot
- Les particules AMCL (en mode localisation)

**Note** : Dans Docker, vous devez compiler manuellement avec `colcon build`. Car les scripts `scripts/build.sh` et `scripts/setup.sh` sont conçus pour fonctionner directement sur le robot.

## Cas d'utilisation

### Utilisation recommandée

- **Visualiser les cartes SLAM** créées par le robot depuis votre Mac/PC Windows
- **Déboguer visuellement** le système robotique avec RViz2
- **Tester la compilation** de votre code avant de le déployer sur le robot
- **Développer sur macOS/Windows** sans installer ROS2 nativement

### Limitations

- Développer dans un conteneur Docker offre des possibilités limitées par rapport à un accès direct au robot via SSH
- Pour le développement complet, il est recommandé d'utiliser SSH directement sur le robot
- Certaines fonctionnalités matérielles (accès direct aux périphériques) ne sont pas disponibles dans Docker

**Recommandation** : Utilisez ce conteneur principalement pour la **visualisation avec RViz2**. Pour le développement complet, connectez-vous directement au robot via SSH.

## Liens utiles

- [Documentation ROS2 Humble](https://docs.ros.org/en/humble/)
- [Documentation Docker](https://docs.docker.com/)

